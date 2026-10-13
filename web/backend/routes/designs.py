# -*- coding: utf-8 -*-
"""/api/designs — 설계 목록(트리) · 설계 하나(3D 보기) · 조립 기록 · 설계 규칙 숫자 + AI 생성(고른 설계 바탕 후보 3개 · 고르기) · 스캔 [저장]
(web/README 2 · 4장, SDD 6.6 · 6.8, W111 · W108).

읽기와 저장은 DesignStore 한 곳, AI 는 DesignGenerator 한 곳. 생성은 오래 걸려(≤ 60초) 뒤 스레드로 돌리고 진행은 /ws {type: gen} 으로 민다.
트리는 화면이 목록의 parent_id 로 만든다(따로 /tree 를 두지 않는다 — 같은 내용을 두 번 내보내지 않게).
"""
import threading
import uuid
from collections import OrderedDict
from typing import Literal

from design_gen import GenError, scan_actions
from design_store import split_design_id
from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

router = APIRouter(prefix='/api/designs')
JOBS_KEEP = 20      # 들고 있는 생성 작업 수 — 넘으면 오래된 것부터 버린다(후보는 메모리에만, E-84 ⑥)


def slim(cand):
    """화면에 보낼 후보 — 블록(3D 미리보기) · 검사 결과 요약. 레시피 · 조립 방법(크다)은 backend 에만 둔다."""
    out = {'idea': cand['idea'], 'parts': cand['parts']}
    if 'check' in cand:
        out['check'] = {k: cand['check'][k] for k in ('ok', 'reason', 'min_margin_mm', 'errors')}
    return out


class GenJobs:
    """AI 생성 작업 — 후보 3개를 backend 메모리에 들고 사람이 고를 때까지 기다린다(새로고침 · 재시작하면 사라져도 된다, E-84 ⑥).

    한 번에 하나만 돈다: 두 요청이 같은 새 ID(Template 다음 V)로 검사 · 저장하지 않게. 진행은 publish({type: gen, data}) 로 화면에 민다.
    바깥 영향: OpenAI 호출 · check_design 요청(뒤 스레드), 고르면 저장. 로봇은 움직이지 않는다.
    """

    def __init__(self, gen, publish):
        """gen = DesignGenerator, publish = WsHub.publish(스레드 안전)."""
        self.gen, self.publish = gen, publish
        self.jobs = OrderedDict()
        self.lock = threading.Lock()

    def start(self, text, base_id):
        """생성 하나를 뒤 스레드로 시작하고 job_id 를 돌려준다. base_id = 사람이 설계 목록에서 고른 설계(= 부모 — 10/13 PL).
        이미 도는 것이 있으면 RuntimeError(화면은 '만드는 중')."""
        with self.lock:
            if any(j['state'] == 'running' for j in self.jobs.values()):
                raise RuntimeError('이미 설계를 만드는 중이에요')
            job_id = uuid.uuid4().hex[:8]
            self.jobs[job_id] = {'job_id': job_id, 'state': 'running', 'text': text, 'base_id': base_id,
                                 'template': split_design_id(base_id)[0], 'candidates': []}
            while len(self.jobs) > JOBS_KEEP:
                self.jobs.popitem(last=False)
        threading.Thread(target=self._run, args=(job_id,), daemon=True).start()
        return job_id

    def start_scan(self, scan_result):
        """스캔 [저장] 하나를 뒤 스레드로 시작하고 job_id 를 돌려준다(IRD 8.4 ⑦ — AI 채우기 · 검사 · 저장, 최대 3번 부름).

        생성과 같은 줄에 선다(한 번에 하나) — 둘이 같은 Template 의 다음 V 번호를 같이 받지 않게(설계_DB 5절 · 8절 C).
        진행은 publish({type: scan_save, data}) — 생성 화면(type gen)과 섞이지 않게. 이미 도는 것이 있으면 RuntimeError.
        """
        with self.lock:
            if any(j['state'] == 'running' for j in self.jobs.values()):
                raise RuntimeError('이미 설계를 만들거나 저장하는 중이에요')
            job_id = uuid.uuid4().hex[:8]
            self.jobs[job_id] = {'job_id': job_id, 'state': 'running', 'kind': 'scan', 'run_id': scan_result.get('run_id'),
                                 'text': '', 'template': None, 'candidates': []}
            while len(self.jobs) > JOBS_KEEP:
                self.jobs.popitem(last=False)
        threading.Thread(target=self._run_scan, args=(job_id, scan_result), daemon=True).start()
        return job_id

    def _run_scan(self, job_id, scan_result):
        """(뒤 스레드) save_scan → 저장한 설계 또는 실패(code · message · 버튼 actions)를 job 에 적고 화면에 알린다.
        어떤 예외든 job 을 failed 로 끝낸다(화면이 '저장하는 중'에 머물지 않게)."""
        job = self.jobs[job_id]

        def progress(stage, info):
            if 'attempt' in info:
                job['attempt'] = info['attempt']
            self.publish({'type': 'scan_save', 'data': {'job_id': job_id, 'stage': stage, **info}})
        try:
            rec = self.gen.save_scan(scan_result, on_progress=progress)
            job.update(state='saved', saved={'design_id': rec['design_id'], 'version': rec['version'], 'parent_id': rec['parent_id']})
        except GenError as e:
            job.update(state='failed', code=e.code, message=e.message, detail=e.detail, actions=scan_actions(e.detail))
        except Exception as e:   # noqa: BLE001 — 저장소 · 검사 경로 고장도 화면에 실패로
            job.update(state='failed', code='ERROR', message=f'스캔 저장 중 오류({type(e).__name__})', detail=[],
                       actions=scan_actions([]))
        self.publish({'type': 'scan_save', 'data': {'stage': job['state'], **self.view(job_id)}})

    def _run(self, job_id):
        """(뒤 스레드) generate → 결과 · 실패를 job 에 적고 화면에 알린다. 어떤 예외든 job 을 failed 로 끝낸다(화면이 '만드는 중'에 머물지 않게)."""
        job = self.jobs[job_id]

        def progress(stage, info):
            # job 에도 그때그때 적는다 — 화면은 알림이 올 때마다 GET 으로 job 전체를 다시 받아 알림이 몰려와도 빠짐없이 그린다.
            # 후보 그림은 끝난 뒤 통과한 것만(10/11 PL A5 — 먼저 보여 주기 없음), 도는 동안은 수만(검사할 수 · 통과 수 · 더 만들 수)
            data = dict(info)
            if stage == 'reading':
                job.setdefault('reading', []).append(data.get('design_id'))
            for k in ('attempt', 'count', 'passed', 'need'):
                if k in data:
                    job[k] = data[k]
            if 'check' in data:
                data['check'] = {k: data['check'][k] for k in ('ok', 'reason', 'min_margin_mm', 'errors')}
            self.publish({'type': 'gen', 'data': {'job_id': job_id, 'stage': stage, **data}})
        try:
            result = self.gen.generate(job['text'], job['base_id'], on_progress=progress)
            job.update(state='ready', result=result, candidates=[slim(c) for c in result['candidates']])
        except GenError as e:
            job.update(state='failed', code=e.code, message=e.message, detail=e.detail)
        except Exception as e:   # noqa: BLE001 — 저장소 · 검사 경로 고장도 화면에 실패로
            job.update(state='failed', code='ERROR', message=f'생성 중 오류({type(e).__name__})', detail=[])
        self.publish({'type': 'gen', 'data': {'stage': 'done' if job['state'] == 'ready' else 'failed', **self.view(job_id)}})

    def view(self, job_id):
        """화면에 보낼 job 모습(후보는 slim). 없으면 KeyError."""
        job = self.jobs[job_id]
        out = {k: job[k] for k in ('job_id', 'state', 'text', 'template', 'candidates')}
        if 'result' in job:
            out.update({k: job['result'][k] for k in ('design_id', 'parent_id', 'reference', 'attempts', 'elapsed_s')})
        for k in ('base_id', 'reading', 'attempt', 'count', 'passed', 'need', 'code', 'message', 'detail', 'saved', 'kind', 'run_id',
                  'actions'):
            if k in job:
                out[k] = job[k]
        return out

    def pick(self, job_id, index, made_by):
        """사람이 고른 합격 후보를 저장한다(IRD 8.3 ⑥). 반환: 저장한 설계 요약. 없는 job · 아직 안 끝남 · 떨어진 후보 · 이미 저장은 ValueError."""
        job = self.jobs.get(job_id)
        if job is None or job['state'] != 'ready':
            raise ValueError('고를 수 있는 생성 결과가 없어요(없거나 아직 만드는 중 · 이미 저장)')
        rec = self.gen.save(job['result'], index, made_by)
        job.update(state='saved', saved={'design_id': rec['design_id'], 'version': rec['version'], 'parent_id': rec['parent_id'],
                                         'index': index})
        self.publish({'type': 'gen', 'data': {'stage': 'saved', **self.view(job_id)}})
        return job['saved']


class GenBody(BaseModel):
    """요청 문장(글상자 · 음성 문장) + 사람이 설계 목록에서 고른 설계 ID(= 부모, Template 은 그 앞부분 — 10/13 PL)."""
    text: str
    design_id: str


class PickBody(BaseModel):
    """고른 후보 번호(0부터) · 만든 방법(글상자 web · 음성 voice)."""
    index: int
    made_by: Literal['web', 'voice'] = 'web'


@router.post('/generate')
def generate(body: GenBody, request: Request):
    """고른 설계를 바탕으로 후보 3개 만들기 시작 → {job_id}. 진행 · 결과는 /ws {type: gen} 과 GET /generate/{job_id}.
    문장이 비었으면 400, 고른 설계가 저장소에 없으면 404(만들기 전에 거름), 이미 만드는 중이면 409."""
    text = body.text.strip()
    if not text:
        raise HTTPException(400, '요청 문장이 비었어요')
    try:
        request.app.state.store.get_design(body.design_id)
    except (KeyError, ValueError):
        raise HTTPException(404, f'고른 설계가 없어요: {body.design_id}') from None
    try:
        return {'job_id': request.app.state.jobs.start(text, body.design_id)}
    except RuntimeError as e:
        raise HTTPException(409, str(e)) from None


@router.get('/generate/{job_id}')
def generate_state(job_id: str, request: Request):
    """생성 작업 하나의 지금 모습(새로고침 뒤 다시 그릴 때). 없으면 404."""
    try:
        return request.app.state.jobs.view(job_id)
    except KeyError:
        raise HTTPException(404, '그 생성 작업이 없어요(재시작 · 오래됨)') from None


@router.post('/generate/{job_id}/pick')
def pick(job_id: str, body: PickBody, request: Request):
    """고른 후보 저장 → {design_id, version, parent_id, index}. 고를 수 없으면 409(이유 그대로)."""
    try:
        return request.app.state.jobs.pick(job_id, body.index, body.made_by)
    except ValueError as e:
        raise HTTPException(409, str(e)) from None


class ScanSaveBody(BaseModel):
    """[그대로 저장]을 누른 스캔의 run_id — 화면이 본 스캔과 backend 가 들고 있는 마지막 스캔이 같은지 확인한다."""
    run_id: str


@router.post('/scan/save')
def scan_save(body: ScanSaveBody, request: Request):
    """스캔 [그대로 저장](IRD 8.4 ⑦ · E-80) 시작 → {job_id}. 진행 · 결과는 /ws {type: scan_save} 와 GET /scan/save/{job_id}.
    마지막 scan_result 가 없거나 run_id 가 다르면 409(화면이 옛 스캔을 저장하지 않게), 이미 만들거나 저장하는 중이면 409."""
    scan = request.app.state.mqtt.snapshot().get('scan_result')
    if not isinstance(scan, dict) or scan.get('run_id') != body.run_id:
        raise HTTPException(409, '저장할 스캔 결과가 없거나 지금 스캔과 달라요 — 다시 스캔해 주세요')
    try:
        return {'job_id': request.app.state.jobs.start_scan(scan)}
    except RuntimeError as e:
        raise HTTPException(409, str(e)) from None


@router.get('/scan/save/{job_id}')
def scan_save_state(job_id: str, request: Request):
    """스캔 저장 작업 하나의 지금 모습 — 실패면 code · message · actions(['cancel'] 또는 ['rescan', 'cancel']). 없으면 404."""
    try:
        return request.app.state.jobs.view(job_id)
    except KeyError:
        raise HTTPException(404, '그 저장 작업이 없어요(재시작 · 오래됨)') from None


@router.get('/scan/recommend')
def scan_recommend(run_id: str, request: Request):
    """스캔 '추천 설계'(10/13 플랜 B 1단계) → {run_id, candidates[{design_id, family, made_by, matched, scan_count, part_count, rate}]}.
    마지막 scan_result 와 저장된 설계 전부의 일치율(비전 match_rate 와 같은 식) 높은 순 3개. 화면이 고르면 그 ID 로 select_design 을
    보낸다 — 이미 검사를 통과한 저장 설계라 AI · 새 검사 · 새 저장이 없다. 읽기만 한다.
    마지막 scan_result 가 없거나 run_id 가 다르면 409(옛 스캔으로 추천하지 않게), 스캔 블록 모양이 틀려도 409."""
    scan = request.app.state.mqtt.snapshot().get('scan_result')
    if not isinstance(scan, dict) or scan.get('run_id') != run_id:
        raise HTTPException(409, '추천할 스캔 결과가 없거나 지금 스캔과 달라요 — 다시 스캔해 주세요')
    try:
        return {'run_id': run_id, 'candidates': request.app.state.store.match_scan(scan['blocks']['blocks'])}
    except (KeyError, TypeError) as e:
        raise HTTPException(409, f'스캔 결과 모양이 틀렸어요({type(e).__name__})') from None


@router.get('')
def list_designs(request: Request, family: str | None = None):
    """설계 요약 목록 [{design_id, family, version, parent_id, made_by, block_count, size_mm, last_build}]. family 로 거를 수 있다."""
    return request.app.state.store.list_designs(family or None)    # ?family= 빈 값은 거르지 않음(빈 목록이 되지 않게)


@router.get('/rules')
def rules(request: Request):
    """화면이 쓰는 설계 규칙 숫자(robot.yaml 에서 backend 가 읽은 값 — 10/9 PL E-78). 3D 블록 크기 · 작업공간(mm)."""
    r = request.app.state.rules
    return {'block_size_mm': [round(v * 1000, 3) for v in r['block_size_m']],
            'assembly_area_half_mm': round(r['assembly_area_half_m'] * 1000, 3)}


@router.get('/templates')
def templates(request: Request):
    """모양(Template) 이름 · 설명 — 화면이 설계 ID 대신 '벤치 V001' 처럼 보여 줄 때 쓴다(이름 = V000 레시피 template.name).
    /{design_id} 보다 먼저 둔다(아니면 'templates' 를 설계 ID 로 읽는다)."""
    return [{'id': k, 'name': v['name'], 'label': v['label']} for k, v in request.app.state.gen.templates().items()]


@router.get('/{design_id}')
def get_design(design_id: str, request: Request):
    """design/3.0 하나. 없으면 404, 저장된 형식이 다르면 409(옛 형식은 변환하지 않는다 — E-69)."""
    try:
        return request.app.state.store.get_design(design_id)
    except KeyError:
        raise HTTPException(404, f'{design_id} 설계가 없다') from None
    except ValueError as e:
        raise HTTPException(409, str(e)) from None


@router.get('/{design_id}/builds')
def builds(design_id: str, request: Request):
    """이 설계의 조립 기록(build/2 전부, 최근 것 먼저). 설계가 없으면 404 — 기록이 없으면 빈 목록."""
    store = request.app.state.store
    try:
        store.get_design(design_id)
    except KeyError:
        raise HTTPException(404, f'{design_id} 설계가 없다') from None
    except ValueError as e:
        raise HTTPException(409, str(e)) from None
    return store.builds_for(design_id)
