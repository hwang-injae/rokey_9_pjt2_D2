# -*- coding: utf-8 -*-
"""/api/designs — 설계 목록(트리) · 설계 하나(3D 보기) · 조립 기록 · 설계 규칙 숫자 + AI 생성(Template 고르기 · 후보 3개 · 고르기)
(web/README 2 · 4장, SDD 6.6 · 6.8, W111 · W108).

읽기와 저장은 DesignStore 한 곳, GPT 는 DesignGenerator 한 곳. 생성은 오래 걸려(≤ 60초) 뒤 스레드로 돌리고 진행은 /ws {type: gen} 으로 민다.
트리는 화면이 목록의 parent_id 로 만든다(따로 /tree 를 두지 않는다 — 같은 내용을 두 번 내보내지 않게).
"""
import threading
import uuid
from collections import OrderedDict
from typing import Literal

from design_gen import GenError
from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

router = APIRouter(prefix='/api/designs')
JOBS_KEEP = 20      # 들고 있는 생성 작업 수 — 넘으면 오래된 것부터 버린다(후보는 메모리에만, E-84 ⑥)


def slim(cand):
    """화면에 보낼 후보 — 블록(3D 미리보기) · 검사 결과 요약. 레시피 · 조립 방법(크다)은 backend 에만 둔다."""
    out = {'idea': cand['idea'], 'blocks': cand['blocks']}
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

    def start(self, text, template):
        """생성 하나를 뒤 스레드로 시작하고 job_id 를 돌려준다. 이미 도는 것이 있으면 RuntimeError(화면은 '만드는 중')."""
        with self.lock:
            if any(j['state'] == 'running' for j in self.jobs.values()):
                raise RuntimeError('이미 설계를 만드는 중이에요')
            job_id = uuid.uuid4().hex[:8]
            self.jobs[job_id] = {'job_id': job_id, 'state': 'running', 'text': text, 'template': template, 'candidates': []}
            while len(self.jobs) > JOBS_KEEP:
                self.jobs.popitem(last=False)
        threading.Thread(target=self._run, args=(job_id,), daemon=True).start()
        return job_id

    def _run(self, job_id):
        """(뒤 스레드) generate → 결과 · 실패를 job 에 적고 화면에 알린다. 어떤 예외든 job 을 failed 로 끝낸다(화면이 '만드는 중'에 머물지 않게)."""
        job = self.jobs[job_id]

        def progress(stage, info):
            # job 에도 그때그때 적는다 — 화면은 알림이 올 때마다 GET 으로 job 전체를 다시 받아 알림이 몰려와도 빠짐없이 그린다
            data = dict(info)
            if stage == 'reading':
                job.setdefault('reading', []).append(data.get('design_id'))
            if 'attempt' in data:
                job['attempt'] = data['attempt']
            if 'candidates' in data:
                data['candidates'] = [slim(c) for c in data['candidates']]
                job['candidates'] = data['candidates']
            if 'check' in data:
                data['check'] = {k: data['check'][k] for k in ('ok', 'reason', 'min_margin_mm', 'errors')}
                if 0 <= data.get('index', -1) < len(job['candidates']):
                    job['candidates'][data['index']] = {**job['candidates'][data['index']], 'check': data['check']}
            self.publish({'type': 'gen', 'data': {'job_id': job_id, 'stage': stage, **data}})
        try:
            result = self.gen.generate(job['text'], job['template'], on_progress=progress)
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
        for k in ('reading', 'attempt', 'code', 'message', 'detail', 'saved'):
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


class TextBody(BaseModel):
    """요청 문장(글상자 · 음성을 사람이 확인한 것)."""
    text: str


class GenBody(BaseModel):
    """요청 문장 + 사용자가 확인한 Template(E-84 ③)."""
    text: str
    template: str


class PickBody(BaseModel):
    """고른 후보 번호(0부터) · 만든 방법(글상자 web · 음성 voice)."""
    index: int
    made_by: Literal['web', 'voice'] = 'web'


@router.post('/template')
def choose_template(body: TextBody, request: Request):
    """요청 문장 → AI 가 고른 Template + 이유 + 고를 수 있는 Template 목록(사용자가 화면에서 한 번 확인 · 바꿈 — W163).
    template null = 범위 밖 안내(OUT_OF_SCOPE)."""
    text = body.text.strip()
    if not text:
        raise HTTPException(400, '요청 문장이 비었어요')
    gen = request.app.state.gen
    templates = [{'id': k, 'label': v} for k, v in gen.templates().items()]   # 실패해도 준다 — AI 가 못 고르면 사람이 직접 고르게
    try:
        choice = gen.classify(text)
    except GenError as e:
        return {'success': False, 'code': e.code, 'message': e.message, 'templates': templates}
    return {'success': True, **choice, 'templates': templates}


@router.post('/generate')
def generate(body: GenBody, request: Request):
    """후보 3개 만들기 시작 → {job_id}. 진행 · 결과는 /ws {type: gen} 과 GET /generate/{job_id}. 이미 만드는 중이면 409."""
    text = body.text.strip()
    if not text:
        raise HTTPException(400, '요청 문장이 비었어요')
    try:
        return {'job_id': request.app.state.jobs.start(text, body.template)}
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


@router.get('/{design_id}')
def get_design(design_id: str, request: Request):
    """design/2.0 하나. 없으면 404, 저장된 형식이 다르면 409(옛 형식은 변환하지 않는다 — E-69)."""
    try:
        return request.app.state.store.get_design(design_id)
    except KeyError:
        raise HTTPException(404, f'{design_id} 설계가 없다') from None
    except ValueError as e:
        raise HTTPException(409, str(e)) from None


@router.get('/{design_id}/builds')
def builds(design_id: str, request: Request):
    """이 설계의 조립 기록(build/1 전부, 최근 것 먼저). 설계가 없으면 404 — 기록이 없으면 빈 목록."""
    store = request.app.state.store
    try:
        store.get_design(design_id)
    except KeyError:
        raise HTTPException(404, f'{design_id} 설계가 없다') from None
    except ValueError as e:
        raise HTTPException(409, str(e)) from None
    return store.builds_for(design_id)
