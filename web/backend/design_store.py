# -*- coding: utf-8 -*-
"""DesignStore — 설계 · 조립 기록을 저장하는 한 곳 (SDD 6.8, W111 1단계 = JSON 파일 폴더 → W088 PostgreSQL). ROS 없음.

하는 일:
- 기본 설계 등록: 레시피 폴더의 <모델ID>_recipe.json + _placements.csv → 변환기 ②(d2_task, E-59) → design/2.0 → designs/<id>.json
- 읽기: list_designs(트리 · RAG 목록 요약 + 최근 조립 결과, E-72 — Template 으로 거름) · children · get_design(design/2.0) · builds_for ·
        examples_for(RAG 자동 전환)
- 쓰기: new_design_id(검사 전에 ID 만 정함) → save_design(검사 합격한 고른 후보 1개) · save_build(build/1, 같은 run_id 는 한 번만) ·
        save_gen_log(AI 호출 기록 — NFR-17)
설계 이름(10/10 E-84 · IRD 2장): 모두 <Template ID>_V<3자리>. Template = 기본 설계 4개 이름(001_CHAIR_BENCH …), 기본 = _V000,
그 밖(AI · 음성 · 스캔)은 같은 Template 의 다음 번호. 고르기 전 후보 · 저장 전 스캔은 웹 메모리에만(E-84 ⑥ — rejected 기록 없음).
형식 이름 상수는 이 파일 한 곳(web/README 4장). 저장한 설계는 고치지 않는다 — 기본 설계 4개만 레시피 파일에서 같은 ID로 다시 등록(E-55 ①).
"""
import json
import logging
import os
import re
import threading
import time
from pathlib import Path

from d2_task.recipe_document import RecipeDocument
from d2_task.recipe_to_blocks import RecipeToBlocks, ori_extents

SCHEMA_DESIGN = 'design/2.0'
SCHEMA_BLOCKS = 'blocks/2.0'
SCHEMA_BUILD = 'build/1'
RECIPE_SUFFIX = '_recipe.json'
MADE_BY_NEW = ('web', 'voice', 'scan')       # save_design 으로 들어오는 made_by(IRD 2장) — cad 는 register_base 만
SAFE_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.\-]*')   # 파일 이름으로 쓰는 ID — 경로 글자(/ \ ..)를 막는다
DESIGN_ID = re.compile(r'([0-9]{3}(?:_[A-Z]+)+)_V([0-9]{3})')   # <Template ID>_V<3자리>(E-84) — 예 001_CHAIR_BENCH_V001
log = logging.getLogger('web.store')


def family_of(design_id):
    """기본 설계 이름 → family(IRD 2장 chair · desk). 이름에 CHAIR · DESK 가 없으면 ValueError(지어내지 않는다)."""
    name = design_id.upper()
    if 'CHAIR' in name:
        return 'chair'
    if 'DESK' in name:
        return 'desk'
    raise ValueError(f'{design_id}: 이름으로 family 를 알 수 없다(CHAIR · DESK)')


def split_design_id(design_id):
    """설계 ID → (Template ID, 번호 int). 예 '001_CHAIR_BENCH_V003' → ('001_CHAIR_BENCH', 3). E-84 꼴이 아니면 ValueError."""
    m = DESIGN_ID.fullmatch(design_id) if isinstance(design_id, str) else None
    if not m:
        raise ValueError(f'설계 ID {design_id!r} 가 <Template ID>_V<3자리> 꼴이 아니다(E-84)')
    return m.group(1), int(m.group(2))


def same_major(got, expected):
    """형식 버전 규칙(IRD 6장): 이름과 앞자리가 같으면 받는다(뒷자리 · 모르는 칸은 무시). 'state/1' 같은 점 없는 이름도 앞자리로 본다."""
    if not isinstance(got, str) or '/' not in got:
        return False
    name, ver = got.split('/', 1)
    exp_name, exp_ver = expected.split('/', 1)
    return name == exp_name and ver.split('.')[0] == exp_ver.split('.')[0]


def build_summary(build):
    """build/1 → 목록 · 프롬프트용 한 줄 {run_id, result, placed, total, max_err_mm}.

    max_err_mm = 블록마다 잰 dx · dy · dz 중 가장 큰 절댓값(mm, 측정값만 — null 은 뺌). 잰 값이 하나도 없으면 None.
    """
    errs = [abs(b[k]) for b in build.get('blocks') or [] if isinstance(b, dict)
            for k in ('dx_m', 'dy_m', 'dz_m') if isinstance(b.get(k), (int, float)) and not isinstance(b.get(k), bool)]
    return {'run_id': build.get('run_id'), 'result': build.get('result'), 'placed': build.get('placed'),
            'total': build.get('total'), 'max_err_mm': round(max(errs) * 1000, 1) if errs else None}


class DesignStore:
    """설계(designs) · 조립 기록(builds) 저장소. 1단계는 data_dir/designs/<design_id>.json · builds/<run_id>.json.

    입력: data_dir(웹 PC 로컬 폴더 — 규칙 ⑥, gitignore), recipe_dir(기본 설계 레시피 폴더), block_mm(robot.yaml block_size_m × 1000).
    바깥 영향: data_dir 아래 파일 쓰기(임시 파일에 쓴 뒤 이름 바꾸기 — 쓰다 꺼져도 반쪽 파일이 남지 않게).
    실패: 없는 설계는 KeyError, 형식 · 규칙이 틀리면 ValueError — 부르는 쪽(REST · MQTT)이 success false 로 바꾼다.
    """

    def __init__(self, data_dir, recipe_dir, block_mm):
        """폴더를 만들고(있으면 그대로) 방향 코드 표를 준비한다. 기본 설계 등록은 register_bases() 를 따로 부른다."""
        self.designs_dir = Path(data_dir) / 'designs'
        self.builds_dir = Path(data_dir) / 'builds'
        self.gen_logs_dir = Path(data_dir) / 'gen_logs'
        self.recipe_dir = Path(recipe_dir)
        self.block_mm = list(block_mm)
        self.extent = ori_extents(self.block_mm)
        self._lock = threading.Lock()   # 같은 ID 를 두 요청이 동시에 저장하지 않게(REST 는 여러 스레드에서 불린다)
        for d in (self.designs_dir, self.builds_dir, self.gen_logs_dir):
            d.mkdir(parents=True, exist_ok=True)

    # ---------- 기본 설계 ----------
    def register_bases(self):
        """레시피 폴더의 기본 설계를 모두 다시 등록한다(웹 시작 때). 반환: 등록한 design_id 목록.

        하나가 틀려도 나머지는 등록하고 틀린 것은 로그만 남긴다 — 레시피 하나 때문에 화면 전체가 안 뜨지 않게.
        도면에서 온 설계라 등록 때 check_design 은 부르지 않는다(로봇 PC 가 안 붙어 있어도 목록이 떠야 함 — 검사는 레시피 PR 때 끝남).
        """
        done = []
        for path in sorted(self.recipe_dir.glob('*' + RECIPE_SUFFIX)):
            design_id = path.name[:-len(RECIPE_SUFFIX)]
            try:
                self.register_base(design_id)
                done.append(design_id)
            except (OSError, ValueError, KeyError) as e:
                log.warning('[저장소] 기본 설계 %s 등록 실패: %s', design_id, e)
        return done

    def register_base(self, design_id):
        """기본 설계 하나: 레시피 두 파일 → 변환기 ② → design/2.0(V000 · made_by cad · 부모 없음) → 저장. 실패는 OSError · ValueError."""
        family = family_of(design_id)
        doc = RecipeDocument.load(self.recipe_dir, design_id)
        design = doc.design(design_id)
        design.update(schema=SCHEMA_DESIGN, family=family, version='V000', parent_id=None, made_by='cad',
                      blocks=RecipeToBlocks(design_id, family, self.block_mm).convert(doc.recipe, doc.placements),
                      created=round(time.time(), 3))
        self._write(self._design_path(design_id), design)
        return design

    # ---------- 읽기 ----------
    def get_design(self, design_id):
        """design/2.0 하나. 없으면 KeyError, 저장된 형식이 design/2 가 아니면 ValueError(옛 형식은 변환하지 않고 거절, E-69)."""
        path = self._design_path(design_id)
        if not path.is_file():
            raise KeyError(design_id)
        design = json.loads(path.read_text(encoding='utf-8'))
        if not same_major(design.get('schema'), SCHEMA_DESIGN):
            raise ValueError(f'{design_id}: 저장된 형식 {design.get("schema")!r} 은 {SCHEMA_DESIGN} 이 아니다')
        return design

    def list_designs(self, family=None, template=None):
        """설계 요약 목록(화면 트리 · RAG 목록 요약, E-72). 읽을 수 없는 파일은 건너뛴다.

        template 을 주면 그 Template 의 설계만 — AI 생성은 사용자가 확인한 Template 안에서만 참고 설계를 고르게(E-84 ③,
        GPT 목록 요약 · 도구 get_design 허용 목록). 다른 Template 설계를 읽고 만든 후보는 save_design 에서 거절되기 때문.
        출력: [{design_id, family, version, parent_id, made_by, block_count, size_mm[x, y, z], last_build}] — family · design_id 순.
        size_mm = 바깥 크기(블록이 차지하는 x · y · z 범위, mm). last_build = 가장 최근 조립 한 줄(build_summary) 또는 None.
        """
        return [row for row, _ in self._rows(family, template)]

    def children(self, design_id):
        """design_id 를 부모로 둔 설계 요약 목록(버전 트리 — 바로 아래 한 층만). 없으면 빈 목록."""
        return [row for row in self.list_designs() if row['parent_id'] == design_id]

    def builds_for(self, design_id):
        """이 설계의 조립 기록 build/1 전부, 최근 것 먼저(run_id 가 시각으로 시작해 글자 순 = 시각 순, IRD 2장)."""
        return sorted((b for b in self._builds() if b.get('design_id') == design_id),
                      key=lambda b: b['run_id'], reverse=True)

    def examples_for(self, template, n=3):
        """RAG 자동 전환용 예시(E-72 — GPT 가 도구 get_design 으로 못 골랐을 때): 이 Template 의 기본 설계(V000) + 최근 파생 n 개.

        출력: [{요약 칸(list_designs 와 같음), 'blocks': blocks/2.0}] — 기본 설계 먼저, 파생은 저장 시각 최신순.
        프롬프트에는 blocks 만 넣는다(recipe · placements 는 GPT 가 쓰지 않음). 모르는 Template 이면 ValueError.
        """
        self._check_template(template)
        rows = self._rows(template=template)
        bases = [{**row, 'blocks': d['blocks']} for row, d in rows if row['made_by'] == 'cad']
        derived = sorted(((row, d) for row, d in rows if row['made_by'] != 'cad'),
                         key=lambda rd: rd[1].get('created') or 0, reverse=True)[:max(0, n)]
        return bases + [{**row, 'blocks': d['blocks']} for row, d in derived]

    def _rows(self, family=None, template=None):
        """(요약, 설계 전체) 짝 목록 — family · design_id 순. 읽거나 요약할 수 없는 설계는 로그만 남기고 건너뛴다.
        template 을 주면 이름이 <template>_V### 인 설계만(E-84 꼴이 아닌 옛 이름 파일은 빠진다)."""
        last = {}
        for b in self._builds():                     # 설계마다 가장 최근 조립(run_id 가 가장 큰 것)
            if b.get('design_id') and b['run_id'] > last.get(b['design_id'], {}).get('run_id', ''):
                last[b['design_id']] = b
        out = []
        for path in sorted(self.designs_dir.glob('*.json')):
            try:
                d = self.get_design(path.stem)
                blocks = d['blocks']['blocks']
                row = {k: d.get(k) for k in ('design_id', 'family', 'version', 'parent_id', 'made_by')}
                row.update(block_count=len(blocks), size_mm=self._size_mm(blocks),
                           last_build=build_summary(last[d['design_id']]) if d.get('design_id') in last else None)
            except (OSError, ValueError, KeyError, TypeError) as e:
                log.warning('[저장소] %s 건너뜀: %s', path.name, e)
                continue
            if (family is None or row['family'] == family) and \
                    (template is None or self._template_or_none(row['design_id']) == template):
                out.append((row, d))
        return sorted(out, key=lambda rd: (rd[0]['family'] or '', rd[0]['design_id']))

    def _builds(self):
        """저장된 build/1 전부(run_id 가 글자인 것만). 읽을 수 없는 파일은 건너뛴다."""
        out = []
        for path in self.builds_dir.glob('*.json'):
            try:
                b = json.loads(path.read_text(encoding='utf-8'))
            except (OSError, ValueError) as e:
                log.warning('[저장소] 조립 기록 %s 건너뜀: %s', path.name, e)
                continue
            if isinstance(b, dict) and isinstance(b.get('run_id'), str):
                out.append(b)
        return out

    def _size_mm(self, blocks):
        """blocks/2.0 블록들의 바깥 크기 [x, y, z] mm. 방향 코드를 모르면 ValueError."""
        lo, hi = [float('inf')] * 3, [float('-inf')] * 3
        for b in blocks:
            ex, ey, ez = self.extent[b['ori']]
            box = ((b['x'] - ex / 2, b['x'] + ex / 2), (b['y'] - ey / 2, b['y'] + ey / 2), (b['z'], b['z'] + ez))
            for i, (a, c) in enumerate(box):
                lo[i], hi[i] = min(lo[i], a), max(hi[i], c)
        return [round(hi[i] - lo[i], 1) for i in range(3)] if blocks else [0, 0, 0]

    # ---------- 새 설계 ----------
    def new_design_id(self, template, parent_id=None):
        """새 설계의 (design_id, version) 을 정한다 — 저장하지 않는다. **검사 전에** 부른다: 변환기 ①이 이 ID 로 레시피 model_id ·
        block_id 를 만들기 때문(IRD 2장 — 블록 이름 앞에 설계 ID).

        규칙(10/10 E-84 · IRD 2장 · SDD 6.8): <Template ID>_V<3자리>, 번호 = 그 Template 에 저장된 가장 큰 V + 1(만든 차례 — 부모와 상관없음).
        Template = 기본 설계 4개 이름(그 _V000 이 등록돼 있어야 함) — AI 생성은 사용자가 확인한 Template, 스캔은 부모의 Template.
        후보 3개는 이 ID 하나로 검사하고 고른 1개만 save_design(그 사이 다른 요청이 같은 ID 를 먼저 쓰면 저장에서 거절 → 여기부터 다시).
        실패: 모르는 Template · 부모가 다른 Template 이거나 999 를 넘으면 ValueError, 부모가 없으면 KeyError.
        """
        self._check_template(template)
        if parent_id is not None:
            self.get_design(parent_id)                 # 없으면 KeyError
            if self._template_or_none(parent_id) != template:
                raise ValueError(f'부모 {parent_id} 는 Template {template} 이 아니다 — 같은 Template 만 부모로 둔다(E-84)')
        used = [self._template_number(path.stem, template) for path in self.designs_dir.glob(f'{template}_V*.json')]
        n = max((u for u in used if u is not None), default=0) + 1
        if n > 999:
            raise ValueError(f'{template} 의 번호가 V999 를 넘는다')
        return f'{template}_V{n:03d}', f'V{n:03d}'

    def save_design(self, blocks, recipe, placements, parent_id, made_by, prompt=None, check=None):
        """검사를 통과해 사람이 고른 설계 하나를 저장한다(SDD 6.8 · IRD 8.3 ⑥ · 8.4 ⑦). 저장한 설계는 고치지 않는다(E-55 ①).

        입력: blocks(blocks/2.0 — design_id 는 new_design_id 로 정한 것) · recipe · placements(그 blocks 로 check_design 이 돌려준 것) ·
              parent_id · made_by(web · voice · scan) · prompt(요청 문장) · check(check_result/2.0 — ok · min_margin_mm · errors).
        출력: 저장한 기록 = design/2.0(version = ID 의 V 번호, family = Template 에서) + prompt · check · created(DB designs 칸, FR-D02).
        실패(ValueError — 저장하지 않음): 검사 불합격(SR-09 — AI 설계는 검사 통과 없이 로봇에 가지 않는다) · made_by · 형식 ·
              ID 꼴(E-84 — V000 은 기본 설계 몫) · 모르는 Template · family 다름 · 레시피 짝(model_id · recipe_sha256 — 로봇이 못 읽는 레시피) ·
              부모 없음 · 부모가 다른 Template(마지막 방어선 — 앞에서 list_designs(template=…)로 거름) · 같은 ID 가 이미 있음.
        """
        if made_by not in MADE_BY_NEW:
            raise ValueError(f'made_by 는 {MADE_BY_NEW} 중 하나다: {made_by!r}')
        if not isinstance(check, dict) or check.get('ok') is not True:
            raise ValueError('검사에 합격한 설계만 저장한다(check.ok 가 true 가 아니다)')
        if not isinstance(blocks, dict) or not same_major(blocks.get('schema'), SCHEMA_BLOCKS):
            raise ValueError(f'blocks 형식이 {SCHEMA_BLOCKS} 가 아니다')
        design_id = blocks.get('design_id')
        template, number = split_design_id(design_id)
        if number == 0:
            raise ValueError(f'{design_id}: V000 은 기본 설계(도면) 몫이다 — new_design_id 로 다음 번호를')
        self._check_template(template)
        family = family_of(template)
        if blocks.get('family') != family:
            raise ValueError(f'blocks family {blocks.get("family")!r} 가 Template {template} 의 {family} 가 아니다')
        RecipeDocument(recipe, placements)          # 형식 · model_id 짝 · recipe_sha256 — 로봇(작업 판단)과 같은 확인. 틀리면 ValueError
        if recipe.get('model_id') != design_id:
            raise ValueError(f'레시피 model_id {recipe.get("model_id")!r} 가 {design_id} 가 아니다 — 이 ID 로 다시 검사')
        if parent_id is not None:
            if not self._design_path(parent_id).is_file():
                raise ValueError(f'부모 {parent_id} 가 없다')
            if self._template_or_none(parent_id) != template:
                raise ValueError(f'부모 {parent_id} 는 Template {template} 이 아니다 — 같은 Template 만 부모로 둔다(E-84)')
        record = {'schema': SCHEMA_DESIGN, 'design_id': design_id, 'family': family, 'version': f'V{number:03d}',
                  'parent_id': parent_id, 'made_by': made_by, 'recipe': recipe, 'placements': placements, 'blocks': blocks,
                  'prompt': prompt, 'check': {k: check.get(k) for k in ('ok', 'min_margin_mm', 'errors')},
                  'created': round(time.time(), 3)}
        path = self._design_path(design_id)
        with self._lock:
            if path.exists():
                raise ValueError(f'{design_id} 가 이미 있다 — 저장한 설계는 고치지 않는다(new_design_id 부터 다시)')
            self._write(path, record)
        log.info('[저장소] 새 설계 %s (부모 %s, %s)', design_id, parent_id, made_by)
        return record

    def _check_template(self, template):
        """Template 확인 — 기본 설계 4개 이름 꼴이고 그 _V000 이 등록돼 있어야 한다(E-84 ② Template 은 기본 4개로 고정). 아니면 ValueError."""
        if not isinstance(template, str) or not DESIGN_ID.fullmatch(f'{template}_V000') \
                or not self._design_path(f'{template}_V000').is_file():
            raise ValueError(f'모르는 Template: {template!r}(기본 설계 _V000 이 등록된 4개만)')

    @staticmethod
    def _template_or_none(design_id):
        """설계 ID 의 Template ID. E-84 꼴이 아니면(옛 이름 등) None."""
        try:
            return split_design_id(design_id)[0]
        except ValueError:
            return None

    @staticmethod
    def _template_number(name, template):
        """파일 이름(확장자 뺀 것)이 <template>_V### 이면 번호, 아니면 None."""
        try:
            t, n = split_design_id(name)
        except ValueError:
            return None
        return n if t == template else None

    # ---------- 조립 기록 ----------
    def save_build(self, build):
        """build/1 하나를 builds/<run_id>.json 으로 저장한다. 반환: 새로 저장했으면 True, 같은 run_id 가 이미 있으면 False.

        작업 관리자는 저장이 확인될 때까지 같은 요약을 다시 보낸다(timeout.service_s 간격) — 그래서 두 번째부터는 쓰지 않고 성공으로 본다.
        실패: 형식이 build/1 이 아니거나 run_id 가 파일 이름으로 못 쓰는 글자면 ValueError.
        """
        if not isinstance(build, dict) or not same_major(build.get('schema'), SCHEMA_BUILD):
            raise ValueError(f'형식이 {SCHEMA_BUILD} 가 아니다(받은 schema: {build.get("schema") if isinstance(build, dict) else None!r})')
        path = self.builds_dir / f'{self._safe(build.get("run_id"))}.json'
        if path.exists():
            return False
        self._write(path, build)
        return True

    # ---------- AI 호출 기록(NFR-17) ----------
    def save_gen_log(self, log):
        """AI 호출 기록 하나(프롬프트 · 응답 원문 · 시간 · 검사 결과 — NFR-17 같은 요청 반복 때 차이를 설명하려고)를
        gen_logs/<시각>_<종류>.json 으로 남긴다. 반환: 파일 이름. 키 값은 들어오지 않는다(design_gen 은 키를 기록에 넣지 않음)."""
        now = time.time()
        name = f"{time.strftime('%Y%m%d_%H%M%S', time.localtime(now))}_{int(now * 1000) % 1000:03d}_{self._safe(log.get('kind') or 'gen')}.json"
        self._write(self.gen_logs_dir / name, log)
        return name

    # ---------- 로봇이 부름(MQTT — MqttClient.serve) ----------
    def answer_get_design(self, body):
        """/d2/hmi/get_design 요청 {"design_id"} → 응답 dict(success · reason + design/2.0 칸). 없거나 틀리면 success false · ERROR."""
        design_id = body.get('design_id')
        try:
            return {'success': True, 'reason': '', **self.get_design(design_id)}
        except (KeyError, ValueError, OSError) as e:
            log.warning('[저장소] get_design %s 실패: %s', design_id, e)
            return {'success': False, 'reason': 'ERROR', 'message': f'{design_id} 설계를 못 꺼냄'}

    def answer_save_build(self, body):
        """/d2/hmi/save_build 요청(build/1 + req_id) → {success, reason, ok}. ok = 저장됨(같은 run_id 두 번째도 true, IRD 4.2)."""
        build = {k: v for k, v in body.items() if k != 'req_id'}
        try:
            fresh = self.save_build(build)
        except (ValueError, OSError) as e:
            log.warning('[저장소] save_build 실패: %s', e)
            return {'success': True, 'reason': '', 'ok': False, 'message': str(e)}
        log.info('[저장소] save_build %s %s', build.get('run_id'), '저장' if fresh else '이미 있음')
        return {'success': True, 'reason': '', 'ok': True}

    # ---------- 파일 ----------
    def _design_path(self, design_id):
        """design_id → 파일 경로. 파일 이름으로 못 쓰는 ID 는 KeyError(없는 설계와 같게 — 경로 밖으로 못 나가게)."""
        try:
            return self.designs_dir / f'{self._safe(design_id)}.json'
        except ValueError as e:
            raise KeyError(design_id) from e

    @staticmethod
    def _safe(name):
        """파일 이름으로 쓸 ID 확인(글자 · 숫자 · _ . - 만, '..' 없음). 아니면 ValueError."""
        if not isinstance(name, str) or not SAFE_ID.fullmatch(name) or '..' in name:
            raise ValueError(f'파일 이름으로 쓸 수 없는 ID: {name!r}')
        return name

    @staticmethod
    def _write(path, obj):
        """임시 파일에 쓰고 이름을 바꾼다(같은 폴더 안 os.replace 는 한 번에 바뀐다)."""
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(obj, ensure_ascii=False, indent=1), encoding='utf-8')
        os.replace(tmp, path)
