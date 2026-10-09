# -*- coding: utf-8 -*-
"""DesignStore — 설계 · 조립 기록을 저장하는 한 곳 (SDD 6.8, W111 1단계 = JSON 파일 폴더 → W088 PostgreSQL). ROS 없음.

지금 하는 일(W111 최소형 — 화면 설계 트리 · 3D 보기와 로봇의 get_design · save_build 에 필요한 만큼):
- 기본 설계 등록: 레시피 폴더의 <모델ID>_recipe.json + _placements.csv → 변환기 ②(d2_task, E-59) → design/2.0 → designs/<id>.json
- list_designs(트리용 요약) · get_design(design/2.0) · save_build(build/1, 같은 run_id 는 한 번만)
뒤에 붙는 것: save_design(W108 · W112 — 고른 후보 1개 저장) · children · examples_for(RAG 자동 전환, E-72).
형식 이름 상수는 이 파일 한 곳(web/README 4장). 저장한 설계는 고치지 않는다 — 기본 설계 4개만 레시피 파일에서 같은 ID로 다시 등록(E-55 ①).
"""
import json
import logging
import os
import re
from pathlib import Path

from d2_task.recipe_document import RecipeDocument
from d2_task.recipe_to_blocks import RecipeToBlocks, ori_extents

SCHEMA_DESIGN = 'design/2.0'
SCHEMA_BLOCKS = 'blocks/2.0'
SCHEMA_BUILD = 'build/1'
RECIPE_SUFFIX = '_recipe.json'
SAFE_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.\-]*')   # 파일 이름으로 쓰는 ID — 경로 글자(/ \ ..)를 막는다
log = logging.getLogger('web.store')


def family_of(design_id):
    """기본 설계 이름 → family(IRD 2장 chair · desk). 이름에 CHAIR · DESK 가 없으면 ValueError(지어내지 않는다)."""
    name = design_id.upper()
    if 'CHAIR' in name:
        return 'chair'
    if 'DESK' in name:
        return 'desk'
    raise ValueError(f'{design_id}: 이름으로 family 를 알 수 없다(CHAIR · DESK)')


def same_major(got, expected):
    """형식 버전 규칙(IRD 6장): 이름과 앞자리가 같으면 받는다(뒷자리 · 모르는 칸은 무시). 'state/1' 같은 점 없는 이름도 앞자리로 본다."""
    if not isinstance(got, str) or '/' not in got:
        return False
    name, ver = got.split('/', 1)
    exp_name, exp_ver = expected.split('/', 1)
    return name == exp_name and ver.split('.')[0] == exp_ver.split('.')[0]


class DesignStore:
    """설계(designs) · 조립 기록(builds) 저장소. 1단계는 data_dir/designs/<design_id>.json · data_dir/builds/<run_id>.json.

    입력: data_dir(웹 PC 로컬 폴더 — 규칙 ⑥, gitignore), recipe_dir(기본 설계 레시피 폴더), block_mm(robot.yaml block_size_m × 1000).
    바깥 영향: data_dir 아래 파일 쓰기(임시 파일에 쓴 뒤 이름 바꾸기 — 쓰다 꺼져도 반쪽 파일이 남지 않게).
    실패: 없는 설계는 KeyError, 형식이 다르면 ValueError — 부르는 쪽(REST · MQTT)이 success false 로 바꾼다.
    """

    def __init__(self, data_dir, recipe_dir, block_mm):
        """폴더를 만들고(있으면 그대로) 방향 코드 표를 준비한다. 기본 설계 등록은 register_bases() 를 따로 부른다."""
        self.designs_dir = Path(data_dir) / 'designs'
        self.builds_dir = Path(data_dir) / 'builds'
        self.recipe_dir = Path(recipe_dir)
        self.block_mm = list(block_mm)
        self.extent = ori_extents(self.block_mm)
        self.designs_dir.mkdir(parents=True, exist_ok=True)
        self.builds_dir.mkdir(parents=True, exist_ok=True)

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
        """기본 설계 하나: 레시피 두 파일 → 변환기 ② → design/2.0(v1.0 · made_by cad · 부모 없음) → 저장. 실패는 OSError · ValueError."""
        family = family_of(design_id)
        doc = RecipeDocument.load(self.recipe_dir, design_id)
        design = doc.design(design_id)
        design.update(schema=SCHEMA_DESIGN, family=family, version='1.0', parent_id=None, made_by='cad',
                      blocks=RecipeToBlocks(design_id, family, self.block_mm).convert(doc.recipe, doc.placements))
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

    def list_designs(self, family=None):
        """설계 요약 목록(화면 트리 · RAG 목록 요약, E-72). 읽을 수 없는 파일은 건너뛴다.

        출력: [{design_id, family, version, parent_id, made_by, block_count, size_mm[x, y, z]}] — family · design_id 순.
        size_mm = 바깥 크기(블록이 차지하는 x · y · z 범위, mm).
        """
        rows = []
        for path in sorted(self.designs_dir.glob('*.json')):
            try:
                d = self.get_design(path.stem)
                blocks = d['blocks']['blocks']
                row = {k: d.get(k) for k in ('design_id', 'family', 'version', 'parent_id', 'made_by')}
                row.update(block_count=len(blocks), size_mm=self._size_mm(blocks))
            except (OSError, ValueError, KeyError, TypeError) as e:
                log.warning('[저장소] %s 건너뜀: %s', path.name, e)
                continue
            if family is None or row['family'] == family:
                rows.append(row)
        return sorted(rows, key=lambda r: (r['family'] or '', r['design_id']))

    def _size_mm(self, blocks):
        """blocks/2.0 블록들의 바깥 크기 [x, y, z] mm. 방향 코드를 모르면 ValueError."""
        lo, hi = [float('inf')] * 3, [float('-inf')] * 3
        for b in blocks:
            ex, ey, ez = self.extent[b['ori']]
            box = ((b['x'] - ex / 2, b['x'] + ex / 2), (b['y'] - ey / 2, b['y'] + ey / 2), (b['z'], b['z'] + ez))
            for i, (a, c) in enumerate(box):
                lo[i], hi[i] = min(lo[i], a), max(hi[i], c)
        return [round(hi[i] - lo[i], 1) for i in range(3)] if blocks else [0, 0, 0]

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
