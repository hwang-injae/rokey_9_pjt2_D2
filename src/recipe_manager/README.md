# recipe_manager — CAD 설계에서 조립 레시피까지

젠가 가구의 CAD 설계를 **조립 레시피**(어떤 블록을, 어떤 순서로, 어떻게 잡아서 놓는가)로 바꾼다. ROS 노드가 아니라 **미리 돌리는 도구**다. 담당: 한세교(CAD).

- 같은 폴더의 `COLCON_IGNORE` 때문에 `colcon build`는 이 폴더를 건너뛴다(ROS 패키지가 아님).
- 레시피를 로봇 목표(TCP)로 바꾸는 일은 집기·놓기 노드(`src/d2_robot/d2_motion`)가 한다. 작업 폴더의 `Recipe_to_Robot`(두산 직접 실행)은 이 저장소에 넣지 않았다 — W104 합의(pose = 블록 중심)로 역할이 집기·놓기에 있고, 로봇을 움직이는 코드는 팀 정지 규칙(서기 궤적·`SignalHandlerOptions.NO`)을 따라야 하기 때문.

```
cads/<id>.dxf ──inspect──▶ cads/<ID>_plan.json ──(사람이 순서·단계·파지 방법 확인)──▶ build
                                                                                    │
           recipes/<ID>_recipe.json + _placements.csv ◀───────────────────────────────┘
                     │
                     └──▶ 작업 판단(d2_task)이 읽음 → PickPlace(pose = 블록 중심) → 집기·놓기(d2_motion)
```

## 폴더

| 폴더·파일 | 내용 |
|---|---|
| `cads/` | **입력.** 설계 원본(DXF = 도구가 읽는 조립 정의, STEP = 검사기용 치수) + `<ID>_plan.json`(**사람이 쓴 순서·단계·파지 방법** — CAD와 함께 레시피의 원본) |
| `recipes/` | **결과.** 모형마다 `<ID>_recipe.json`(로봇·task·비전이 읽음) + `<ID>_placements.csv`(사람이 보는 배치표) 2개만 |
| `recipe_manager/` | 코드(아래 '코드 구조') |
| `requirements.txt` | `ezdxf`, `numpy`(필수), `cadquery`(STEP 읽기·검사기) |

## 쓰는 레시피 — 의자 2 · 책상 2 (10/6 한세교 결정, 004는 10/7 추가)

레시피 형식은 `assembly.recipe/1.0`(이 도구의 출력, `src/d2_robot/d2_motion` `run_recipe`가 읽음)이다 — `assembly`(조립)는 너무 넓은 말이라 `cad_recipe/1.0`으로 바꾼다(10/7 한세교, 변환기 ②와 W121에서 시점을 정해 같이 바꿈). 레시피 안 모델 · 계획 형식은 10/7에 `cad_model/1.0` · `cad_plan/1.0`으로 바꿨다. 10/4 결정 D-16의 옛 형식 `m0609.jenga.cad_recipe/1.0`(`blocks[]` 구조)은 쓰지 않는다.

| 가구 | 모델 ID (파일 이름) | CAD (`design_id`) | 블록 | 층 | 크기 (mm) | 파지 방법 | 상태 |
|---|---|---|---|---|---|---|---|
| 의자 | `001_CHAIR_BENCH` (옛 LV1) | `001_chair_bench` | 11 | 5 | 75 × 75 × 75 | FLAT_SHORT 8, FLAT_LONG 3 | 1차 (10/6 실기 11/11, 시제품 코드) |
| 의자 | `002_CHAIR_BACK` (옛 LV2) | `002_chair_back` | 16 | 10 | 75 × 75 × 150 | FLAT_SHORT 8, FLAT_LONG 8 | 계획은 DXF 힌트 초안 — 순서·잡기 사람 확인 필요 |
| 책상 | `003_DESK_STAND` (옛 LV4) | `003_desk_stand` | 9 | 3 | 75 × 75 × 105 | STAND_SHORT 4, FLAT_LONG 5 | 계획은 DXF 힌트 초안 — **세운 블록 공급 칸 필요** |
| 책상 | `004_DESK_PEDESTAL` (A안 가운데 기둥, 工자) | `004_desk_pedestal` | 11 | 7 | 75 × 75 × 105 | FLAT_LONG 6, FLAT_SHORT 5 | 받은 DXF(`A안_가운데기둥_책상_11블록.dxf`)를 001~003과 같은 꼴로 정리해 `cads/004_desk_pedestal.dxf`(10/7 — 원점 십자선 · 번호 글자 층 뺌, 블록마다 `004_DESK_PEDESTAL_B###` + SEQ · STAGE · GRASP 속성, 핸들 그대로). 순서 · 잡기는 받은 옛 레시피(`desk_a_pedestal.recipe.json`)와 같음. STEP은 이 DXF에서 Open CASCADE 7.9로 변환(10/7 한세교) 뒤 제품 이름을 `004_desk_pedestal` · `004_DESK_PEDESTAL_B###`로 맞춤. 검사 묶음 통과(최소 여유 12.5 mm), 실기 전 |

- 크기는 레시피 CAD 치수(명목 블록 75 × 25 × 15 mm)로 계산한 바깥 크기다. 실측 블록은 74.5 × 24.8 × 14.75 mm다. 조립 작업 영역은 30 × 30 × 30 cm다.
- 세운 블록(STAND_*)은 같은 자세로 놓인 공급 칸에서 집어야 한다(재파지 없음). `003_DESK_STAND`를 쓰기 전에 세운 블록 칸을 교시하고 집기·놓기를 실기 확인한다.
- 이름 = `<번호 3자리>_<가구>_<모양>`(번호 = 의자 먼저, 책상 다음, 새로 만들면 004부터): 모델 ID(대문자)가 레시피 파일 이름(`<모델ID>_recipe.json` — 이름 안 구분은 `_`, 점은 확장자 앞 하나만, 10/7 한세교)과 `block_id` 앞글자(`001_CHAIR_BENCH_B001`)가 되고, CAD 파일 이름(소문자)이 `design_id`다(10/6 한세교, 옛 `LV1`·`lv1_bench`에서 바꿈).
- 책장(LV3)·아치(LV5)·세운 의자(LV6)는 쓰지 않아 뺐다. 원본은 한세교 작업 폴더에 있다.
- 이 폴더의 DXF와 계획으로 `build`를 다시 돌리면 같은 레시피가 나온다(10/7 시험 `tests/test_recipe_manager.py`).

## 명령 (저장소 루트에서)

```bash
pip install -r src/recipe_manager/requirements.txt

# 1) CAD → 블록 표(화면) + 계획 양식 cads/<ID>_plan.json (DXF 속성 SEQ/STAGE/GRASP가 있으면 미리 채움)
python3 src/recipe_manager/recipe_manager/main.py inspect src/recipe_manager/cads/001_chair_bench.dxf --model-id 001_CHAIR_BENCH
# 2) cads/001_CHAIR_BENCH_plan.json의 sequence·stage·grasp 확인
# 3) 검증 → recipes/ 에 레시피 + 배치표
python3 src/recipe_manager/recipe_manager/main.py build src/recipe_manager/cads/001_chair_bench.dxf src/recipe_manager/cads/001_CHAIR_BENCH_plan.json
```

같은 이름의 출력 파일이 있으면 `inspect`·`build`는 덮어쓰기 / 다른 이름으로 저장 / 취소를 묻는다. 입력을 받을 수 없는 환경에서는 기존 파일을 지키려고 취소한다.

| 계획 값 | 입력 내용 |
|---|---|
| `sequence` | 놓는 순서 1..N, 빠짐·중복 없음 |
| `stage` | 1부터 시작하고, 순서를 따라 같거나 1씩 증가 |
| `grasp` | 파지 방법 6가지(아래 약속). 닫힘 축 `grasp_axis`는 `build`가 계산한다 |

`build`는 CAD를 다시 읽는다. 계획의 CAD 해시(`source_cad_sha256`)가 다르면 거부하므로, CAD가 바뀌었으면 `inspect`부터 다시 한다.

### build 검사 항목

- 블록이 축에 평행한 직육면체이고, 서로 겹치지 않고, 책상 아래에 있지 않다.
- 모든 블록을 정확히 한 번 놓고, `sequence`·`stage` 규칙을 지킨다.
- 파지 방법의 상태(눕힘·옆세움·세움)가 CAD 배치와 같다 → 닫힘 축은 늘 수평.
- 책상 위가 아닌 블록은 먼저 놓인 블록 위에 놓인다(받침 = `support_block_ids`).
- 실제 파지·충돌·안정성·로봇 도달은 보지 않는다(안정성은 task의 검사 묶음).

입력 조건: DXF는 `INSUNITS=4`(mm), 블록마다 3DFACE 6개로 된 블록의 INSERT(스케일 1, 배열 아님). STEP은 블록마다 평면 6면 솔리드. `instance_id`는 CAD 원본 ID(`DXF_INSERT_<handle>`, `STEP_SOLID_0001`).

## 코드 구조

모델·계획·레시피는 처음부터 **저장할 JSON 모양의 dict**로 다룬다(객체 ↔ JSON 변환 층 없음, 10/7 간소화).

| 파일 | 클래스 | 하는 일 |
|---|---|---|
| `recipe_manager/cad_reader.py` | `CadReader` | DXF·STEP → 블록 목록 `[{'id', 'vertices', 'hints'}]` |
| `recipe_manager/recipe_builder.py` | `RecipeBuilder` | 블록 → 모델(`make_model`) → 계획 양식(`make_plan_template`) → 검증된 레시피(`build_recipe`) · 배치표 줄. 파지 방법 6가지 · 겹침 · 받침 · `block_id` 계산. ROS·파일 없음 — 다른 코드가 `from recipe_manager.recipe_builder import RecipeBuilder`로 쓴다 |
| `recipe_manager/main.py` | `RecipeManager` | 명령(`inspect`·`build`) · 파일 저장(덮어쓰기 질문) |

시험: 저장소 루트에서 `python3 -m pytest tests/test_recipe_manager.py -q` — 3종을 DXF에서 다시 만들어 저장된 레시피와 같은지 + 겹침·뜬 블록·순서·잡기 상태 오류 거부.

## 기본 설계 블록 JSON (`<모델ID>_blocks.json`, W117)

HMI 가 DB 기본 설계를 등록할 때(W111) 이 폴더 하나만 읽도록, 같은 설계의 `_structure` · `_recipe` · `_placements` 옆에 변환기 ②가 만든 `blocks/1` 파일을 둔다.
**레시피가 바뀌면 다시 만든다**(저장소 맨 위에서 한 줄 — 변환 · `DesignChecker` 검사에 하나라도 실패하면 아무것도 쓰지 않는다):

```bash
PYTHONPATH=src/d2_task python3 -m d2_task.base_blocks
```

레시피를 고치고 이 파일을 안 고치면 `src/d2_task/test/test_base_blocks.py`가 실패한다.

## 지켜야 할 약속

- 단위 mm, 설계 원점 = 바닥 외곽 가운데, X 오른쪽·Y 뒤쪽·Z 위, 책상면 Z = 0.
- **잡기 이름은 파지 방법 6가지** `<FLAT|EDGE|STAND>_<LONG|SHORT>`(IRD 2장, S-25). 다리 벽 `FLAT_SHORT`, 좌판 `FLAT_LONG`. 상태(눕힘·옆세움·세움)는 CAD 배치와 같아야 하고, 공급 블록도 놓을 때와 같은 상태여야 한다(재파지 없음).
- 순서와 잡기는 형상으로 추론하지 않는다. 사람이 계획 파일에 쓰거나 CAD 속성 힌트를 초안으로 가져온다.
- 레시피에는 로봇·현장 값(TCP, 조립 원점, 그리퍼 명령값)을 넣지 않는다. 조립 원점은 `src/d2_robot/d2_bringup/config/robot.yaml`의 `assembly_origin` 하나만 쓴다.

## W104 합의 뒤 남은 일 (W105 · W106)

- 팀 형식으로 내보내기(W105): `block_id` = `<모델ID>_B<sequence 3자리>`(`001_CHAIR_BENCH_B001`…) — 10/7부터 레시피 `steps[]`에 `block_id`·`support_block_ids`로 적는다. `instance_id`(DXF 핸들)는 CAD를 다시 저장하면 바뀌어 CAD 추적용으로만 남긴다. 남은 것: `design_id` = `001_chair_bench`, `grasp`·`supports`, 블록 중심 x·y + `bottom_z_mm` + `size_mm`(CAD 명목값). yaw는 `size_mm`로 계산(긴 변이 CAD x면 0°).
- task 노드가 레시피를 읽는 경로(ROS 패키지 경로로 읽을 곳) — W106에서 정한다.
