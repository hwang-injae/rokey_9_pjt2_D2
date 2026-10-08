# recipe_manager — CAD 설계에서 조립 레시피까지

젠가 가구의 CAD 설계를 **레시피**(구조 — 무엇을 어디에)와 **조립 방법**(어떤 순서로 어떻게 잡아 놓는가)으로 바꾼다(10/8 E-69 용어). ROS 노드가 아니라 **미리 돌리는 도구**다. 담당: 한세교(CAD).

- 같은 폴더의 `COLCON_IGNORE` 때문에 `colcon build`는 이 폴더를 건너뛴다(ROS 패키지가 아님).
- 레시피를 로봇 목표(TCP)로 바꾸는 일은 집기·놓기 노드(`src/d2_robot/d2_motion`)가 한다. 작업 폴더의 `Recipe_to_Robot`(두산 직접 실행)은 이 저장소에 넣지 않았다 — W104 합의(pose = 블록 중심)로 역할이 집기·놓기에 있고, 로봇을 움직이는 코드는 팀 정지 규칙(서기 궤적·`SignalHandlerOptions.NO`)을 따라야 하기 때문.
- 형식은 **E-69**(작명 규칙 v3, `docs/작명규칙_설계_레시피_블록_v3_*.md`) · IRD 6장 `recipe/2.0` · `placements/2.0`을 따른다(10/8 — 옛 E-52 `cad_structure/1.0` · `cad_recipe/1.0` · `_structure.json`은 없앰). 용어: **레시피 = 구조**(`_recipe.json`), **조립 방법 = placements**(`_placements.csv`).

```
cads/<id>.dxf (블록 이름 · SEQ · STAGE · GRASP 속성 = 원본)
      └── build(검증) ──▶ recipes/<ID>_recipe.json      레시피 = 무엇을 어디에 (recipe/2.0 — 블록 이름 · 부품 규격 · 중심 · 방향 · CAD 핸들)
                          recipes/<ID>_placements.csv  조립 방법 = 어떻게 (순서 · 단계 · 잡기 · 닫힘 축 · 받침 + 표시 칸, recipe_sha256 · schema)
                     │
                     └──▶ 작업 판단(d2_task) · 손목 블록 인식(d2_vision) · 집기·놓기 · 장면(d2_motion)이 읽음
                          (웹 backend는 기본 설계 등록 때 변환기 ②로 blocks/2.0 으로 바꿔 DB에 — E-59 · E-69)
```

## 폴더

| 폴더·파일 | 내용 |
|---|---|
| `cads/` | **입력 = 원본.** DXF(도구가 읽는 조립 정의 — INSERT 블록 이름 = 블록 이름, INSERT 속성 SEQ · STAGE · GRASP = 순서 · 단계 · 잡기) + STEP(치수 참고용 — 도구는 읽지 않음, 제품 이름 = 블록 이름). 계획 파일은 없다(E-52) |
| `recipes/` | **결과.** 모형마다 `<ID>_recipe.json` · `<ID>_placements.csv` 2개(E-69). 손으로 고치지 않는다 — CAD를 고치고 `build`를 다시. 블록 JSON(`blocks/2.0`) 파일은 두지 않는다 — 웹이 등록 때 변환기 ②(`d2_task.recipe_to_blocks`)로 바꾼다(10/8 E-59 · E-69) |
| `recipe_manager/` | 코드(아래 '코드 구조') |
| `requirements.txt` | `ezdxf`, `numpy` |

## 쓰는 레시피 — 의자 2 · 책상 2 (10/6 한세교 결정, 004는 10/7 추가)

| 가구 | 모델 ID (= `design_id`, 파일 이름) | CAD 파일 이름 | 블록 | 층 | 크기 (mm) | 파지 방법 | 블록 이름 | 상태 |
|---|---|---|---|---|---|---|---|---|
| 의자 | `001_CHAIR_BENCH` (옛 LV1) | `001_chair_bench` | 11 | 5 | 75 × 75 × 75 | FLAT_SHORT 9, FLAT_LONG 2 | `LEG_001_01~04` · `LEG_002_01~04` · `SEAT_001_01~03` | 1차 (10/6 실기 11/11) |
| 의자 | `002_CHAIR_BACK` (옛 LV2) | `002_chair_back` | 16 | 10 | 75 × 75 × 150 | FLAT_SHORT 14, FLAT_LONG 2 | 벤치 + `BACK_001_01~05` | 10/6 실기 16/16 |
| 책상 | `003_DESK_STAND` (옛 LV4) | `003_desk_stand` | 9 | 3 | 75 × 75 × 105 | STAND_SHORT 4, FLAT_SHORT 3, FLAT_LONG 2 | `LEG_001~004_01` · `BEAM_001~002_01` · `TOP_001_01~03` | 10/6 실기 9/9 — **세운 블록 공급 칸 필요** |
| 책상 | `004_DESK_PEDESTAL` (가운데 기둥, 工자) | `004_desk_pedestal` | 11 | 7 | 75 × 75 × 105 | FLAT_SHORT 7, FLAT_LONG 4 | `BASE_001_01~03` · `COLUMN_001_01~05` · `TOP_001_01~03` | 받은 DXF(`A안_가운데기둥_책상_11블록.dxf`)를 001~003과 같은 꼴로 정리(10/7 — 원점 십자선 · 번호 글자 층 뺌, SEQ · STAGE · GRASP 속성, 핸들 그대로). STEP은 이 DXF에서 Open CASCADE 7.9로 변환. 검사 묶음 통과(최소 여유 12.5 mm), 실기 전 |

- 크기는 레시피 CAD 치수(명목 블록 75 × 25 × 15 mm)로 계산한 바깥 크기다. 실측 블록은 74.5 × 24.8 × 14.75 mm다. 조립 작업 영역은 30 × 30 × 30 cm다.
- 세운 블록(STAND_*)은 같은 자세로 놓인 공급 칸에서 집어야 한다(재파지 없음). `003_DESK_STAND`를 쓰기 전에 세운 블록 칸을 교시하고 집기·놓기를 실기 확인한다.
- 모델 ID = `<번호 3자리>_<가구>_<모양>`(의자 먼저, 책상 다음, 새로 만들면 다음 번호). CAD 파일 이름 = 모델 ID 소문자, 기본 설계 `design_id` = 모델 ID 그대로(대문자 — W121 · E-60), 레시피 파일 = `<모델ID>_recipe.json` · `_placements.csv`(이름 안 구분은 `_`, 점은 확장자 앞 하나만).
- **블록 이름**(E-52 · E-69) = `<역할>[_<옵션>]_<부품 3자리>_<블록 2자리>`(역할 · 옵션은 영문 대문자 한 단어씩, 옵션 0~1개 — `build`가 검사) — 역할 `LEG` · `SEAT` · `BACK` · `BEAM` · `TOP` · `BASE` · `COLUMN`. 부품 번호는 같은 역할 부품을 앞(−y)→뒤, 왼(−x)→오른 순, 블록 번호는 부품 안에서 아래층부터 · 앞→뒤 · 왼→오른(`build`가 위치와 대조해 틀리면 거부). 노드 사이 `block_id` = `<모델ID>_<블록 이름>`(예 `001_CHAIR_BENCH_LEG_001_01`) — 레시피 파일 안에는 블록 이름만 쓴다. 놓는 순서는 이름이 아니라 `sequence`.
- **잡기는 짧은 쪽 우선**(10/7 한세교 — 긴 쪽 잡기에서 놓기 오차가 더 컸다): 놓는 순간 두 손가락이 들어가면 `*_SHORT`, 막히면 `*_LONG`(손가락 판단은 task 검사 묶음 `grasp_options`). CAD 4종의 GRASP 속성도 이 규칙으로 맞춰 12개 블록이 긴 쪽 → 짧은 쪽으로 바뀌었다(001 `SEAT_001_01`, 002 `SEAT_001_01` · `BACK_001_01~05`, 003 `BEAM_001_01` · `BEAM_002_01` · `TOP_001_01`, 004 `BASE_001_01` · `TOP_001_01`) — 10/6 실기는 긴 쪽이었으므로 **W118에서 실기 재확인**.
- 10/7 W139: CAD 4종 안 블록 이름을 `<모델ID>_B<순서>` → 역할 이름으로 바꿈(DXF INSERT · 블록 정의 · XDATA, STEP 제품 이름 — 핸들 · 좌표 · 속성 그대로). 블록 중심 · 회전 · 순서 · 받침 높이는 4종 모두 바꾸기 전과 같다(잡기만 위 12개가 바뀜).
- 책장(LV3)·아치(LV5)·세운 의자(LV6)는 쓰지 않아 뺐다. 원본은 한세교 작업 폴더에 있다.

## 명령 (저장소 루트에서)

```bash
pip install -r src/recipe_manager/requirements.txt
python3 src/recipe_manager/recipe_manager/main.py build src/recipe_manager/cads/001_chair_bench.dxf
```

모델 ID는 CAD 파일 이름을 대문자로 바꿔 정한다. 순서 · 단계 · 잡기를 바꾸려면 **CAD 속성(SEQ · STAGE · GRASP)을 고치고** 다시 `build` 한다. 같은 이름의 출력 파일이 있으면 덮어쓰기 / 다른 이름으로 저장 / 취소를 묻는다(입력을 받을 수 없는 환경에서는 기존 파일을 지키려고 취소).

| CAD 속성 | 입력 내용 |
|---|---|
| `SEQ` | 놓는 순서 1..N, 빠짐·중복 없음 |
| `STAGE` | 1부터 시작하고, 순서를 따라 같거나 1씩 증가 |
| `GRASP` | 파지 방법 6가지(아래 약속). 옛 이름 `SIDE_25` · `END_75` · `THICKNESS_15` · 축 이름(`WIDTH` 등)도 받아 CAD 배치로 바꾼다. 닫힘 축 `grasp_axis`는 `build`가 계산한다 |

### build 검사 항목

- 블록 이름이 규칙에 맞고 겹치지 않으며, 번호가 위치 순서와 맞다.
- 블록이 축에 평행한 직육면체이고, 서로 겹치지 않고, 책상 아래에 있지 않다.
- 모든 블록에 SEQ · STAGE · GRASP가 있고(빠진 블록은 한 번에 알린다) 순서 · 단계 규칙을 지킨다.
- 파지 방법의 상태(눕힘·옆세움·세움)가 CAD 배치와 같다 → 닫힘 축은 늘 수평.
- 책상 위가 아닌 블록은 먼저 놓인 블록 위에 놓인다(받침 = `supports`).
- 실제 파지·충돌·안정성·로봇 도달은 보지 않는다(안정성 · 손가락 틈은 task의 검사 묶음).

입력 조건: DXF는 `INSUNITS=4`(mm), 블록마다 3DFACE 6개로 된 블록의 INSERT(스케일 1, 배열 아님). STEP은 이름 · 속성이 없어 레시피 원본으로 쓰지 않는다.

## 코드 구조

구조 · 조립 방법은 처음부터 **저장할 JSON 모양의 dict**로 다룬다(객체 ↔ JSON 변환 층 없음).

| 파일 | 클래스 | 하는 일 |
|---|---|---|
| `recipe_manager/cad_reader.py` | `CadReader` | DXF → 블록 목록 `[{'block', 'handle', 'vertices', 'hints'}]` |
| `src/d2_task/d2_task/recipe_builder.py` | `RecipeBuilder` | 블록 → 레시피(`make_recipe` — 이름 · 번호 · 겹침 검사) → 조립 방법(`make_placements` — 순서 · 잡기 · 받침 · `recipe_sha256`) · CSV 줄(`make_placement_rows`). ROS·파일 없음. 변환기 ①(W110, 10/10)과 한 벌로 쓰려고 d2_task 에 있다(E-58) — `main.py`가 소스 폴더를 import 경로에 넣어 쓴다 |
| `recipe_manager/main.py` | `RecipeManager` | 명령(`build`) · 파일 저장(덮어쓰기 질문) |

시험: 저장소 루트에서 `python3 -m pytest tests/test_recipe_manager.py -q` — 4종을 DXF에서 다시 만들어 저장 파일(`_recipe.json` · `_placements.csv`)과 같은지 + 폴더엔 모형마다 2개 · 짝 해시 `recipe_sha256`(구조 객체의 키 정렬 · 공백 없는 JSON, 값 고정) · 이름 · 옵션 하나 · 번호 · 겹침 · 뜬 블록 · 순서 · 단계 · 잡기 상태 · 빠진 속성 거부 · 옛 잡기 이름 변환. 변환기 ① 시험은 W110 PR(10/10)과 함께 온다(지금 없음).

## 지켜야 할 약속

- 단위 mm, 설계 원점 = 바닥 외곽 가운데, X 오른쪽·Y 뒤쪽·Z 위, 책상면 Z = 0.
- **잡기 이름은 파지 방법 6가지** `<FLAT|EDGE|STAND>_<LONG|SHORT>`(IRD 2장, S-25). 다리 벽 `FLAT_SHORT`, 좌판 `FLAT_LONG`. 상태(눕힘·옆세움·세움)는 CAD 배치와 같아야 하고, 공급 블록도 놓을 때와 같은 상태여야 한다(재파지 없음).
- CAD 레시피의 순서와 잡기는 사람이 CAD 속성에 적는다(잡기는 짧은 쪽 우선 규칙대로). AI 생성 · 스캔 설계는 **AI가** 역할 · 부품 · 순서 · 단계 · 잡기를 쓰고, 변환기 ①은 번호 · 중심 · R · 받침 · 닫힘 축만 채워 검사한다 — 빠지거나 틀리면 `CHECK_FAILED`로 재생성, 코드가 대신 정하지 않는다(10/8 E-69).
- 레시피에는 로봇·현장 값(TCP, 조립 원점, 그리퍼 명령값)을 넣지 않는다. 조립 원점은 `src/d2_robot/d2_bringup/config/robot.yaml`의 `assembly_origin` 하나만 쓴다.
- CAD 핸들(`cad.handle`) · `source_cad.sha256`은 CAD 대응 검증용이다 — 사람 · 노드 사이에서는 블록 이름만 쓴다.

## 남은 일

- 변환기 ① 본체(`blocks_to_recipe` — `d2_task` 안 별도 파일, 10/8 E-58)와 task 노드 연결(`task_node.py`의 `DesignChecker(cfg)`에 `blocks_to_recipe` 인자): 한세교 W110, 10/10 오전 PR — AI 칸(`role` · `part` · `stage` · `grasp`) 받기 · 공식 채움 없음 · 역할 목록 파일(`roles.json`, 이름은 안)(10/8 E-69).
