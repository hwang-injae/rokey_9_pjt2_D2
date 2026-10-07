# recipe_manager — CAD 설계에서 조립 레시피까지

젠가 가구의 CAD 설계를 **조립 레시피**(어떤 블록을, 어떤 순서로, 어떻게 잡아서 놓는가)로 바꾼다. ROS 노드가 아니라 **미리 돌리는 도구**다. 담당: 한세교(CAD).

- 같은 폴더의 `COLCON_IGNORE` 때문에 `colcon build`는 이 폴더를 건너뛴다(ROS 패키지가 아님).
- 레시피를 로봇 목표(TCP)로 바꾸는 일은 집기·놓기 노드(`src/d2_motion`)가 한다. 작업 폴더의 `Recipe_to_Robot`(두산 직접 실행)은 이 저장소에 넣지 않았다 — W104 합의(pose = 블록 중심)로 역할이 집기·놓기에 있고, 로봇을 움직이는 코드는 팀 정지 규칙(서기 궤적·`SignalHandlerOptions.NO`)을 따라야 하기 때문.

```
cad/*.dxf ──inspect──▶ recipes/<ID>.plan.json ──(사람이 순서·단계·파지 방법 확인)──▶ build
                                                                                    │
           recipes/<ID>.recipe.json + .placements.csv ◀───────────────────────────────┘
                     │
                     └──▶ 작업 판단(d2_task)이 읽음 → PickPlace(pose = 블록 중심) → 집기·놓기(d2_motion)
```

## 폴더

| 폴더·파일 | 내용 |
|---|---|
| `cad/` | 설계 원본 3종(의자 2 · 책상 1). DXF = 조립 정의(도구가 읽는 원본), STEP = 검사기용(치수 검증) |
| `recipes/` | 도구의 출력. `<ID>.plan.json`은 **사람이 작성한 입력**이라 CAD와 함께 레시피의 원본이다 |
| `CAD_to_Recipe/` | CAD → 모델·계획 양식(`inspect`) → 레시피(`build`). 사용법은 [CAD_to_Recipe/README.md](CAD_to_Recipe/README.md) |
| `requirements.txt` | `ezdxf`, `numpy`(필수), `cadquery`(STEP 읽기·검사기) |

## 쓰는 레시피 — 의자 2 · 책상 2 (10/6 한세교 결정)

레시피 형식은 `assembly.recipe/1.0`(이 도구의 출력, `src/d2_motion` `run_recipe`가 읽음)이다 — `assembly`(조립)는 너무 넓은 말이라 `cad_recipe/1.0`으로 바꾼다(10/7 한세교, 변환기 ②와 W121에서 시점을 정해 같이 바꿈). 레시피 안 모델 · 계획 형식은 10/7에 `cad_model/1.0` · `cad_plan/1.0`으로 바꿨다. 10/4 결정 D-16의 옛 형식 `m0609.jenga.cad_recipe/1.0`(`blocks[]` 구조)은 쓰지 않는다.

| 가구 | 모델 ID (파일 이름) | CAD (`design_id`) | 블록 | 층 | 크기 (mm) | 파지 방법 | 상태 |
|---|---|---|---|---|---|---|---|
| 의자 | `001_CHAIR_BENCH` (옛 LV1) | `001_chair_bench` | 11 | 5 | 75 × 75 × 75 | FLAT_SHORT 8, FLAT_LONG 3 | 1차 (10/6 실기 11/11, 시제품 코드) |
| 의자 | `002_CHAIR_BACK` (옛 LV2) | `002_chair_back` | 16 | 10 | 75 × 75 × 150 | FLAT_SHORT 8, FLAT_LONG 8 | 계획은 DXF 힌트 초안 — 순서·잡기 사람 확인 필요 |
| 책상 | `003_DESK_STAND` (옛 LV4) | `003_desk_stand` | 9 | 3 | 75 × 75 × 105 | STAND_SHORT 4, FLAT_LONG 5 | 계획은 DXF 힌트 초안 — **세운 블록 공급 칸 필요** |
| 책상 | (새로, `004_DESK_…`) | — | — | — | — | — | 한세교가 새 CAD로 만들 예정 |

- 크기는 레시피 CAD 치수(명목 블록 75 × 25 × 15 mm)로 계산한 바깥 크기다. 실측 블록은 74.5 × 24.8 × 14.75 mm다. 조립 작업 영역은 30 × 30 × 30 cm다.
- 세운 블록(STAND_*)은 같은 자세로 놓인 공급 칸에서 집어야 한다(재파지 없음). `003_DESK_STAND`를 쓰기 전에 세운 블록 칸을 교시하고 집기·놓기를 실기 확인한다.
- 이름 = `<번호 3자리>_<가구>_<모양>`(번호 = 의자 먼저, 책상 다음, 새로 만들면 004부터): 모델 ID(대문자)가 레시피 파일 이름과 `block_id` 앞글자(`001_CHAIR_BENCH_B001`)가 되고, CAD 파일 이름(소문자)이 `design_id`다(10/6 한세교, 옛 `LV1`·`lv1_bench`에서 바꿈).
- 책장(LV3)·아치(LV5)·세운 의자(LV6)는 쓰지 않아 뺐다. 원본은 한세교 작업 폴더에 있다.
- 이 폴더의 DXF와 계획으로 `build`를 다시 돌리면 같은 레시피가 나온다(10/6 확인).

## 명령 (저장소 루트에서)

```bash
pip install -r src/recipe_manager/requirements.txt

# 1) CAD → 모델 + 계획 양식 (DXF 속성 SEQ/STAGE/GRASP가 있으면 미리 채움)
python3 src/recipe_manager/CAD_to_Recipe/main.py inspect src/recipe_manager/cad/001_chair_bench.dxf --model-id 001_CHAIR_BENCH
# 2) recipes/001_CHAIR_BENCH.plan.json의 sequence·stage·grasp 확인
# 3) 검증 → 레시피 + 배치표
python3 src/recipe_manager/CAD_to_Recipe/main.py build src/recipe_manager/cad/001_chair_bench.dxf src/recipe_manager/recipes/001_CHAIR_BENCH.plan.json
```

같은 이름의 출력 파일이 있으면 `inspect`·`build`는 덮어쓰기 / 다른 이름으로 저장 / 취소를 묻는다.

## 지켜야 할 약속

- 단위 mm, 설계 원점 = 바닥 외곽 가운데, X 오른쪽·Y 뒤쪽·Z 위, 책상면 Z = 0.
- **잡기 이름은 파지 방법 6가지** `<FLAT|EDGE|STAND>_<LONG|SHORT>`(IRD 2장, S-25). 다리 벽 `FLAT_SHORT`, 좌판 `FLAT_LONG`. 상태(눕힘·옆세움·세움)는 CAD 배치와 같아야 하고, 공급 블록도 놓을 때와 같은 상태여야 한다(재파지 없음).
- 순서와 잡기는 형상으로 추론하지 않는다. 사람이 계획 파일에 쓰거나 CAD 속성 힌트를 초안으로 가져온다.
- 레시피에는 로봇·현장 값(TCP, 조립 원점, 그리퍼 명령값)을 넣지 않는다. 조립 원점은 `src/d2_bringup/config/robot.yaml`의 `assembly_origin` 하나만 쓴다.

## W104 합의 뒤 남은 일 (W105 · W106)

- 팀 형식으로 내보내기(W105): `block_id` = `<모델ID>_B<sequence 3자리>`(`001_CHAIR_BENCH_B001`…) — 10/7부터 레시피 `steps[]`에 `block_id`·`support_block_ids`로 적는다. `instance_id`(DXF 핸들)는 CAD를 다시 저장하면 바뀌어 CAD 추적용으로만 남긴다. 남은 것: `design_id` = `001_chair_bench`, `grasp`·`supports`, 블록 중심 x·y + `bottom_z_mm` + `size_mm`(CAD 명목값). yaw는 `size_mm`로 계산(긴 변이 CAD x면 0°).
- task 노드가 레시피를 읽는 경로(ROS 패키지 경로로 읽을 곳) — W106에서 정한다.
