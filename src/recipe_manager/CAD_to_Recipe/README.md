# CAD를 조립 레시피로 변환

통합 저장소에서는 레시피를 로봇 목표(TCP)로 바꾸는 일을 집기·놓기 노드(`src/d2_motion`)가 한다(W104: pose = 블록 중심). 이 도구는 레시피까지만 만든다.

CAD(STEP·DXF)는 **정의**만 담는다. 어떤 부품이 어디에 놓이는지가 여기에 해당한다. 어떤 순서로 어떻게 놓을지는 **레시피**(작업지시서)가 정한다. 이 코드는 CAD를 한 번 읽어 모델을 만들고, 사람이 작성한 계획을 검증해 레시피 JSON을 만든다. 로봇 좌표·TCP·그리퍼 설정(현장 설정)과 실행 기록(LOT)은 다루지 않는다.

## 설치

저장소 루트에서 Python 3.10 이상으로 실행한다.

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r src/recipe_manager/requirements.txt
```

DXF만 쓴다면 `ezdxf`와 `numpy`만 있으면 된다. `cadquery`는 STEP을 읽을 때만 필요하다.

## 사용법

```bash
# 1) CAD → 모델 + 계획 양식
python src/recipe_manager/CAD_to_Recipe/main.py inspect src/recipe_manager/cad/001_chair_bench.dxf --model-id 001_CHAIR_BENCH
#    src/recipe_manager/recipes/001_CHAIR_BENCH.model.json  부품 치수, 인스턴스별 목표 중심·회전 (참고용)
#    src/recipe_manager/recipes/001_CHAIR_BENCH.plan.json   단계마다 sequence, stage, grasp(파지 방법) 입력

# 2) 계획 파일 작성 (DXF INSERT에 SEQ/STAGE/GRASP 속성이 있으면 미리 채워져 있음)

# 3) CAD + 계획 → 레시피
python src/recipe_manager/CAD_to_Recipe/main.py build src/recipe_manager/cad/001_chair_bench.dxf src/recipe_manager/recipes/001_CHAIR_BENCH.plan.json
#    src/recipe_manager/recipes/001_CHAIR_BENCH.recipe.json      레시피 (작업지시서)
#    src/recipe_manager/recipes/001_CHAIR_BENCH.placements.csv   배치표 (아래 설명)
```

`build`는 검증을 통과하면 **배치표**를 터미널에 출력하고 CSV로도 저장한다. sequence 순서대로 한 줄에 한 단계씩, 레시피 단계(순번, 단계, 잡기 축, 지지 부품)와 CAD 목표 배치(치수, 중심 좌표, 방향)를 나란히 보여 준다. 방향 `+Y/-X/+Z`는 부품의 L/W/T 축이 각각 모델의 +Y, −X, +Z를 향한다는 뜻이다.

출력은 항상 `src/recipe_manager/recipes/` 폴더에 `<모델ID>.model.json`, `<모델ID>.plan.json`, `<모델ID>.recipe.json`, `<모델ID>.placements.csv`로 저장된다. 어느 폴더에서 실행해도 위치는 같다.

| 계획 값 | 입력 내용 |
|---|---|
| `sequence` | 배치 순서 1..N, 빠짐·중복 없음 |
| `stage` | 1부터 시작하고, 순서를 따라 같거나 1씩 증가 |
| `grasp` | 파지 방법 `<상태>_<LONG\|SHORT>` 6가지([IRD 2장 잡기](../../../docs/02_인터페이스_IRD_v3_100620.md#2-공통-값)). 상태 `FLAT`(눕힘)·`EDGE`(옆세움)·`STAND`(세움)는 CAD 배치와 같아야 한다. `LONG`은 수평 치수 중 긴 쪽, `SHORT`는 짧은 쪽을 끼운다. 예: 젠가 벽 `FLAT_SHORT`(25 mm), 좌판 `FLAT_LONG`(75 mm). 닫힘 축 `grasp_axis`는 `build`가 계산한다 |

`build`는 CAD를 다시 읽어 모델을 만든다. 계획의 CAD 해시가 다르면 거부하므로, CAD가 바뀌었으면 `inspect`부터 다시 한다. 같은 이름의 출력 파일이 이미 있으면 **[1] 덮어쓰기 / [2] 다른 이름으로 저장(같은 폴더) / [3] 취소** 중에서 고른다. 입력을 받을 수 없는 환경에서는 기존 파일을 지키기 위해 취소한다.

## 레시피 검사 항목

- 모든 부품을 정확히 한 번 배치한다.
- `sequence`와 `stage` 규칙을 지킨다.
- 잡기 축이 수평이다.
- 부품끼리 겹치지 않고 책상 아래에 있지 않다.
- 책상 위가 아닌 부품은 먼저 놓인 부품 위에 놓인다. 이 관계를 `support_instance_ids`로 기록한다.

실제 파지·충돌·안정성·로봇 도달 가능성은 검사하지 않는다.

## 입력 조건

- 단위 mm, 오른손 좌표계, Z 위쪽, 책상면 Z=0. CAD 원점을 유지한다.
- 축에 평행한 직육면체 부품만 지원한다.
- STEP은 부품마다 독립된 평면 6면 솔리드여야 한다.
- DXF는 `INSUNITS=4`이고, 부품마다 3DFACE 6개로 된 블록의 INSERT여야 한다(스케일 1, 배열 아님).
- 인스턴스 ID는 CAD 원본 ID(`STEP_SOLID_0001`, `DXF_INSERT_<handle>`)를 쓴다.

## 코드 구조

폴더는 데이터 층을 따른다. 각 층의 클래스(dataclass·Enum)는 그 층의 `defined/` 폴더에 있고, 클래스 하나가 같은 이름의 파일 하나에 들어 있다.

```
CAD_to_Recipe/
  main.py                 명령줄 입구: 인자 해석 → RecipeManager 호출
  RecipeManager.py        레시피 작업 관리: inspect_cad / build_recipe, 파일 저장(덮어쓰기 질문)
  cad/                    CAD 파일 읽기
    CadReader.py            CadReader.read_cad(): STEP·DXF → [RawBox]
    defined/
      RawBox.py             CAD에서 읽은 부품 하나의 원시 데이터 (꼭짓점, DXF 속성 힌트)
  model/                  모델 층: 무엇을 어디에 (CAD 정의)
    defined/
      BoundingBox.py        축 정렬 경계 상자, 겹침·위에 놓임 판정
      CADPartDefinition.py  부품 정의 = 규격 (같은 치수 공유)
      CADPartInstance.py    부품 인스턴스 = 놓인 부품 하나의 목표 위치·방향 (DXF INSERT / STEP 솔리드 1개)
      Model.py              CADPartDefinition·CADPartInstance 묶음, RawBox → 모델 변환과 배치 검사
  recipe/                 레시피 층: 어떤 순서로 어떻게 (작업지시서)
    defined/
      GraspMethod.py        파지 방법 6가지 (FLAT/EDGE/STAND × LONG/SHORT)
      GraspAxis.py          닫힘 축 LENGTH / WIDTH / THICKNESS (계산 값)
      Step.py               단계 하나 (순서, 단계, 파지 방법, 닫힘 축, 지지 부품)
      Plan.py               사람이 채우는 레시피 초안, 양식 생성과 JSON 읽기
      Recipe.py             계획 검증 → 레시피, 배치표(make_placement_rows)
  tests/test_cad_to_recipe.py   단위 테스트 (작업 공간에만 있음, 통합 저장소에는 넣지 않음)
```

처리 흐름:

```
CadReader.read_cad(CAD) → [RawBox] → Model.create_from_raw_boxes → Model ─┬─ Plan.create_template → plan.json (사람이 작성)
                                                            └─ Recipe.build_from_plan(Model, Plan.create_from_dict(plan.json)) → recipe.json, placements.csv
```

## 테스트

테스트는 작업 공간에만 있다(통합 저장소에는 넣지 않음).

```bash
python src/recipe_manager/CAD_to_Recipe/tests/test_cad_to_recipe.py
```

CAD 라이브러리가 없으면 해당 테스트는 `skipped`가 된다. 이 경우 실제 STEP/DXF 변환은 검증되지 않은 것이다. LV1 회귀 테스트는 `src/recipe_manager/cad/001_chair_bench.dxf`의 결과를 기존 `archive/03_Recipes/lv1_bench.recipe.json`과 대조한다.
