# web/backend — AI 설계 생성 파트 설계 노트 (W108, 10/7 황인재)

> **10/8 E-69로 바뀜:** 템플릿 길(4장 · `templates.py` · `tune_system.txt` · `param_schema.json`) · `base_designs/`(10/3 lv*.json) · `blocks/1` 출력은 **옛것**이다. 지금 기준은 [../README.md](../README.md) 4장 — GPT-4o가 `blocks/2.0`(role · part · stage · grasp 포함)을 직접 쓰고 후보 3개 중 사람이 1개를 고른다. 이 노트의 피드백 문구 · 기록 · 시험 틀(3 · 8 · 9장)은 그대로 쓴다.


> 정본은 [SDD 3.1.1 · 6.6](../../docs/03_설계_SDD_v3_101012.md) · [IRD 6 · 7 · 8.3장](../../docs/02_인터페이스_IRD_v3_101012.md) · 결정 E-08 ~ E-15 · E-40 · E-41. 이 노트는 그것을 **코드로 옮길 때의 세부**(파일 · 함수 · 프롬프트 · 스키마 · 시험)만 적는다. 정본과 다르면 정본이 맞다.
> 이 폴더의 다른 부분(REST · WebSocket · MQTT · DB)은 W126 · W111에서 더한다. 여기는 **AI 파트**만.

## 1. 범위 — 무엇을 만들고 무엇을 안 만드나

| 만든다 (HMI, 웹 PC) | 안 만든다 (다른 파트 · 나중) |
|---|---|
| `design_gen.py` DesignGenerator — 2단계 생성(튜닝 → 자유) · 검사 피드백 재생성 루프(최대 2번) · 기록 | 검사 계산 `design_checker.py`(비전 · `d2_task`, W109) — 1단계는 10/3 `jenga_check` 계산을 **로컬 검사기로 임시 사용** |
| `templates.py` — 기본 설계 4개 기준 템플릿 생성기(`build_template`) | 레시피 변환기 ①(한세교) — AI는 블록 JSON까지만(E-15) |
| `prompts/` — 튜닝 · 자유 시스템 프롬프트 2개, JSON Schema 2개 | 미리보기 — frontend `Preview3D`(three.js, E-40). PNG 없음 |
| `trial_w108.py` — 요청 18개 시험 실행기(V-44) + 결과표 | DB `design_store.py`(W111, 같은 날 오후) — 1단계는 `base_designs/` 파일 4개만 읽음 |

## 2. 파일과 역할

```
web/backend/
├── design_gen.py        DesignGenerator — generate(text) → 저장할 설계 또는 OUT_OF_SCOPE / GEN_FAILED (ROS · FastAPI 없음, pytest 가능)
├── templates.py         템플릿 4종 생성기 + 숫자 범위(FR-G02). 10/3 jenga_check.build_template 손질
├── check_local.py       LocalChecker — 10/3 jenga_check 계산(형식 · 파고듦 · 받침 · 안정성 7 mm) → check_result/1 모양으로 답함. W109 뒤 MQTT check_design으로 교체(§5)
├── prompts/
│   ├── tune_system.txt  ① 튜닝 프롬프트 — 템플릿 4종 · 숫자 범위 · family 판단
│   ├── free_system.txt  ② 자유 생성 프롬프트 — 좌표 · 방향 코드 · 규칙 6개 · {examples} 자리
│   ├── param_schema.json  ①의 구조화 출력 스키마 (strict)
│   └── blocks_schema.json ②의 구조화 출력 스키마 (strict)
├── base_designs/        기본 설계 4개 blocks/1 — RAG 예시 · 템플릿 기준값 (W117 비전 변환기 ②가 주면 교체. 1단계는 10/3 lv1 · lv2 · lv4에서 변환 — §7)
└── trial_w108.py        V-44 시험 실행기. 결과는 out_w108/<시각>/ (gitignore)
```

- **한 곳 규칙**(SDD 3.1.1): OpenAI 호출은 `design_gen.py` 안 `_call_llm()` 한 함수만. 키는 `OPENAI_API_KEY` 환경 변수(`.env` → `--env-file`), 코드 · 기록에 절대 안 들어감.
- 생성기는 **검사기와 저장소를 호출 가능 객체로 받는다**: `DesignGenerator(checker, examples_for)`. 10/7 = `LocalChecker` · 파일 폴더, 10/8 = `MqttClient.request('d2/task/check_design')` · `DesignStore.examples_for`. 생성 코드는 바뀌지 않는다.

## 3. 흐름 — `DesignGenerator.generate(text, requested_by)`

```
text ──① 튜닝 호출(tune_system + param_schema)──► {decision, family, template, params, count, reason}
          decision reject ──────────────────────► OUT_OF_SCOPE (가구 2종 밖 · 숫자 범위 밖 · 54개 초과, reason 글) — 끝, LLM 1번
          decision template ─► templates.build(template, params, count) ─► blocks/1   (gen_path = "param")
             코드 쪽 범위 검사도 실패(LLM이 범위를 어김) ─► OUT_OF_SCOPE (이유 = ValueError 글) — 끝
          decision free ─► ② 자유 호출(free_system + examples + blocks_schema) ─► blocks/1   (gen_path = "free")
       ③ checker(blocks/1) ─► check_result/1
          ok ─► 결과 {blocks, check, gen_path, prompt 기록} 반환 → (W111) store.save_design → 화면
          실패 ─► feedback = errors[].detail 줄들 → 같은 길로 다시(①은 피드백을 담아 튜닝 재호출, ②는 대화에 이어 재생성) — 최대 2번
       3번 모두 실패 ─► GEN_FAILED {errors, attempts}  → 화면 이유 + 기본 설계 권함
```

| 항목 | 정한 것 |
|---|---|
| 모델 | `gpt-4o`, temperature 0, 요청마다 새 대화. `response_format = {"type":"json_schema", "json_schema":{"strict":true, …}}` → 파싱 실패 0(FR-G04) |
| 시간 | 호출당 timeout 30 s, 네트워크 오류 재시도 1(TR-12). 요청 → 결과 목표 ≤ 60 s. 3회 실패 전이라도 60 s를 넘기면 그 시점 결과로 GEN_FAILED |
| 재생성 | 튜닝 길: 피드백을 user 메시지로 더해 **튜닝을 다시**(숫자를 바꾸게). 2번째도 실패면 자유 길로 넘어가지 않는다(템플릿이 실패하면 숫자 문제 — 자유 생성이 더 나을 이유 없음). 자유 길: 10/3 `run_v09`처럼 assistant 응답 + 피드백을 이어 붙여 재호출 |
| 피드백 문구 | `"검사기 결과:\n- {detail}\n- {detail}\n규칙에 맞게 고친 설계 전체를 같은 JSON 형식으로 다시 답하라."` — detail은 `check_result/1 errors[].detail` 그대로(IRD 6장) |
| 기록(NFR-17) | 시도마다 {stage, prompt_sha, messages, raw_response, elapsed_ms, check} 목록 → 반환값 `attempts`. W111에서 `designs`(통과) · `rejected`(실패)에 같이 저장 |
| 개수 2 | `count = 2`(같은 가구 두 개)는 생성 · 저장. E-18의 '조립 보류'를 여기에도 적용할지는 PL 확인. 요청 · 시험 문장에 '2인용'은 쓰지 않고 모양으로 적는다(10/7 황인재 — 뜻이 애매함) |

## 4. 템플릿 4종 (`templates.py`) — 기본 설계 4개(E-14) 기준

좌표 · 방향 코드는 `blocks/1`(mm, 설계 좌표계, `ori` x · y · xe · ye · zx · zy). 기본값이면 **기본 설계와 블록 하나까지 같아야 한다**(pytest로 `base_designs/`와 비교).

| 템플릿 | 숫자(기본값) · 범위(FR-G02 제안) | 만드는 법 | 기본값 = 기본 설계 |
|---|---|---|---|
| `chair` | `legs` 다리 벽 층수(4) 2~6 · `back` 등받이 층수(5) **0~6, 0이면 벤치** · `seat_w` 좌판 블록 수(3) 2~4 · `count`(1) 1~2 | 다리 벽: x = ±25, `y`, z = k·15 (k < legs) → 좌판: `x`, z = legs·15, y = (i − (seat_w−1)/2)·25 → 등받이: `x`, y = 좌판 맨 뒤 y, z = (legs+1)·15 부터 back층 | back 5 → `002_CHAIR_BACK`(16) · back 0 → `001_CHAIR_BENCH`(11) — CAD placements.csv와 좌표 일치 확인함(10/7) |
| `desk_std` | `top_len` 상판 블록 수(3) 2~4 · `count` 1~2 | 세운 다리 `zx` 4개 (±30, ±25) → 보 `x` 2개 z = 75, y = ±25 → 상판 `y` z = 90, x = (i − (top_len−1)/2)·25 | top_len 3 → `003_DESK_STAND`(9) — placements.csv와 일치 확인함 |
| `desk_wall` **(안)** | `legs` 다리 벽 층수(6) 3~8 · `top_len`(3) 2~4 · `shelf` 중간 선반(false) · `count` 1~2 | 벤치와 같은 다리 벽 2개 × legs층 + 상판 top_len개. `shelf = true`면 legs/2 층 위에 선반 3개를 끼우고 그 위로 다시 벽(책장 1단 방식) | **한세교 CAD(W074, 10/7 오후~10/8)가 나오면 벽 간격 · 층수 · 상판을 맞춤.** 그 전에는 벤치 6층 자리값으로 둠 |
| 공통 | 블록 합계 ≤ 54 · `count` 2면 좌우(x)로 50 mm 띄워 복제(10/3 `COPY_GAP_MM`) | 범위 밖 → `ValueError(이유 글)` → OUT_OF_SCOPE | |

10/3의 bookshelf · corbel_arch · chair_standing은 **뺀다**(가구 2종 밖, E-10). 자유 생성 프롬프트 예시도 기본 설계 4개(+RAG 파생본)만 넣는다.

## 5. 검사 연결 — 1단계 로컬, 2단계 MQTT

| | 1단계 (10/7 W108) | 2단계 (10/8~, W109 · W126 · W127 뒤) |
|---|---|---|
| 누가 | `check_local.LocalChecker` — 10/3 `jenga_check`의 형식 · 파고듦 · 받침 · 안정성(쌓는 도중 최소 여유 ≥ 7 mm) | task `DesignChecker`(비전) — 위 4개 + 잡기 틈 · 막힌 칸 · 작업영역 + 변환기 ① 레시피 |
| 어떻게 | 함수 호출 | `MqttClient.request('d2/task/check_design', blocks/1)` → `…/res`(req_id, 5 s) |
| 답 모양 | **둘 다 `check_result/1`** `{ok, min_margin_mm, errors:[{block, reason, detail}], recipe?}` — 로컬은 `recipe` 없음 | 같음 |
| detail 글 | 10/3 `summary()` 대신 블록 단위로: `"n번 공중(받침 없음)"` · `"n번 ↔ m번 겹침"` · `"n번 단계 여유 3.0 mm < 7"` · `"형식: order가 1~N 연속이 아님"` | IRD 6장 예시와 같은 투 |

- **요청 · 응답 모양(10/7 15시 PL, main 9d8425d · IRD 4.2):** 요청은 `blocks/1` 객체를 **그대로** 보낸다(바깥 상자에 다시 넣지 않음, 다리가 `req_id`만 더함). 응답은 두 층이다 — 서비스 `success`(요청을 처리했나) · `check_result/1`의 `ok`(설계 합격). **`success = true` + `ok = false`만 LLM 피드백으로 돌린다.** `success = false`나 5초 `TIMEOUT`은 설계 문제가 아니므로 재생성하지 않고 이유를 붙여 멈춘다(재생성 횟수를 쓰지 않음).
- 로컬 검사기는 **임시**다. 10/3 `jenga_check.py`는 `docs/research/`에 그대로 두고(지우지 않음), 계산 함수(`Block` · `check` · `penetrations` · `step_margins` · `format_errors`)만 `check_local.py`로 옮긴다. 10/3 코드의 **내민 구조 버그**(SDD 6.7 ④)는 비전이 `design_checker.py`에서 고친다 — 로컬 검사기는 10/3 그대로 쓰고 결과표에 "로컬 검사기(10/3 계산)"라고 적는다.

## 6. 프롬프트 · 스키마 (`prompts/`) — 파일이 정본, 여기는 요지

- **`tune_system.txt`**: 10/3 `prompt_v09_fallback` 손질. 바뀐 것 ① 템플릿 6종 → **4종**(§4 표의 이름 · 숫자 · 기본값 · 범위 · 블록 수 식) ② **`decision` 칸 추가**(`template` · `free` · `reject`) — 자유 길과 거절을 LLM이 분명히 가르게 한다(10/3은 `template: null` 하나라 둘을 구분 못 했음) ③ **`family` 칸 추가**(`chair` · `desk` · `null`) ④ 피드백이 오면 숫자를 바꿔 다시 고르라는 문장.
- **`free_system.txt`**: 10/3 `prompt_v09` 손질. 바뀐 것 ① 템플릿 6종 본문 → **`{examples}` 자리**(RAG가 기본 설계 · 파생본 · 조립 결과 글을 채움) ② 규칙 6개 그대로(받침 · 파고듦 · order · 7 mm · ≤ 54 · 요청 지킴) + "로봇 손가락이 들어갈 틈" 한 줄(검사 5번에 대비) ③ 출력은 `{"blocks":[…]}`만(schema · design_id · family는 코드가 붙임).
- **`param_schema.json`**(strict): `{decision: enum[template,free,reject], family: enum[chair,desk]|null, template: enum[chair,desk_std,desk_wall]|null, params: {legs,back,seat_w,top_len: int|null, shelf: bool|null}, count: int, reason: string|null}` — strict 모드는 모든 칸이 `required`라 **안 쓰는 숫자는 null**로 받는다. 코드가 null을 빼고 `build_template`에 넘긴다.
- **`blocks_schema.json`**(strict): `{blocks: [{order:int, x:number, y:number, z:number, ori: enum 6}]}` 최대 54개. `inferred`는 코드가 false로 붙인다.

## 7. 기본 설계 4개 파일 (`base_designs/`, 1단계)

| 파일 | 출처 | 상태 |
|---|---|---|
| `bench.json` | 10/3 `lv1_bench.json` → blocks/1 (= `001_CHAIR_BENCH` placements, 확인함) | 1단계 |
| `chair.json` | 10/3 `lv2_chair.json` → blocks/1 (= `002_CHAIR_BACK` 16개 — placements 대조는 구현 때) | 1단계 |
| `desk_std.json` | 10/3 `lv4_table_standing.json` → blocks/1 (= `003_DESK_STAND`, 확인함) | 1단계 |
| `desk_wall.json` | **없음** — 한세교 CAD(W074) → 비전 변환기 ②(W117) | 받으면 넣음 |

`design_id`는 W111(DesignStore)에서 정한다 — 1단계 안: `bench` · `chair` · `desk_std` · `desk_wall`(IRD 예시 `"design_id":"bench"` 투). `family`: `chair` · `desk`.

## 8. W108 시험 (`trial_w108.py`, V-44) — 완료 기준: 길 선택 맞음 · 검사 통과 ≥ 2/3 · 파싱 실패 0 · 시간 기록

| 구분 | 요청(안) | 기대 길 |
|---|---|---|
| 튜닝 10 | 다리가 5층인 벤치 · 등받이가 높은 의자 · 등받이 없는 의자 · 다리가 낮은 의자 · 좌판이 넓은 벤치 · 의자 2개 · 상판이 긴 책상 · 선반 있는 책상 · 다리를 세운 책상 · 블록 20개로 벤치 | `param` (template ≠ null) |
| 자유 5 | 좌판이 좌우로 긴 벤치 · 높은 의자 · 작은 탁자 · 등받이가 비스듬한 의자 · 다리가 넓게 벌어진 책상 | `free` (template null, family ≠ null) |
| 범위 밖 3 | 침대 · 계단 3칸 · 다리 20층인 의자 | `OUT_OF_SCOPE`(family null / 숫자 범위 밖) |

- 10/3 `run_v09` 방식 그대로: 요청마다 새 대화, 첫 응답으로 채점, 응답을 손으로 안 고침. 재생성 결과는 따로 적는다.
- 결과표 `out_w108/<시각>/results.md`: `# · 요청 · 기대 길 · 고른 길 · 맞음 · 형식 · 파고듦 · 최소 여유 · 통과(첫) · 재생성 · 블록 수 · 시간 ms`. 요청마다 `NN_tryK_raw.txt` · `NN_tryK.json`. **PNG 없음**(E-40) — 눈으로 볼 때는 10/3 `jenga_check.py --png`를 그 폴더에서 손으로.
- `--dry-run`: 키 없이 템플릿 4종 기본값이 `base_designs/`와 같은지 + 로컬 검사기 통과 확인. `--ping`: 키 · 모델 확인 1번.
- 결과는 [04 결과표 V-44](../../docs/04_결과_결과표_v2_101012.md)에 숫자로, 프롬프트 sha · 모델 · 날짜와 함께.

## 9. 시험(pytest, `tests/web/`) — 키 없이

- `test_templates.py`: 4종 기본값 = `base_designs/`(블록 하나까지) · 범위 밖 → ValueError · count 2 복제 · 합계 ≤ 54.
- `test_check_local.py`: 기본 설계 4개 통과 · 공중 블록 · 겹침 · 여유 부족 샘플이 `check_result/1` 모양으로 실패.
- `test_design_gen.py`: LLM을 가짜 함수로 바꿔(응답 글자를 미리 줌) 길 선택 · 재생성 2번 · GEN_FAILED · OUT_OF_SCOPE · 60 s 초과 처리.

## 10. 정할 것 · 다른 파트에서 받을 것

| 무엇 | 누구 · 언제 | 그 전까지 |
|---|---|---|
| `desk_wall` 치수(벽 간격 · 층수 · 상판 · 선반 자리) | 한세교 W074(10/7 오후~10/8) | 벤치 6층 자리값(안)으로 시험, 결과표에 '안' 표시 |
| 기본 설계 4개 blocks/1 정본 | 비전 W117(변환기 ②) | 10/3 JSON 변환본(§7) |
| FR-G02 숫자 범위 확정(위 표는 '제안') | PL(황인재) — W108 결과 보고 10/8 | 표 값 |
| `check_design` MQTT 왕복 · `req_id` | W126(황인재) · W127(다리) · W109(비전) | LocalChecker |
| `design_id` 규칙 · `rejected` 보관 | W111 | 반환값만 |
| 파생 설계 `family` 글 ↔ IRD 예시 `"family":"chair"` 일치 | IRD 그대로 `chair` · `desk` | — |
