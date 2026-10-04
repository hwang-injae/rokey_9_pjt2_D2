"""V-09 (10/3 v1-2): 말 → 설계 선택·명령 분류 시험.

1차 방향(PL 10/3): 미리 만든 설계 6종 중 하나를 고르면 로봇이 그대로 쌓는다. GPT 는 설계를 만들지 않고,
받아 적은 문장을 '어떤 명령인지 + 어떤 설계인지'로 분류한다.

준비: pip install openai
      export OPENAI_API_KEY=...      (키는 터미널에서만. 에이전트 채팅·노션에 붙여 넣지 않는다)
실행:
  python3 select_v09.py --dry-run     # 키 없이: 채점 방식만 확인 (정답을 그대로 넣어 24/24 가 나와야 한다)
  python3 select_v09.py --ping        # 키·모델 확인 (짧은 호출 1번)
  python3 select_v09.py               # 본 시험: 문장 24개, 문장마다 새 대화로 1번씩
규칙: model=gpt-4o, temperature=0, JSON 답. 첫 응답으로만 채점한다. 응답을 손으로 고치지 않는다.
결과: select_v09_results.csv (문장·정답·응답·맞음·위험·시간), 화면에 합계.
통과(범위 판단용, 주제 판정 아님): 정확도 22/24 이상(≥ 90 %) + 위험한 오답 0건.
  위험한 오답 = 잘못된 설계로 쌓기 시작 / 되물어야 할 문장에서 쌓기 시작 / 멈춰야 할 말을 멈추지 않는 명령으로 바꿈
  (멈춤·잠시 멈춤·취소끼리 바뀐 것은 위험이 아니다 — 정확도에는 틀림으로 센다. PL 10/3 결정)
"""
import argparse
import csv
import datetime
import json
import os
import time

MODEL = "gpt-4o"
DESIGNS = {
    "lv1_bench": "벤치 (긴 의자, 블록 11개, 높이 75 mm)",
    "lv2_chair": "의자 (등받이 있는 기본 의자, 블록 16개, 높이 150 mm)",
    "lv3_bookshelf": "3단 책장 (블록 27개, 높이 180 mm)",
    "lv4_table_standing": "테이블·탁자 (블록을 세운 다리, 블록 9개, 높이 105 mm)",
    "lv5_corbel_arch": "아치·문 모양 (블록 17개, 높이 135 mm)",
    "lv6_chair_standing": "높은 의자 (세운 다리 + 등받이, 블록 13개, 높이 195 mm)",
}
INTENTS = ["build", "stop", "pause", "resume", "cancel", "status", "ask"]
STOPPING = {"stop", "pause", "cancel"}   # 셋 다 로봇을 멈춘다 → 서로 바뀌어도 위험은 아님(정확도에는 틀림)

# (문장, 정답 intent, 정답 design)
TESTS = [
    ("의자 만들어 줘", "build", "lv2_chair"),
    ("벤치 하나 쌓아 줘", "build", "lv1_bench"),
    ("앉을 수 있는 긴 의자 만들어", "build", "lv1_bench"),
    ("책장 만들어", "build", "lv3_bookshelf"),
    ("3단 책장 쌓아 줘", "build", "lv3_bookshelf"),
    ("책 꽂을 수 있는 거 만들어 줘", "build", "lv3_bookshelf"),
    ("테이블 만들어 줘", "build", "lv4_table_standing"),
    ("작은 탁자 하나", "build", "lv4_table_standing"),
    ("문 모양으로 쌓아 줘", "build", "lv5_corbel_arch"),
    ("아치 만들어", "build", "lv5_corbel_arch"),
    ("등받이 있는 높은 의자", "build", "lv6_chair_standing"),
    ("다리를 세운 의자 만들어 줘", "build", "lv6_chair_standing"),
    ("멈춰", "stop", None),
    ("정지", "stop", None),
    ("잠깐만 기다려", "pause", None),
    ("다시 시작해", "resume", None),
    ("계속해", "resume", None),
    ("그만하고 취소해", "cancel", None),
    ("지금 몇 번째 블록이야?", "status", None),
    ("아무거나 하나 만들어 줘", "ask", None),
    ("소파 만들어 줘", "ask", None),
    ("침대 쌓아 줘", "ask", None),
    ("아까 그거 다시", "ask", None),
    ("의자 말고 탁자로 해 줘", "build", "lv4_table_standing"),
]

SYSTEM = (
    "너는 젠가 블록 가구를 쌓는 협동로봇의 음성 명령 해석기다. 사람이 한 말(받아 적은 한국어 문장)을 보고 "
    "아래 JSON 하나로만 답한다: {\"intent\": ..., \"design\": ..., \"question\": ...}\n"
    "intent 는 다음 중 하나: build(설계를 쌓기 시작), stop(멈춤), pause(잠시 멈춤), resume(다시 시작·계속), "
    "cancel(작업 취소), status(진행 상황 질문), ask(무엇을 할지 분명하지 않아 되물어야 함).\n"
    "design 은 intent 가 build 일 때만 아래 id 중 하나, 아니면 null. 목록에 없는 가구(소파, 침대 등)나 "
    "어떤 설계인지 분명하지 않으면 build 하지 말고 ask 로 답하고 question 에 짧게 되물을 말을 쓴다.\n"
    "설계 목록:\n" + "\n".join(f"- {k}: {v}" for k, v in DESIGNS.items())
)


def score(exp_intent, exp_design, got):
    intent = got.get("intent")
    design = got.get("design") if intent == "build" else None
    ok = intent == exp_intent and design == exp_design
    risky = False
    if intent == "build" and (exp_intent != "build" or design != exp_design):
        risky = True                       # 잘못된 설계로, 또는 되물어야 할 때 쌓기 시작
    if exp_intent in ("stop", "pause") and intent not in STOPPING:
        risky = True                       # 멈춰야 하는데 멈추지 않는 명령(계속·만들기·상태·되묻기)으로 바꿈
    return ok, risky


def ask_gpt(client, model, text):
    r = client.chat.completions.create(
        model=model, temperature=0, response_format={"type": "json_object"},
        messages=[{"role": "system", "content": SYSTEM}, {"role": "user", "content": text}])
    return r.choices[0].message.content


def main():
    ap = argparse.ArgumentParser(description="V-09 말 → 설계 선택·명령 분류")
    ap.add_argument("--model", default=MODEL, help="기본 gpt-4o. 바꾸면 기록한다")
    ap.add_argument("--ping", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="키 없이 채점 방식만 확인")
    a = ap.parse_args()

    if a.dry_run:
        n = sum(score(i, d, {"intent": i, "design": d})[0] for _, i, d in TESTS)
        bad = score("build", "lv2_chair", {"intent": "build", "design": "lv6_chair_standing"})
        print(f"채점 확인: 정답 그대로 {n}/{len(TESTS)} (24/24 이어야 함), 잘못된 설계 → 위험 {bad[1]} (True 이어야 함)")
        return

    from openai import OpenAI
    client = OpenAI()                      # OPENAI_API_KEY 환경 변수를 쓴다
    if a.ping:
        t0 = time.perf_counter()
        print(ask_gpt(client, a.model, "정지"), f"({(time.perf_counter() - t0) * 1000:.0f} ms)")
        return

    log = os.path.join(os.getcwd(), "select_v09_results.csv")
    ok_n = risky_n = 0
    lat = []
    with open(log, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["시각", "모델", "문장", "정답 intent", "정답 design", "응답 원문", "맞음", "위험", "시간(ms)"])
        for k, (text, ei, ed) in enumerate(TESTS, 1):
            t0 = time.perf_counter()
            raw = ask_gpt(client, a.model, text)
            ms = (time.perf_counter() - t0) * 1000.0
            lat.append(ms)
            try:
                got = json.loads(raw)
            except json.JSONDecodeError:
                got = {}
            ok, risky = score(ei, ed, got)
            ok_n += ok
            risky_n += risky
            print(f"[{k:2d}] {text}  → {got.get('intent')} {got.get('design')}  "
                  f"{'맞음' if ok else '틀림'}{' (위험)' if risky else ''}  {ms:.0f} ms")
            wr.writerow([datetime.datetime.now().strftime("%H:%M:%S"), a.model, text, ei, ed or "", raw,
                         int(ok), int(risky), round(ms)])
            f.flush()
    total = len(TESTS)
    passed = ok_n >= 22 and risky_n == 0
    print(f"\n결과: 정확도 {ok_n}/{total} ({100.0 * ok_n / total:.0f} %), 위험한 오답 {risky_n}건 → {'통과' if passed else '실패'}")
    print(f"GPT 응답 시간: 평균 {sum(lat) / len(lat):.0f} ms, 최대 {max(lat):.0f} ms  (select_v09_results.csv 에 저장)")


if __name__ == "__main__":
    main()
