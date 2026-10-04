"""V-09 실행기: 지시를 GPT-4o 에 하나씩 보내고, jenga_check.py 로 검사해 저장한다.

같은 폴더에 jenga_check.py, prompt_v09.txt, prompt_v09_fallback.txt 가 있어야 한다.
준비: pip install openai matplotlib     (matplotlib 은 그림용. 없으면 그림만 건너뛴다)
      export OPENAI_API_KEY=...          (키는 터미널에서만. AI 에이전트 채팅에 붙여 넣지 않는다)
실행:
  python3 run_v09.py --ping              # 키와 모델 확인 (짧은 호출 1번)
  python3 run_v09.py                     # 본 시험: 지시 15개
  python3 run_v09.py --fallback          # 대안: 템플릿 하나와 숫자만 고르게 한다 (템플릿 변형 지시 10개)
시험 규칙:
  - 지시 하나마다 새 대화로 1번 호출한다. 앞 지시의 대화를 이어 쓰지 않는다.
  - model=gpt-4o, temperature=0, 답은 JSON 객체 (response_format=json_object).
  - 점수는 첫 응답으로만 매긴다. 응답을 손으로 고치지 않는다 (앞뒤 ``` 표시만 자동으로 뗀다).
  - 본 시험에서 첫 응답이 실패한 지시는 검사 결과를 알려 주고 최대 2번 다시 생성해 '다시 생성' 칸에 따로 적는다(참고).
결과: out_v09/<시각>/ 에 지시마다 응답 원문(NN_tryK_raw.txt), 블록 JSON(NN_tryK.json), 그림(NN_tryK.png),
      표와 합계는 results.md (지시 하나가 끝날 때마다 다시 쓴다).
"""
import argparse
import datetime
import hashlib
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jenga_check as J  # noqa: E402

MODEL = "gpt-4o"
TEMPERATURE = 0
MAX_RETRY = 2
INSTRUCTIONS = [  # (지시, 구분)
    ("4단 책장", "템플릿"), ("2단 책장", "템플릿"), ("다리가 5층인 벤치", "템플릿"), ("넓은 벤치", "템플릿"),
    ("등받이가 높은 의자", "템플릿"), ("등받이 없는 의자", "템플릿"), ("선반이 넓은 책장", "템플릿"),
    ("다리가 낮은 벤치", "템플릿"), ("블록 20개로 벤치", "템플릿"), ("의자 2개", "템플릿"),
    ("2인용 벤치", "새 모양"), ("높은 의자", "새 모양"), ("작은 탁자", "새 모양"), ("계단 3칸", "새 모양"), ("침대", "새 모양"),
]
FEEDBACK = "검사기 결과: {line}\n규칙에 맞게 고친 설계 전체를 같은 JSON 형식으로 다시 답하라."


def ask(client, messages, model):
    import openai
    for k in range(3):
        try:
            r = client.chat.completions.create(model=model, temperature=TEMPERATURE,
                                               response_format={"type": "json_object"}, messages=messages)
            return r.choices[0].message.content or ""
        except (openai.AuthenticationError, openai.PermissionDeniedError, openai.NotFoundError,
                openai.BadRequestError) as e:
            raise SystemExit("API 오류 (다시 해도 같다): %s\n키와 모델 이름을 확인하고 PL에게 알린다." % e)
        except openai.APIError as e:      # 연결 끊김·시간 초과·서버 오류·사용량 제한: 잠깐 쉬고 다시
            print("  API 오류, %d초 뒤 다시 호출: %s" % (5 * (k + 1), e))
            time.sleep(5 * (k + 1))
    raise SystemExit("API 호출이 3번 실패했다. 네트워크를 확인하고 PL에게 알린다.")


def judge(reply, fallback):
    """응답 글자 -> (블록 목록 또는 None, 검사 결과, 대안일 때 고른 템플릿 설명)."""
    try:
        data = J.parse_text(reply)
        if fallback:
            if not isinstance(data, dict) or "template" not in data:
                raise ValueError('"template" 키가 없다')
            if data["template"] is None:
                return None, {"format_ok": False, "pass": False, "out_of_range": True,
                              "errors": ["범위 밖: %s" % data.get("reason", "")]}, "범위 밖"
            choice = "%s %s x%s" % (data["template"], json.dumps(data.get("params") or {}, ensure_ascii=False),
                                    data.get("count", 1))
            items = J.build_template(data["template"], data.get("params") or {}, data.get("count", 1))
        else:
            choice = ""
            items = J.items_from_data(data)
        res = J.evaluate_items(items)
        return (items if res["format_ok"] else None), res, choice
    except (ValueError, KeyError, TypeError, IndexError, AttributeError) as e:
        return None, {"format_ok": False, "pass": False, "errors": ["읽기 실패: %s" % e]}, "형식 실패"


def save(out, tag, reply, items, res, title):
    with open(os.path.join(out, tag + "_raw.txt"), "w", encoding="utf-8") as f:
        f.write(reply)
    if items is None:
        return
    with open(os.path.join(out, tag + ".json"), "w", encoding="utf-8") as f:
        f.write(J.to_codes(items) + "\n")
    try:
        J.draw_png(items, os.path.join(out, tag + ".png"), title)
    except ImportError:
        pass                          # matplotlib 이 없으면 그림만 건너뛴다


def cell_margin(res):
    if not res["format_ok"]:
        return "-"
    return "공중" if res["floating"] else "%.1f" % res["min_margin"]


def write_results(out, meta, rows, fallback):
    lines = ["# V-09 결과 (%s)" % ("대안: 템플릿 고르고 숫자만" if fallback else "본 시험"), "",
             "- 모델 %s, temperature %s, 시작 %s, 프롬프트 %s (sha256 앞 12자리 %s)" % (
                 meta["model"], meta["temperature"], meta["started"], meta["prompt"], meta["prompt_sha"]),
             "- 점수는 첫 응답으로만 매긴다. '지시와 맞음'은 그림(.png)을 보고 사람이 O/X 를 적는다 (기록만).", ""]
    if fallback:
        lines += ["| # | 지시 | GPT가 고른 것 | 형식 OK | 파고듦 없음 | 쌓는 중 최소 여유 (mm) | 통과 | 블록 수 | 지시와 맞음 |",
                  "|---|---|---|---|---|---|---|---|---|"]
    else:
        lines += ["| # | 지시 | 구분 | 형식 OK | 파고듦 없음 | 쌓는 중 최소 여유 (mm) | 통과 (첫 응답) | 블록 수 | 다시 생성 | 지시와 맞음 |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        res = r["res"]
        fmt = "O" if res["format_ok"] else "X"
        pen = "-" if not res["format_ok"] else ("O" if not res["penetrations"] else "X")
        nb = str(res.get("blocks", "-"))
        ok = "O" if res["pass"] else "X"
        if fallback:
            lines.append("| %d | %s | %s | %s | %s | %s | %s | %s | |" % (
                r["no"], r["text"], r["choice"], fmt, pen, cell_margin(res), ok, nb))
        else:
            lines.append("| %d | %s | %s | %s | %s | %s | %s | %s | %s | |" % (
                r["no"], r["text"], r["kind"], fmt, pen, cell_margin(res), ok, nb, r["retry"]))
    lines.append("")
    tmpl = [r for r in rows if r["kind"] == "템플릿"]
    new = [r for r in rows if r["kind"] == "새 모양"]
    if fallback:
        lines.append("- 대안 통과: %d/%d (범위 밖 %d개)" % (
            sum(r["res"]["pass"] for r in tmpl), len(tmpl), sum(bool(r["res"].get("out_of_range")) for r in tmpl)))
    else:
        lines.append("- 템플릿 변형, 첫 응답 통과: %d/%d (주제 판정 기준 10/10)" % (sum(r["res"]["pass"] for r in tmpl), len(tmpl)))
        lines.append("- 새 모양, 첫 응답 통과: %d/%d (기준 3/5, FR-P04 참고)" % (sum(r["res"]["pass"] for r in new), len(new)))
        lines.append("- 다시 생성까지 포함한 통과 (참고): 템플릿 %d/%d, 새 모양 %d/%d" % (
            sum(r["final_pass"] for r in tmpl), len(tmpl), sum(r["final_pass"] for r in new), len(new)))
    lines.append("- 실패 이유 (첫 응답): 범위 밖 %d, 형식 %d, 파고듦 %d, 공중에 뜬 블록 %d, 여유 7 mm 미만 %d" % (
        sum(bool(r["res"].get("out_of_range")) for r in rows),
        sum(not r["res"]["format_ok"] and not r["res"].get("out_of_range") for r in rows),
        sum(r["res"]["format_ok"] and bool(r["res"]["penetrations"]) for r in rows),
        sum(r["res"]["format_ok"] and r["res"]["floating"] for r in rows),
        sum(r["res"]["format_ok"] and not r["res"]["floating"] and r["res"]["min_margin"] < J.PASS_MM for r in rows)))
    with open(os.path.join(out, "results.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser(description="V-09 GPT-4o 설계 생성 시험")
    ap.add_argument("--ping", action="store_true", help="키와 모델만 확인")
    ap.add_argument("--fallback", action="store_true", help="대안: 템플릿 고르고 숫자만")
    ap.add_argument("--model", default=MODEL, help="기본 gpt-4o. 바꾸면 PL에게 알리고 기록한다")
    ap.add_argument("--prompt", default="", help="프롬프트 파일 (기본 prompt_v09.txt / 대안 prompt_v09_fallback.txt)")
    a = ap.parse_args()
    from openai import OpenAI
    client = OpenAI()               # OPENAI_API_KEY 환경 변수를 쓴다
    if a.ping:
        reply = ask(client, [{"role": "user", "content": 'JSON 으로 {"ok": true} 만 답하라.'}], a.model)
        print("응답: %s\n키와 모델(%s) 확인 끝" % (reply, a.model))
        return
    prompt_path = a.prompt or os.path.join(HERE, "prompt_v09_fallback.txt" if a.fallback else "prompt_v09.txt")
    with open(prompt_path, encoding="utf-8") as f:
        system = f.read()
    started = datetime.datetime.now()
    out = os.path.join(os.getcwd(), "out_v09", started.strftime("%m%d_%H%M%S") + ("_fallback" if a.fallback else ""))
    os.makedirs(out)
    meta = {"model": a.model, "temperature": TEMPERATURE, "started": started.strftime("%Y-%m-%d %H:%M:%S"),
            "prompt": os.path.basename(prompt_path), "prompt_sha": hashlib.sha256(system.encode("utf-8")).hexdigest()[:12]}
    with open(os.path.join(out, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    with open(os.path.join(out, "prompt_used.txt"), "w", encoding="utf-8") as f:
        f.write(system)
    todo = [(i, t, k) for i, (t, k) in enumerate(INSTRUCTIONS, 1) if not (a.fallback and k != "템플릿")]
    rows = []
    for no, text, kind in todo:
        messages = [{"role": "system", "content": system}, {"role": "user", "content": "지시: " + text}]
        reply = ask(client, messages, a.model)
        items, res, choice = judge(reply, a.fallback)
        tag = "%02d_try1" % no
        save(out, tag, reply, items, res, "#%d try1 %s" % (no, "PASS" if res["pass"] else "FAIL"))
        line = ("범위 밖 (%s) | 실패" % res["errors"][0][len("범위 밖: "):]) if res.get("out_of_range") else J.summary(res)
        print("%2d %s [%s] %s" % (no, text, kind, (choice + " | ") if choice else "") + line)
        row = {"no": no, "text": text, "kind": kind, "res": res, "choice": choice, "retry": "-", "final_pass": res["pass"]}
        if not a.fallback and not res["pass"]:
            for t in range(2, MAX_RETRY + 2):
                messages += [{"role": "assistant", "content": reply}, {"role": "user", "content": FEEDBACK.format(line=line)}]
                reply = ask(client, messages, a.model)
                items, res2, _ = judge(reply, False)
                tag = "%02d_try%d" % (no, t)
                save(out, tag, reply, items, res2, "#%d try%d %s" % (no, t, "PASS" if res2["pass"] else "FAIL"))
                line = J.summary(res2)
                print("     다시 생성 %d: %s" % (t - 1, line))
                if res2["pass"]:
                    row["retry"], row["final_pass"] = "%d번째에 통과" % (t - 1), True
                    break
            else:
                row["retry"] = "%d번 모두 실패" % MAX_RETRY
        rows.append(row)
        write_results(out, meta, rows, a.fallback)
    print("\n" + open(os.path.join(out, "results.md"), encoding="utf-8").read())
    print("저장한 곳:", out)


if __name__ == "__main__":
    main()
