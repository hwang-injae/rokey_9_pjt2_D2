"""V-18 소음 속 음성 명령 인식률. 강의 STT 와 같은 설정: OpenAI whisper-1, 16 kHz, 모노, 16-bit, 5초 녹음.

필요: sudo apt install libportaudio2
      pip install openai sounddevice numpy      (mediapipe 를 깔았으면 sounddevice·numpy 는 이미 있다)
      export OPENAI_API_KEY=...                  (키는 터미널에서만. AI 에이전트 채팅에 붙여 넣지 않는다)
실행: python3 stt_test.py --list                              # 마이크 번호 보기
      python3 stt_test.py --dist 1m --mic 3 --noise 로봇소음    # 명령 20개
      python3 stt_test.py --dist 1m --mic 3 --noise 로봇소음 --stop 10   # '정지'만 10번
진행: 화면에 나온 문장을, Enter 를 누른 뒤 5초 안에 말한다.
판정: 띄어쓰기·문장부호를 뺀 글자가 문장과 똑같으면 맞음. 인식 원문은 stt_results.csv 에 남는다.
"""
import argparse
import csv
import datetime
import time
import io
import os
import wave

SECONDS, RATE = 5, 16000
COMMANDS = [
    "의자 만들어 줘", "벤치 만들어 줘", "책장 만들어 줘", "정지", "멈춰",
    "계속해", "다시 해", "취소해", "지금 몇 단계야", "그 블록은 내가 놓을게",
    "다 됐어", "천천히 해", "처음부터 다시 해", "블록이 몇 개 남았어", "잠깐 기다려",
    "시작해", "등받이를 더 높여 줘", "사진 찍어 줘", "왼쪽 다리부터 쌓아 줘", "오늘은 여기까지 하자",
]


def norm(s):
    """글자와 숫자만 남긴다 (띄어쓰기·문장부호 제거)."""
    return "".join(ch for ch in s if ch.isalnum()).lower()


def record(device):
    import sounddevice as sd
    audio = sd.rec(int(SECONDS * RATE), samplerate=RATE, channels=1, dtype="int16", device=device)
    sd.wait()
    return audio


def to_wav(audio):
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(audio.tobytes())
    return buf.getvalue()


def transcribe(client, wav_bytes):
    r = client.audio.transcriptions.create(model="whisper-1", file=("cmd.wav", wav_bytes, "audio/wav"))
    return r.text


def main():
    ap = argparse.ArgumentParser(description="V-18 음성 명령 인식률")
    ap.add_argument("--list", action="store_true", help="마이크 목록만 보기")
    ap.add_argument("--mic", default=None, help="마이크 번호나 이름 (--list 로 확인). 없으면 기본 마이크")
    ap.add_argument("--dist", default="", help="마이크와의 거리, 예: 1m")
    ap.add_argument("--noise", default="", help="소음 조건 기록용, 예: 로봇소음 / 없음")
    ap.add_argument("--stop", type=int, default=0, help="명령 20개 대신 '정지'를 이 횟수만큼")
    a = ap.parse_args()
    if a.list:
        import sounddevice as sd
        print(sd.query_devices())
        return
    device = int(a.mic) if a.mic is not None and a.mic.isdigit() else a.mic
    from openai import OpenAI
    client = OpenAI()          # OPENAI_API_KEY 환경 변수를 쓴다
    phrases = ["정지"] * a.stop if a.stop else COMMANDS
    log = os.path.join(os.getcwd(), "stt_results.csv")
    new = not os.path.exists(log)
    ok_n = 0
    lat = []                   # 녹음이 끝난 뒤 글자가 돌아오기까지 걸린 시간 (ms)
    with open(log, "a", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        if new:
            wr.writerow(["시각", "거리", "소음", "마이크", "문장", "인식 원문", "맞음", "STT 시간(ms)"])
        for k, p in enumerate(phrases, 1):
            input("[%d/%d] Enter 를 누르고 5초 안에 말한다:  %s " % (k, len(phrases), p))
            wav = to_wav(record(device))
            t0 = time.perf_counter()
            text = transcribe(client, wav)
            ms = (time.perf_counter() - t0) * 1000.0
            lat.append(ms)
            ok = norm(text) == norm(p)
            ok_n += ok
            print("    인식: %s  -> %s  (%.0f ms)" % (text, "맞음" if ok else "틀림", ms))
            wr.writerow([datetime.datetime.now().strftime("%H:%M:%S"), a.dist, a.noise, a.mic, p, text, int(ok), round(ms)])
            f.flush()
    print("결과 (%s, %s): %d/%d = %.0f %%  (stt_results.csv 에 저장)" % (
        a.dist or "-", a.noise or "-", ok_n, len(phrases), 100.0 * ok_n / len(phrases)))
    if lat:
        print("STT 시간: 평균 %.0f ms, 최대 %.0f ms (녹음 5초 뒤 글자가 오기까지. 네트워크 포함)" % (sum(lat) / len(lat), max(lat)))


if __name__ == "__main__":
    main()
