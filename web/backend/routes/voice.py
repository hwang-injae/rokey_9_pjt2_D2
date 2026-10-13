# -*- coding: utf-8 -*-
"""/api/voice — 화면 마이크 버튼(10/13 PL 결정 5 · 6).

버튼 → backend → MQTT d2/hmi/voice/listen(웹 PC 안 — 다리 안 거침) → voice.py 녹음 · Whisper → d2/hmi/intent {intent: request_design, text}
→ backend → /ws {type: intent} → 화면이 글 창에 넣고 고른 설계로 바로 생성. 웨이크워드 없음. 음성은 요청 글 넣기만 —
음성으로 설계 고르기 · 출발은 화면 흐름에서 쓰지 않는다.
"""
from fastapi import APIRouter, Request

router = APIRouter(prefix='/api/voice')


@router.post('/listen')
def listen(request: Request):
    """voice.py 에 한 번 듣기를 알린다 → {success, reason[, message]}. 브로커가 없으면 success false(화면이 그대로 알림).
    voice.py 가 꺼져 있으면 답(intent)이 안 온다 — 화면이 기다림 시간을 넘기면 안내한다."""
    return request.app.state.mqtt.listen()
