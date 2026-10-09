# -*- coding: utf-8 -*-
"""/ws — 로봇 상태 · 연결 · 음성 의도를 화면에 밀어 주는 WebSocket (E-41, web/README 2 · 5장, W126).

메시지는 모두 {"type", "data"} 한 겹: state · progress · scan_result · safety · gripper · bridge_alive · broker · intent · wrist_image ·
timing(붙을 때 한 번 — 요청 시간 제한) (+ 생성 gen, W112).
화면이 붙으면 들고 있는 마지막 값을 먼저 다 보내고(MQTT retained 자리), 그 뒤 바뀔 때마다 보낸다.
"""
import asyncio

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()
QUEUE_MAX = 200     # 화면 하나가 밀려 있을 때 쌓아 두는 이벤트 수. 넘치면 가장 오래된 것부터 버린다(상태는 마지막 값이 중요)


class WsHub:
    """화면 연결들에 이벤트를 나눠 준다. publish 는 어느 스레드에서 불러도 된다(paho · 연결 신호 스레드 → asyncio 루프)."""

    def __init__(self):
        """루프는 앱이 시작할 때 bind 로 넣는다."""
        self.loop = None
        self.queues = set()

    def bind(self, loop):
        """이벤트를 넘길 asyncio 루프(uvicorn 루프)를 정한다."""
        self.loop = loop

    def publish(self, event):
        """이벤트 하나를 모든 화면 큐에 넣는다(스레드 안전). 루프가 없으면(시작 전) 버린다 — 화면은 붙을 때 처음 값을 받는다."""
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self._fan_out, event)

    def _fan_out(self, event):
        """(루프 안) 큐마다 넣는다. 꽉 찬 큐는 가장 오래된 이벤트를 하나 버리고 넣는다."""
        for q in list(self.queues):
            if q.full():
                q.get_nowait()
            q.put_nowait(event)


async def _pump(websocket, q):
    """큐의 이벤트를 차례로 화면에 보낸다. 받기 쪽(ws)이 끊김을 알면 취소된다."""
    while True:
        await websocket.send_json(await q.get())


@router.websocket('/ws')
async def ws(websocket: WebSocket):
    """화면 하나: 처음 값을 type 별로 보낸 뒤 이벤트를 보낸다. 화면은 보내는 것이 없지만, 받기를 기다려야 끊김을 바로 안다."""
    await websocket.accept()
    hub = websocket.app.state.hub
    q = asyncio.Queue(maxsize=QUEUE_MAX)
    hub.queues.add(q)
    pump = None
    try:
        for kind, data in websocket.app.state.mqtt.snapshot().items():
            if kind == 'bridge_alive':
                data = {'alive': data}
            elif kind == 'broker':
                data = {'connected': data}
            await websocket.send_json({'type': kind, 'data': data})
        pump = asyncio.create_task(_pump(websocket, q))
        while True:
            await websocket.receive_text()     # 끊기면 WebSocketDisconnect — 그때 보내기도 멈춘다
    except WebSocketDisconnect:
        pass
    finally:
        hub.queues.discard(q)
        if pump:
            pump.cancel()
