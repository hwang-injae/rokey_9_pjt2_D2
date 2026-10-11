# -*- coding: utf-8 -*-
"""/api/robot — 화면 버튼(설계 선택 · 출발 · [계속] · 스캔 · 취소 · 정지 · 다시 시작) · 지금 상태 · 손목 검출 그림 · 스캔 사진 · 점군 (IRD 10.1 · 10.5, web/README 2장, W126 · W127).

버튼 하나 = MQTT 요청 하나(MqttClient.request). 응답은 다리가 넘긴 ROS 응답 칸 그대로(success · reason 또는 message) — 판단은 로봇 쪽이 한다.
자동 재전송 없음: 시간 초과 · BUSY · 빈 reason 거절 모두 그대로 돌려주고 사람이 다시 누른다. 버튼이 '보내는 중'에 머물지 않게 늘 답한다.
"""
from typing import Literal

from fastapi import APIRouter, HTTPException, Request, Response
from pydantic import BaseModel

router = APIRouter(prefix='/api/robot')
NEEDS_BRIDGE = ('start', 'scan')   # 로봇 PC 가 끊겼으면 막는 명령(IRD 10.3). 정지는 막지 않는다 — 닿지 않으면 키 · 펜던트


class Command(BaseModel):
    """화면 명령(IRD 2장 cmd). start 는 WAIT_SUPPLY · WAIT_HMI 에서 [계속]과 같다. mode 는 auto 고정(칸만 둠)."""
    cmd: Literal['select_design', 'start', 'scan', 'cancel']
    design_id: str = ''
    mode: str = 'auto'


@router.post('/command')
def command(body: Command, request: Request):
    """/d2/hmi/command 요청 → {success, reason}. 로봇 PC 가 끊겼으면 출발 · 스캔은 보내지 않고 success false + 안내."""
    mqtt = request.app.state.mqtt
    if body.cmd in NEEDS_BRIDGE and not mqtt.bridge_connected():
        return {'success': False, 'reason': '', 'message': '로봇 PC 연결 끊김 — 출발 · 스캔을 막았어요'}
    return mqtt.request('/d2/hmi/command', body.model_dump())


@router.post('/stop')
def stop(request: Request):
    """/d2/safety/stop — source web, reason 비움(정지 노드가 STOP_WEB 으로 적음, 10/9 PL). 다리가 끊겨 있어도 보내 본다."""
    return request.app.state.mqtt.request('/d2/safety/stop', {'source': 'web', 'reason': ''})


@router.post('/resume')
def resume(request: Request):
    """/d2/safety/resume — 잠금 풀기 한 번(S-24). 풀리면 작업 관리자가 관측부터 다시."""
    return request.app.state.mqtt.request('/d2/safety/resume', {})


@router.get('/state')
def state(request: Request):
    """지금 값 전부(state · progress · scan_result · safety · gripper + bridge_alive · broker). 화면이 처음 붙을 때."""
    return request.app.state.mqtt.snapshot()


def _blob(request, kind, media_type, what):
    """MqttClient 가 들고 있는 마지막 바이트를 그대로 돌려준다. 없으면 404. 브라우저가 옛 것을 쓰지 않게 캐시 끔."""
    data = request.app.state.mqtt.blob(kind)
    if data is None:
        raise HTTPException(404, f'{what}이 아직 없다')
    return Response(data, media_type=media_type, headers={'Cache-Control': 'no-store'})


@router.get('/wrist.jpg')
def wrist(request: Request):
    """마지막 손목 검출 그림(JPEG — 손목 비전이 그려 보낸 그대로, E-67)."""
    return _blob(request, 'wrist_image', 'image/jpeg', '손목 검출 그림')


@router.get('/scan.jpg')
def scan_image(request: Request):
    """마지막 스캔 사진(JPEG ≤ 500 KB — 다리가 image_path 파일을 읽어 보낸 것, IRD 10.5). 새 scan_result 가 오면 지워진다."""
    return _blob(request, 'scan_image', 'image/jpeg', '스캔 사진')


@router.get('/scan_cloud.ply')
def scan_cloud(request: Request):
    """마지막 스캔 점군(PLY, base_link m — 화면 점군 창 three.js PLYLoader 가 읽음, IRD 10.5). cloud_path 가 없던 스캔이면 404."""
    return _blob(request, 'scan_cloud', 'application/octet-stream', '스캔 점군')
