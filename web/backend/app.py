# -*- coding: utf-8 -*-
"""WebApp — 웹 backend 시작점 (SDD 3.1.1, W126). FastAPI :8000 한 프로세스, ROS 없음(E-32 · E-41).

하는 일: 앱 만들기 · routes 등록 · frontend/out 정적 서빙 · robot.yaml 규칙 읽기(한 곳, 읽기 전용 — 10/9 PL E-78) · MqttClient 시작.
실행(웹 PC 호스트, compose 전): cd web/backend && MQTT_HOST=localhost uvicorn app:app --host 0.0.0.0 --port 8000
설정: 환경 변수 MQTT_HOST · MQTT_PORT(.env), ROBOT_YAML(컨테이너에서 robot.yaml 을 읽기 전용으로 연결한 경로, 비우면 저장소 파일).
OpenAI 키는 있는지만 로그에 찍고 값은 절대 찍지 않는다(팀 규칙 4).
"""
import asyncio
import logging
import os
import sys
from contextlib import asynccontextmanager
from pathlib import Path

import yaml
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

REPO = Path(__file__).resolve().parents[2]
for _pkg in ('d2_bridge', 'd2_task'):          # 웹 PC 에 ROS 는 없고, 이 두 패키지의 ROS 없는 파일만 같은 저장소에서 가져다 쓴다(E-59 · W126)
    _path = str(REPO / 'src' / _pkg)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from mqtt_client import MqttClient   # noqa: E402 — 위에서 d2_bridge 경로를 넣은 뒤
from routes import robot, ws         # noqa: E402

ROBOT_YAML = Path(os.environ.get('ROBOT_YAML') or REPO / 'src' / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml')
FRONTEND_OUT = REPO / 'web' / 'frontend' / 'out'
log = logging.getLogger('web')


def load_rules(path=ROBOT_YAML):
    """robot.yaml 에서 웹이 쓰는 키만 읽는다(10/9 PL E-78 — 측정값 · 자세 · 원점은 안 읽음).

    출력: {block_size_m, margin_mm, max_blocks, finger_thickness_m, finger_width_m, assembly_area_half_m,
           req_timeout_s, alive_s, lost_after_s}. 실패: 파일 · 키가 없으면 예외 — 다리 · 검사 묶음과 다른 값으로 돌지 않게 켜지 않는다.
    """
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    return {
        'block_size_m': cfg['block_size_m'],
        'margin_mm': cfg['check']['margin_mm'],
        'max_blocks': cfg['check']['max_blocks'],
        'finger_thickness_m': cfg['finger']['thickness_m'],
        'finger_width_m': cfg['finger']['width_m'],
        'assembly_area_half_m': cfg['assembly_area_half_m'],
        'req_timeout_s': cfg['mqtt']['req_timeout_s'],   # 다리와 같은 MQTT 시간 값(IRD 9장) — 두 PC 가 같은 숫자를 쓰게
        'alive_s': cfg['mqtt']['alive_s'],
        'lost_after_s': cfg['mqtt']['lost_after_s'],
    }


def create_app(mqtt=None, rules=None):
    """FastAPI 앱을 만든다. mqtt · rules 는 시험 때 가짜를 넣고, 비우면 환경 변수 · robot.yaml 로 만든다."""
    rules = rules or load_rules()
    mqtt = mqtt or MqttClient(os.environ.get('MQTT_HOST', 'localhost'), os.environ.get('MQTT_PORT', '1883'),
                              rules['req_timeout_s'], rules['alive_s'], rules['lost_after_s'])
    hub = ws.WsHub()
    mqtt.on_event = hub.publish

    @asynccontextmanager
    async def lifespan(_app):
        """시작: 허브에 루프를 넣고 브로커에 붙는다 · 시작 점검을 찍는다. 끝: 연결 신호 false 를 남기고 나간다."""
        hub.bind(asyncio.get_running_loop())
        mqtt.start()
        log.info('[웹] 브로커 %s:%s · OpenAI 키 %s · 화면 %s', mqtt.host, mqtt.port,
                 '있음' if os.environ.get('OPENAI_API_KEY') else '없음(생성 · 음성 안 됨)',
                 '있음' if FRONTEND_OUT.is_dir() else '없음(frontend 빌드 전 — REST · /ws 만)')
        yield
        mqtt.stop()

    app = FastAPI(title='D2 web backend', lifespan=lifespan)
    app.state.mqtt, app.state.hub, app.state.rules = mqtt, hub, rules
    app.include_router(robot.router)
    app.include_router(ws.router)
    if FRONTEND_OUT.is_dir():
        app.mount('/', StaticFiles(directory=FRONTEND_OUT, html=True), name='frontend')   # 마지막에 — /api · /ws 보다 뒤
    return app


logging.basicConfig(level=logging.INFO, format='[%(name)s] %(message)s')
app = create_app()
