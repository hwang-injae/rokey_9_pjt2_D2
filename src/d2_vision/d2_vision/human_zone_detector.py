"""웹캠 사람 구역 판정 (HumanZoneDetector) — ROS·MediaPipe·OpenCV 없이 도는 계산 부분.

webcam_human 노드가 쓴다. 화면 픽셀 점들을 책상 좌표(mm, base_link)로 바꾸고,
robot.yaml 의 webcam_zones 사각형 안에 들어오는지로 구역별 사람 있음/없음을 정한다.
웹캠이 없어도 시험할 수 있게 노드와 파일을 나눴다.

보정 파일은 docs/research/ref_1003/작업4/webcam_calib.py 가 만든 webcam_H.json 형식을 그대로 읽는다
(H = 화면 픽셀 → 책상 좌표 3x3, width·height = 보정 때 해상도).
"""
import json
import os

import yaml

ZONES = ('assembly', 'robot_supply', 'path', 'human_supply')
ZONE_KEYS = ('x_min_mm', 'x_max_mm', 'y_min_mm', 'y_max_mm')


class ConfigError(Exception):
    """설정·보정 파일이 없거나 값이 비었을 때. 메시지는 그대로 ERROR 로그에 쓴다."""


class HumanZoneDetector:
    """구역 사각형(mm)과 보정 행렬 H 로 '이 점들이 어느 구역 안에 있나'를 답한다.

    입력: 화면 픽셀 점 (u, v) 목록. 출력: {구역 이름: bool} (참 = 사람 있음).
    바깥 영향 없음(파일 읽기만). 설정이 틀리면 만들 때 ConfigError.
    """

    def __init__(self, zones, H, width, height):
        """구역(mm)·보정 행렬·보정 해상도를 받아 둔다. 보통은 from_files 로 만든다."""
        self.zones = zones            # {구역: (x_min, x_max, y_min, y_max)} mm
        self.H = H                    # 3x3 리스트, 픽셀 → mm
        self.width = width            # 보정 때 해상도 (화면 크기가 다르면 좌표가 틀린다)
        self.height = height

    @classmethod
    def from_files(cls, config_path, calib_path):
        """robot.yaml 의 webcam_zones 와 보정 json 을 읽어 만든다. 없거나 비었으면 ConfigError."""
        return cls(cls._load_zones(config_path), *cls._load_calib(calib_path))

    @staticmethod
    def _load_zones(path):
        """robot.yaml 맨 위의 webcam_zones 를 읽는다. 값이 하나라도 비었거나 min >= max 면 ConfigError.

        임의 좌표로 채우지 않는다 — 좌표는 W055 보정 뒤 로봇 파트 robot.yaml 에 들어온다.
        """
        if not path or not os.path.isfile(path):
            raise ConfigError("설정 파일이 없다: '%s' (config_file 파라미터에 robot.yaml 경로를 준다)" % path)
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        raw = data.get('webcam_zones') if isinstance(data, dict) else None
        if not isinstance(raw, dict):
            raise ConfigError("%s 에 webcam_zones 가 없다 (W055 보정 뒤 좌표를 넣는다)" % path)
        zones = {}
        for name in ZONES:
            box = raw.get(name)
            if not isinstance(box, dict):
                raise ConfigError("webcam_zones.%s 가 없다" % name)
            vals = []
            for key in ZONE_KEYS:
                v = box.get(key)
                if isinstance(v, bool) or not isinstance(v, (int, float)):
                    raise ConfigError("webcam_zones.%s.%s 값이 비었거나 숫자가 아니다: %r" % (name, key, v))
                vals.append(float(v))
            if vals[0] >= vals[1] or vals[2] >= vals[3]:
                raise ConfigError("webcam_zones.%s 의 min 이 max 보다 작아야 한다: %s" % (name, vals))
            zones[name] = tuple(vals)
        return zones

    @staticmethod
    def _load_calib(path):
        """webcam_H.json 에서 (H, width, height) 를 읽는다. 없거나 형식이 다르면 ConfigError."""
        if not path or not os.path.isfile(path):
            raise ConfigError("웹캠 보정 파일이 없다: '%s' (calib_file 파라미터, webcam_calib.py calib 로 만든다)" % path)
        try:
            with open(path, encoding='utf-8') as f:
                d = json.load(f)
            flat = [float(v) for row in d['H'] for v in (row if isinstance(row, list) else [row])]
            if len(flat) != 9:
                raise ValueError('H 가 3x3 이 아니다')
            H = [flat[0:3], flat[3:6], flat[6:9]]
            return H, int(d['width']), int(d['height'])
        except (ValueError, KeyError, TypeError) as e:
            raise ConfigError("웹캠 보정 파일 %s 를 읽을 수 없다 (%s). calib 를 다시 한다" % (path, e))

    def px_to_mm(self, u, v):
        """화면 픽셀 (u, v) → 책상 좌표 (x, y) mm. 책상 면 위의 점이라고 보고 계산한다.

        떠 있는 손목은 먼 쪽으로 밀려 보인다 — 여기서는 보정하지 않는다(W055 뒤 구역 경계·margin 으로).
        H 의 나누는 값이 0 에 가까우면 None.
        """
        H = self.H
        x = H[0][0] * u + H[0][1] * v + H[0][2]
        y = H[1][0] * u + H[1][1] * v + H[1][2]
        w = H[2][0] * u + H[2][1] * v + H[2][2]
        if abs(w) < 1e-9:
            return None
        return x / w, y / w

    def occupied(self, points_px):
        """픽셀 점 목록 → {구역: 사람 있음}. 점이 하나도 없으면 모두 False. 경계는 안쪽으로 친다."""
        result = {name: False for name in ZONES}
        for u, v in points_px:
            mm = self.px_to_mm(u, v)
            if mm is None:
                continue
            x, y = mm
            for name, (x0, x1, y0, y1) in self.zones.items():
                if x0 <= x <= x1 and y0 <= y <= y1:
                    result[name] = True
        return result
