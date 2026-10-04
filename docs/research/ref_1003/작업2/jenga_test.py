#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""작업 2 (10/3) 로봇 시험 스크립트: 그리퍼 확인, R-0 확인, 교시, V-16, V-20, V-11.

준비
  - 터미널 1: 브링업을 띄워 둔다. 하루 종일 끄지 않는다.
      실기: ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100
      가상: ros2 launch m0609_rg2_bringup bringup.launch.py
  - 터미널 2: source <내 워크스페이스>/install/setup.bash 뒤 아래처럼 실행한다.
  - 강의 onrobot.py(RG 클래스)를 쓴다. robot_control 패키지를 못 찾으면 onrobot.py를 이 파일 옆에 복사한다.
  - 아래 '현장에서 채울 값'을 먼저 채운다. 모르는 값은 비워 두고 PL에게 묻는다.

실행 (폭은 mm, 힘은 N으로 쓴다)
  python3 jenga_test.py gripper 35          그리퍼만 폭 35 mm로 (브링업 없어도 된다)
  python3 jenga_test.py gripper 25 20       그리퍼만 폭 25 mm, 힘 20 N
  python3 jenga_test.py check               R-0: 툴·TCP 설정, TCP 좌표·Fz·그리퍼 폭 출력
  python3 jenga_test.py pose                교시: 수동 모드로 바꾸고 좌표 출력 (팔은 손으로 끌어 옮긴다)
  python3 jenga_test.py v16                 V-16: 폼 블록 쪽으로 천천히 갔다가 되돌아오기 3번
  python3 jenga_test.py v20 side 20 fast 5  V-20: 잡기(side|end) 힘(N) 속도(slow|fast) 횟수
  python3 jenga_test.py v11 A side 20 10    V-11: 방법(A|B) 잡기(side|end) 힘(N) 횟수 [높이 mm]
                                              A의 높이 = 목표 위 몇 mm에서 열지 (기본 2)
                                              B의 높이 = 목표 아래 몇 mm까지 순응 하강할지 (기본 1)
  python3 jenga_test.py selftest            가상 모드 전용: 모든 시험 동작을 한 번씩 자동으로 돌려 본다

  side = 옆면 잡기(25 mm 방향), end = 끝면 잡기(75 mm 방향)
  시험 기록은 이 파일 옆 jenga_log.csv에 한 줄씩 쌓인다 (selftest는 jenga_selftest_log.csv).

안전
  - 로봇이 움직이기 전마다 '조작하는 사람이 펜던트를 들고 비상정지를 바로 누를 수 있고, 작업 영역에 손이 없으면 Enter'를 묻는다.
  - 멈출 때는 비상정지. Ctrl+C는 스크립트만 끈다. 이미 보낸 로봇 동작은 끝까지 간다.
  - 순응 제어(V-11 방법 B) 중에 끝나도 끝날 때 순응 제어를 끈다. '순응 제어 끄기 실패'가 나오면 펜던트에서 순응 제어를 끈다.
  - 툴·TCP 설정이 실패하거나 그리퍼가 연결되지 않으면 로봇을 움직이지 않고 멈춘다.
"""
import contextlib
import csv
import io
import os
import sys
import time

# ===== 현장에서 채울 값 (모르면 비워 두고 PL에게 묻는다) =====================
TOOL_NAME = ""        # R-0에서 펜던트에 등록한 툴(무게) 이름. 글자 하나까지 같게
TCP_NAME = ""         # R-0에서 펜던트에 등록한 TCP 이름
GAP_OFFSET = 0.0      # R-0에서 잰 값 (mm): 'gripper 35' 뒤 35 − (손가락 끝 패드 사이 실측 간격)
# 교시한 자세 [x, y, z, rx, ry, rz] (mm, 도). pose 모드에서 출력된 목록을 그대로 붙여 넣는다
PICK_SIDE = None      # 옆면 잡기(25 mm 방향)로 집는 자세 (블록이 집는 자리에 있을 때)
PICK_END = None       # 끝면 잡기(75 mm 방향)로 집는 자세
PLACE_SIDE = None     # V-11: 옆면 잡기로 아래 블록 위에 놓는 자세
PLACE_END = None      # V-11: 끝면 잡기로 아래 블록 위에 놓는 자세
SIDE_MOVE = [0.0, 200.0]   # V-20: 옆으로 옮길 거리 [베이스 x, y] (mm). 책상에서 비어 있는 쪽
V16_MOVE = [0.0, 150.0]    # V-16: 폼 블록 쪽으로 갈 거리 [베이스 x, y] (mm)
# =============================================================================

ROBOT_ID, ROBOT_MODEL = "dsr01", "m0609"
GRIPPER_IP, GRIPPER_PORT = "192.168.1.1", "502"   # 강의 코드와 같은 값 (포트는 문자열)

# 잡기 이름(팀 공통). 닫을 폭은 RG2 코드 값(0.1 mm 단위, 200 = 20.0 mm).
# RG2 폭 값은 알루미늄 손가락 안쪽 사이라서, 패드 사이 실제 간격은 GAP_OFFSET만큼 좁을 수 있다.
# 그래서 열 때는 '패드 간격 = 블록 + 10 mm'가 되게 GAP_OFFSET을 더한다 (열 때 손가락 끝이 덜 올라간다).
GRASP = {
    "side": {"name": "옆면 잡기(25 mm 방향)", "block": 25.0, "close": 200},
    "end": {"name": "끝면 잡기(75 mm 방향)", "block": 75.0, "close": 700},
}
SPEED = {"slow": (50.0, 100.0), "fast": (200.0, 500.0)}   # 느림·빠름 (mm/s, mm/s^2)
MOVE = (50.0, 100.0)      # 시험 밖 이동 (최고 속도 1 m/s의 5 %)
DESCENT = (20.0, 50.0)    # 잡는 자리로 마지막 수직 하강
APPROACH = 50.0           # 잡는 자리·놓는 자리 위 높이 (mm)
OPEN_FORCE = 200          # 열 때 힘 20 N (0.1 N 단위)
PLACE_OPEN_FORCE = 30     # V-11에서 놓을 때 여는 힘 3 N (RG2 최소). 힘이 작을수록 천천히 열린다
FZ_LIMIT = 2.0            # 아무것도 안 닿았을 때 Fz 허용 범위 (N)
HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "jenga_log.csv")
FIELDS = ["time", "test", "grasp", "force_N", "speed", "method", "trial", "grip",
          "width_mm", "grip_after", "width_after_mm", "slip", "shift_mm", "lower_moved", "note"]

AUTO = False              # selftest에서만 True (입력을 기본값으로 자동 답한다)
REAL = True               # 시작할 때 컨트롤러에 물어서 정한다 (모르면 실기로 본다)
G = None                  # 그리퍼
dr = None                 # DSR_ROBOT2 모듈
posx = None
NODE = None
COMPLIANCE_ON = False


def open_code(gp):
    """패드 간격이 블록 + 10 mm가 되는 RG2 코드 값."""
    return min(1100, int(round((gp["block"] + 10.0 + GAP_OFFSET) * 10)))


# ----------------------------------------------------------------- 입력·기록
def ask(prompt, auto=""):
    if AUTO:
        print(prompt + f"{auto}  (자동)")
        return auto
    try:
        return input(prompt)
    except EOFError:
        raise SystemExit("[멈춤] 입력이 끊겼다.")


def yes(prompt, auto="y"):
    return ask(prompt + " (y/n) ", auto).strip().lower().startswith("y")


def ask_float(prompt, auto="0"):
    while True:
        s = ask(prompt, auto).strip()
        try:
            return float(s)
        except ValueError:
            print("  숫자로 넣는다 (예: 0.4)")


def ready(msg):
    ask(f"\n[{msg}]\n  펜던트를 들고 비상정지를 바로 누를 수 있고, 작업 영역에 손이 없으면 Enter (그만: Ctrl+C) ")


def log(**kw):
    row = {k: "" for k in FIELDS}
    row["time"] = time.strftime("%H:%M:%S")
    row.update(kw)
    new = not os.path.exists(LOG)
    with open(LOG, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)


# ----------------------------------------------------------------- 그리퍼
def load_rg():
    errors = []
    try:
        from robot_control.onrobot import RG
        return RG
    except ImportError as e:
        errors.append(f"robot_control.onrobot: {e}")
    sys.path.insert(0, HERE)
    try:
        from onrobot import RG
        return RG
    except ImportError as e:
        errors.append(f"onrobot: {e}")
    sys.exit("[멈춤] RG 클래스를 못 불러왔다.\n  " + "\n  ".join(errors) +
             "\n  pymodbus가 없다는 오류면 pymodbus 3.6.x를 깐다."
             "\n  onrobot을 못 찾으면 강의 cobot2의 robot_control/onrobot.py를 이 파일 옆에 복사한다.")


def connect_gripper():
    """rclpy를 켜기 전에 Modbus로 직접 붙는다. 실패하면 (None, 이유).

    rclpy보다 먼저 붙는 이유: 강의 onrobot.py는 Modbus 연결에 실패하면 브링업의 그리퍼 서비스로
    몰래 넘어가는데, 그 길로는 힘 값이 무시된다 (V-20 힘 시험이 틀어진다).
    """
    RG = load_rg()
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            g = RG("rg2", GRIPPER_IP, GRIPPER_PORT)
        return g, g.get_width()
    except TypeError as e:
        return None, f"pymodbus 버전이 맞지 않는 것 같다 (3.6.x 필요): {e}"
    except Exception as e:
        return None, str(e)


class FakeGripper:
    """가상 모드용. 실제 그리퍼는 움직이지 않는다."""

    def __init__(self):
        self.w = 110.0

    def move_gripper(self, width_val, force_val=400):
        self.w = width_val / 10.0
        print(f"  (가상 그리퍼) 폭 {self.w:.1f} mm, 힘 {force_val / 10:.1f} N", file=sys.__stdout__)

    def get_width(self):
        return self.w

    def get_status(self):
        return [0, 1, 0, 0, 0, 0, 0]


def gstatus():
    with contextlib.redirect_stdout(io.StringIO()):   # onrobot.py의 상태 문구는 숨긴다
        return G.get_status()


def gmove(width, force, timeout=5.0):
    """폭·힘(0.1 단위)으로 움직이고 멈출 때까지 기다린다. (잡힘 감지, 폭 mm)를 돌려준다."""
    with contextlib.redirect_stdout(io.StringIO()):
        G.move_gripper(int(width), int(force))
    time.sleep(0.3)
    t0 = time.time()
    while gstatus()[0]:                 # 0번 = 동작 중
        if time.time() - t0 > timeout:
            raise RuntimeError("그리퍼가 5초 안에 멈추지 않았다")
        time.sleep(0.1)
    return bool(gstatus()[1]), float(G.get_width())   # 1번 = 잡힘 감지


def force_code(force_n):
    f = float(force_n)
    if not 3.0 <= f <= 40.0:
        sys.exit("[멈춤] 힘은 3~40 N으로 쓴다 (RG2 범위).")
    return int(round(f * 10))


# ----------------------------------------------------------------- 로봇
def start_ros():
    global dr, posx, NODE, REAL
    import rclpy
    import DR_init
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    # Ctrl+C가 ROS를 먼저 끄지 않게 한다. 그래야 끝날 때(finally) 순응 제어 끄기가 실제로 로봇에 간다.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    NODE = rclpy.create_node("jenga_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = NODE        # 이 줄 다음에 DSR_ROBOT2를 import 한다 (순서를 바꾸면 오류)
    import DSR_ROBOT2
    from DR_common2 import posx as _posx
    from dsr_msgs2.srv import SetRobotMode
    dr, posx = DSR_ROBOT2, _posx
    need = ["movel", "movej", "get_current_posx", "get_tool_force", "set_robot_mode", "set_tool",
            "set_tcp", "get_tool", "get_tcp", "task_compliance_ctrl", "release_compliance_ctrl",
            "ROBOT_MODE_MANUAL", "ROBOT_MODE_AUTONOMOUS", "DR_BASE", "DR_MV_MOD_REL"]
    missing = [n for n in need if not hasattr(dr, n)]
    if missing:
        sys.exit(f"[멈춤] DSR_ROBOT2에 {missing}가 없다. 워크스페이스 버전을 PL에게 알린다.")

    # 컨트롤러 서비스가 보일 때까지 기다린다 (보이기 전에 보낸 첫 명령은 사라져 멈춘다)
    probe = NODE.create_client(SetRobotMode, "dsr_controller2/system/set_robot_mode")
    t0 = time.time()
    while not probe.wait_for_service(timeout_sec=2.0):
        waited = int(time.time() - t0)
        print(f"컨트롤러를 기다리는 중 ({waited}초). 터미널 1의 브링업이 떠 있어야 한다.")
        if waited > 30:
            print("  30초가 넘었다: 브링업 터미널 오류를 보고, "
                  "'ros2 service list | grep set_robot_mode'로 이름이 /dsr01/dsr_controller2/... 인지 본다.")
    time.sleep(1.0)

    system = dr.get_robot_system() if hasattr(dr, "get_robot_system") else -1
    REAL = (system != 1)              # 1 = 가상. 0이나 모르면 실기로 본다 (안전한 쪽)
    print("로봇:", "실제 로봇" if REAL else "가상(에뮬레이터)")


def setup_tool():
    """수동 모드 → 툴·TCP 설정 → 자동 모드. 브링업을 켤 때마다 선택이 풀리므로 매번 한다."""
    if REAL and GAP_OFFSET == 0.0:
        print("알림: GAP_OFFSET이 0이다. R-0 3번에서 잰 패드 간격이 35.0 mm였는지 본다.")
    if not REAL:
        print("가상 모드: 툴·TCP 설정은 건너뛴다 (에뮬레이터에는 등록된 툴이 없다)")
        return
    if not TOOL_NAME or not TCP_NAME:
        sys.exit("[멈춤] TOOL_NAME, TCP_NAME을 먼저 채운다 (R-0에서 펜던트에 등록한 이름). 로봇은 움직이지 않았다.")
    r1 = dr.set_robot_mode(dr.ROBOT_MODE_MANUAL)
    rt = dr.set_tool(TOOL_NAME)
    rc = dr.set_tcp(TCP_NAME)
    r2 = dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)
    print(f"수동 모드 {r1}, set_tool {rt}, set_tcp {rc}, 자동 모드 {r2}  (0 = 성공)")
    if rt != 0 or rc != 0 or r2 != 0:
        sys.exit("[멈춤] 툴/TCP 설정 실패. 이름을 펜던트와 글자 하나까지 같게 하고 다시 실행한다. 로봇은 움직이지 않았다.")
    print("지금 툴:", dr.get_tool(), " / 지금 TCP:", dr.get_tcp())


def tcp_now():
    r = dr.get_current_posx()
    if not isinstance(r, tuple) or r[0] is None:
        raise RuntimeError("get_current_posx 실패")
    return [round(float(v), 2) for v in r[0]]


def fz_now():
    f = dr.get_tool_force()
    return float(f[2]) if isinstance(f, (list, tuple)) and len(f) >= 3 else None


def show(tag):
    p, fz = tcp_now(), fz_now()
    print(f"[{tag}] TCP 좌표 {p}   Fz {'?' if fz is None else f'{fz:.1f}'} N")
    return p, fz


def moved(ret, what):
    if ret != 0:
        raise RuntimeError(f"{what} 이동 실패 (반환 {ret}). 브링업 터미널의 OnLogAlarm 줄을 본다.")


def go(p, dz=0.0, v=MOVE[0], a=MOVE[1]):
    """교시 자세 p에서 z만 dz 올린 자리로 직선 이동 (베이스 기준)."""
    ret = dr.movel(posx(p[0], p[1], p[2] + dz, p[3], p[4], p[5]), vel=v, acc=a, ref=dr.DR_BASE)
    moved(ret, f"z{dz:+.0f}")


def rel(dx=0.0, dy=0.0, dz=0.0, v=MOVE[0], a=MOVE[1]):
    """베이스 기준 상대 직선 이동 (mm)."""
    ret = dr.movel(posx(dx, dy, dz, 0.0, 0.0, 0.0), vel=v, acc=a, ref=dr.DR_BASE, mod=dr.DR_MV_MOD_REL)
    moved(ret, f"상대 [{dx}, {dy}, {dz}]")


def pose_of(name, value):
    if value is None or len(value) != 6:
        sys.exit(f"[멈춤] {name}을(를) 먼저 채운다: pose 모드로 교시해 [x, y, z, rx, ry, rz]를 넣는다.")
    p = [float(v) for v in value]
    if abs(abs(p[4]) - 180.0) > 3.0:
        print(f"  주의: {name}의 ry가 {p[4]:.1f}° — 180에서 3° 넘게 벗어나 손가락이 기울어 있다. 다시 교시하는 것이 좋다.")
    return p


def picks(kind):
    return pose_of("PICK_SIDE" if kind == "side" else "PICK_END", PICK_SIDE if kind == "side" else PICK_END)


def places(kind):
    return pose_of("PLACE_SIDE" if kind == "side" else "PLACE_END", PLACE_SIDE if kind == "side" else PLACE_END)


# ----------------------------------------------------------------- 시험
def mode_check():
    setup_tool()
    p, fz = show("지금")
    if REAL:
        if fz is None or abs(fz) > FZ_LIMIT:
            print("  Fz가 ±2 N 밖이다 → 툴 무게가 안 맞는다. 아무것도 안 닿았는지 보고, R-0 툴 무게를 다시 한다.")
        else:
            print("  Fz 정상 (±2 N 안)")
    print(f"  그리퍼 폭 {G.get_width():.1f} mm (읽히면 브링업과 같이 써도 연결 정상)")
    log(test="R-0 check", note=f"tool={TOOL_NAME} tcp={TCP_NAME} pos={p} fz={fz}")


def mode_pose():
    setup_tool()
    if REAL:
        r = dr.set_robot_mode(dr.ROBOT_MODE_MANUAL)
        print(f"수동 모드로 바꿈 ({r}, 0 = 성공). 로봇은 스스로 움직이지 않는다.")
        print("손목의 직접 교시 버튼을 누른 채 팔을 손으로 끌어 옮긴다.")
    print("명령: Enter = 좌표 출력 / os = 옆면 열기(패드 간격 35 mm) / cs = 옆면 닫기 20 mm·20 N"
          " / oe = 끝면 열기(패드 간격 85 mm) / ce = 끝면 닫기 70 mm·20 N / q = 끝")
    while True:
        cmd = ask("> ", "q").strip().lower()
        if cmd == "q":
            break
        if cmd in ("os", "oe", "cs", "ce"):
            gp = GRASP["side" if cmd[1] == "s" else "end"]
            if cmd[0] == "c":
                ask("  손가락 사이에 손이 없으면 Enter ")
                grip, w = gmove(gp["close"], 200)
                print(f"  닫음: 잡힘 감지 {grip}, 폭 값 {w:.1f} mm. 닫으면 손가락 끝이 조금 내려간다 — 높이를 다시 본다.")
            else:
                _, w = gmove(open_code(gp), OPEN_FORCE)
                print(f"  열림: 폭 값 {w:.1f} mm (패드 간격 약 {w - GAP_OFFSET:.1f} mm)")
        p = tcp_now()
        print(f"  지금 자세: {p}")
        if abs(abs(p[4]) - 180.0) > 3.0:
            print("  주의: ry가 180에서 3° 넘게 벗어났다 → 손가락이 기울어 있다")
    print("끝. 로봇은 수동 모드로 남는다. 다음 시험을 실행하면 자동 모드로 바뀐다.")


def mode_v16(cycles=3):
    setup_tool()
    p0, _ = show("시작 자리")
    dx, dy = float(V16_MOVE[0]), float(V16_MOVE[1])
    stops = resumes = 0
    for i in range(1, cycles + 1):
        ready(f"V-16 {i}/{cycles}: 폼 블록이 받침에 고정되어 경로 위에 있다. 로봇이 [{dx}, {dy}] mm를 50 mm/s로 간다")
        target = posx(p0[0] + dx, p0[1] + dy, p0[2], p0[3], p0[4], p0[5])   # 늘 시작 자리 기준 (상대 이동 아님)
        ret = dr.movel(target, vel=50.0, acc=100.0, ref=dr.DR_BASE)
        print(f"  이동 명령 반환 {ret} (멈추면 -1이 나올 수 있다)")
        stopped = yes("  폼에 닿아 멈췄나?")
        ask("  펜던트에 알림 창이 떴으면 확인을 누른다. 그다음 Enter ")
        ready("시작 자리로 되돌아간다")
        ret2 = dr.movel(posx(*p0), vel=50.0, acc=100.0, ref=dr.DR_BASE)
        resumed = yes("  시작 자리로 되돌아갔나?")
        stops += stopped
        resumes += resumed
        log(test="V-16", trial=i, note=f"stopped={stopped} resumed={resumed} ret={ret}/{ret2}")
        if ret2 != 0 or not resumed:
            print(f"  시작 자리로 못 돌아갔다 (반환 {ret2}) → 여기서 멈춘다. 펜던트 알림을 확인하고 'v16'을 다시 실행한다.")
            break
    print(f"V-16: 멈춤 {stops}/{cycles}, 다시 움직임 {resumes}/{cycles}")


def mode_v20(kind, force_n, speed, count):
    gp, p, force = GRASP[kind], picks(kind), force_code(force_n)
    v, a = SPEED[speed]
    dx, dy = float(SIDE_MOVE[0]), float(SIDE_MOVE[1])
    setup_tool()
    show("시작")
    print(f"V-20 {gp['name']}, 힘 {force_n} N, {speed} (vel {v}, acc {a}), {count}번")
    ready("집는 자리 50 mm 위로 간다")
    go(p, APPROACH)
    slips = 0
    for i in range(1, count + 1):
        ready(f"V-20 {i}/{count}: 블록이 집는 자리(막이에 붙여)에 있다")
        gmove(open_code(gp), OPEN_FORCE)               # 패드 간격 = 블록 + 10 mm로 연다
        go(p, 0.0, *DESCENT)                           # 20 mm/s로 수직 하강
        grip, w1 = gmove(gp["close"], force)
        if not grip:
            print("  잡힘 감지가 없다 → 열고 올라간 뒤 멈춘다. 블록 자리와 잡는 높이를 본다.")
            gmove(open_code(gp), OPEN_FORCE)
            go(p, APPROACH)
            log(test="V-20", grasp=kind, force_N=force_n, speed=speed, trial=i, grip=0, note="잡힘 감지 없음")
            break
        print(f"  사진 1 (잡음, 폭 {w1:.1f} mm)")
        time.sleep(0.5)
        rel(dz=100.0, v=v, a=a)
        rel(dx=dx, dy=dy, v=v, a=a)
        rel(dz=-95.0, v=v, a=a)                        # 책상 5 mm 위에서 멈춘다
        print("  사진 2")
        time.sleep(0.5)
        grip2, w2 = bool(gstatus()[1]), float(G.get_width())
        if not grip2:
            print("  잡힘 감지가 꺼졌다 = 블록이 빠졌다 → 미끄러짐. 위로 95 mm 올라간 뒤 멈춘다 (블록을 찾아 치운다).")
            log(test="V-20", grasp=kind, force_N=force_n, speed=speed, trial=i, grip=1, width_mm=w1,
                grip_after=0, width_after_mm=w2, slip="y", note="블록 빠짐")
            slips += 1
            rel(dz=95.0)
            break
        rel(dz=95.0, v=v, a=a)
        rel(dx=-dx, dy=-dy, v=v, a=a)
        go(p, APPROACH, v, a)
        go(p, 2.0, *DESCENT)                           # 제자리 2 mm 위에서 놓는다
        gmove(open_code(gp), OPEN_FORCE)
        go(p, APPROACH)
        ans = ask("  미끄러짐 없으면 Enter / 있으면 s / 그만은 q. 메모는 뒤에 이어 쓴다 (예: s 살짝 기움) → ", "").strip()
        flag = ans[:1].lower()
        slip, stop = flag == "s", flag == "q"
        note = ans[1:].strip() if flag in ("s", "q") else ans
        slips += slip
        log(test="V-20", grasp=kind, force_N=force_n, speed=speed, trial=i, grip=1, width_mm=w1,
            grip_after=1, width_after_mm=w2, slip="y" if slip else "n", note="selftest" if AUTO else note)
        if stop:
            break
    print(f"V-20 {gp['name']} {force_n} N {speed}: 미끄러짐 {slips}번 (기록: {LOG})")


def mode_v11(method, kind, force_n, count, height):
    gp, p, q, force = GRASP[kind], picks(kind), places(kind), force_code(force_n)
    global COMPLIANCE_ON
    v, a = SPEED["slow"]
    setup_tool()
    show("시작")
    hover = height if method == "A" else 2.0            # 목표 위 이 높이까지는 위치 제어로 내려간다
    press = height if method == "B" else 0.0            # B: 목표 아래 이만큼까지 순응 하강 명령
    print(f"V-11 방법 {method}, {gp['name']}, 힘 {force_n} N, {count}번, "
          + (f"목표 {hover} mm 위에서 연다" if method == "A" else f"순응 제어로 목표 {press} mm 아래까지 명령"))
    ready("집는 자리 50 mm 위로 간다")
    go(p, APPROACH)
    for i in range(1, count + 1):
        ready(f"V-11 {method} {i}/{count}: 새 블록이 집는 자리(막이에 붙여)에 있고, 아래 블록 2개가 테이프 자리에 있다")
        gmove(open_code(gp), OPEN_FORCE)
        go(p, 0.0, *DESCENT)
        grip, _ = gmove(gp["close"], force)
        if not grip:
            print("  잡힘 감지가 없다 → 열고 올라간 뒤 멈춘다.")
            gmove(open_code(gp), OPEN_FORCE)
            go(p, APPROACH)
            break
        go(p, APPROACH)
        go(q, APPROACH, v, a)                          # 놓는 자리 50 mm 위
        go(q, hover, 10.0, 20.0)                       # 10 mm/s로 목표 위까지 (위치 제어)
        note = ""
        if method == "B":
            fz = fz_now()
            print(f"  순응 제어 전 Fz {'?' if fz is None else f'{fz:.1f}'} N")
            if REAL and (fz is None or abs(fz) > FZ_LIMIT):
                print("  Fz가 ±2 N 밖 → 툴 무게가 안 맞아 순응 제어를 켜면 처질 수 있다. 여기서 놓고 멈춘다.")
                gmove(open_code(gp), PLACE_OPEN_FORCE)
                rel(dz=30.0, v=20.0, a=50.0)
                log(test="V-11", method=method, grasp=kind, force_N=force_n, trial=i, note=f"Fz {fz}: 멈춤")
                break
            r = dr.task_compliance_ctrl()             # 기본 강성 [3000, 3000, 3000, 200, 200, 200]
            COMPLIANCE_ON = True
            print(f"  순응 제어 켬 ({r}, 0 = 성공)")
            time.sleep(0.5)
            ret = dr.movel(posx(q[0], q[1], q[2] - press, q[3], q[4], q[5]), vel=5.0, acc=10.0, ref=dr.DR_BASE)
            time.sleep(0.5)
            z = tcp_now()[2]
            print(f"  순응 하강 반환 {ret}, 지금 z {z:.2f} (목표 z {q[2]:.2f})")
            if ret != 0 or z > q[2] + hover - 0.5:
                note = "B 하강 안 됨(거부?) → OnLogAlarm 확인"
                print("  " + note)
        _, wo = gmove(open_code(gp), PLACE_OPEN_FORCE)  # 패드 간격 블록 + 10 mm까지만, 3 N으로 천천히 연다
        if COMPLIANCE_ON:
            r = dr.release_compliance_ctrl()
            COMPLIANCE_ON = False
            print(f"  순응 제어 끔 ({r})")
            time.sleep(0.5)
        if REAL and wo - GAP_OFFSET < gp["block"] + 3.0:
            sys.exit(f"[멈춤] 그리퍼가 다 안 열렸다 (폭 {wo:.1f} mm). 올라가면 블록을 끌 수 있어 멈춘다. PL과 확인한다.")
        rel(dz=30.0, v=20.0, a=50.0)                   # 20 mm/s로 30 mm 빠진다
        go(q, APPROACH, v, a)
        go(p, APPROACH, v, a)                          # 집는 자리 위로 비켜난다 (재는 동안 놓는 자리에서 떨어져 있게)
        if note:
            log(test="V-11", method=method, grasp=kind, force_N=force_n, trial=i, note=note)
            print("  방법 B를 '못 함(사유)'으로 적고 PL에게 알린다.")
            break
        shift = ask_float("  밀림 (mm, 목표 선에서 x·y 중 큰 값): ")
        lower = yes("  아래 블록이 테이프 자리에서 움직였나?", auto="n")
        memo = ask("  메모 (없으면 Enter): ", "selftest" if AUTO else "")
        log(test="V-11", method=method, grasp=kind, force_N=force_n, trial=i, shift_mm=shift,
            lower_moved="y" if lower else "n", note=f"높이 {height} mm" + (f", {memo}" if memo else ""))
        print("  새 블록을 집는 자리(막이)에 다시 놓는다.")
    print(f"V-11 방법 {method} 끝 (기록: {LOG})")


def mode_selftest():
    """가상 모드 전용: 가상 자세를 만들어 모든 시험 동작을 한 번씩 돌린다."""
    global AUTO, LOG, PICK_SIDE, PICK_END, PLACE_SIDE, PLACE_END
    if REAL:
        sys.exit("[멈춤] selftest는 가상 모드(에뮬레이터)에서만 한다.")
    AUTO = True
    LOG = os.path.join(HERE, "jenga_selftest_log.csv")
    print("준비 자세 [0, 0, 90, 0, 90, 0]로 간다 (가상)")
    dr.movej([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel=30.0, acc=30.0)
    r0 = tcp_now()
    PICK_SIDE = [r0[0], r0[1], r0[2] - 200.0, r0[3], r0[4], r0[5]]
    PICK_END = list(PICK_SIDE)
    PLACE_SIDE = [r0[0] + 100.0, r0[1] - 150.0, r0[2] - 185.0, r0[3], r0[4], r0[5]]
    PLACE_END = list(PLACE_SIDE)
    print("가상 PICK", PICK_SIDE, " 가상 PLACE", PLACE_SIDE)
    mode_check()
    mode_v16(cycles=1)
    mode_v20("side", 20, "fast", 1)
    mode_v20("end", 20, "slow", 1)
    mode_v11("A", "side", 20, 1, 2.0)
    mode_v11("B", "side", 20, 1, 1.0)
    print("\nselftest 끝: 위에 [멈춤]이나 Traceback이 없으면 이 PC에서 스크립트가 돈다.")


# ----------------------------------------------------------------- 시작
def main():
    global G, COMPLIANCE_ON
    args = sys.argv[1:]
    modes = ("gripper", "check", "pose", "v16", "v20", "v11", "selftest")
    if not args or args[0] not in modes:
        print(__doc__)
        sys.exit(1)
    mode = args[0]

    g, info = connect_gripper()
    if mode == "gripper":
        if g is None:
            sys.exit(f"[멈춤] 그리퍼 연결 실패: {info}\n  확인: ping 192.168.1.1, 유선 IP 192.168.1.x, pymodbus 3.6.x")
        G = g
        width_mm = float(args[1]) if len(args) > 1 else 35.0
        force_n = float(args[2]) if len(args) > 2 else 20.0
        if not 0.0 <= width_mm <= 110.0:
            sys.exit("[멈춤] 폭은 0~110 mm")
        print(f"지금 폭 {info:.1f} mm → 목표 {width_mm:.1f} mm, 힘 {force_n:.0f} N")
        ask("손가락 사이에 손이 없으면 Enter ")
        grip, w = gmove(int(round(width_mm * 10)), force_code(force_n))
        print(f"폭 {w:.1f} mm, 잡힘 감지 {grip}")
        return

    try:
        if mode == "v20":
            kind, force_n, speed = args[1], float(args[2]), args[3]
            count = int(args[4]) if len(args) > 4 else 5
            if kind not in GRASP or speed not in SPEED:
                raise ValueError
        elif mode == "v11":
            method, kind, force_n = args[1].upper(), args[2], float(args[3])
            count = int(args[4]) if len(args) > 4 else 10
            height = float(args[5]) if len(args) > 5 else (2.0 if method == "A" else 1.0)
            if method not in ("A", "B") or kind not in GRASP or not 0.0 <= height <= 5.0:
                raise ValueError
    except (IndexError, ValueError):
        print(__doc__)
        sys.exit("[멈춤] 인자가 틀렸다. 위 예를 본다.")

    start_ros()
    if g is None:
        if REAL:
            sys.exit(f"[멈춤] 그리퍼 연결 실패: {info}\n  확인: ping 192.168.1.1, 유선 IP 192.168.1.x, pymodbus 3.6.x."
                     "\n  브링업을 끄고 'gripper' 명령은 되는지도 본다 (연결 수 문제인지).")
        print("가상 모드: 그리퍼는 흉내만 낸다")
        G = FakeGripper()
    else:
        G = g
    try:
        if mode == "check":
            mode_check()
        elif mode == "pose":
            mode_pose()
        elif mode == "v16":
            mode_v16()
        elif mode == "v20":
            mode_v20(kind, force_n, speed, count)
        elif mode == "v11":
            mode_v11(method, kind, force_n, count, height)
        elif mode == "selftest":
            mode_selftest()
    finally:
        if COMPLIANCE_ON:
            try:
                dr.release_compliance_ctrl()
                print("순응 제어를 껐다")
            except Exception as e:
                print("순응 제어 끄기 실패:", e)
        try:
            import rclpy
            NODE.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
