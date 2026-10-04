#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[R-01] Ctrl+C·'앞길 막힘' 때 로봇을 실제로 세우는 공통 기능 (real_demo, task3_demo 에 같은 파일).

왜 필요한가 (10/3 가상 시험)
  - MoveIt 실행 취소(cancel)와 두산 motion/move_stop 만으로는 로봇이 안 섰다.
  - 궤적 컨트롤러(dsr_moveit_controller/follow_joint_trajectory)에 '지금 자리에 0.3 s 안에 서기'
    궤적(점 1개 = 지금 관절값, 속도 0)을 보내면 섰다.
  - rclpy.init() 기본값은 Ctrl+C 를 받으면 ROS 를 먼저 꺼서 정지 명령을 못 보낸다.
    -> init_ros() 로 시작한다 (= rclpy.init(signal_handler_options=SignalHandlerOptions.NO)).
  - 10/4 가상 시험: 서기 궤적 바로 뒤에 move_stop 을 부르면 에뮬레이터가 멈춰 버린 적이 있다(6번 중 1번,
    joint_states 끊김, 브링업 다시 켬). 그래서 move_stop 은 서기 궤적으로 안 설 때만 쓰는 '예비'로 둔다.

정지 순서 stop()
  1. 서기 궤적 (dsr_moveit_controller/follow_joint_trajectory)
  2. MoveIt 실행 취소 (실행 중인 ExecuteTrajectory 목표가 있으면)
  3. 관절이 설 때까지 기다림 (0.3 s 동안 변화 0.05도 미만, 그동안 joint_states 가 새로 3번 이상 와야 함)
  4. FIRST_WAIT 초 안에 안 서면 두산 move_stop (빠른 정지) 을 부르고 다시 기다림
  5. 그래도 확인 못 하면 '펜던트 비상정지!' 를 크게 띄운다.
  정지하는 동안 Ctrl+C 를 또 눌러도 정지가 끊기지 않는다. (막힘 정지 중에 누르면, 다 세운 뒤 Ctrl+C 로 끝낸다)

쓰는 법
  from safe_stop import SafeStop, init_ros
  init_ros()                                   # rclpy.init() 대신
  stopper = SafeStop(node)                     # joint_states 를 스스로 받는다
  stopper.prepare()                            # 브링업이 뜬 뒤 한 번 (정지 서비스 찾기)
  ...
  except KeyboardInterrupt:
      stopper.stop('Ctrl+C')
"""
import math
import signal
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
HOLD_ACTION = 'dsr_moveit_controller/follow_joint_trajectory'
HOLD_S = 0.3              # 이 시간 안에 지금 자리에 선다 (s)
STILL_DEG = 0.05          # STILL_S 동안 관절 변화가 이보다 작으면 '섰다' (도)
STILL_S = 0.3
FRESH_N = 3               # 그 사이 joint_states 가 이만큼은 새로 와야 '섰다'로 본다 (안 오면 확인 못 함)
FIRST_WAIT = 2.0          # 서기 궤적 뒤 이 시간 안에 안 서면 move_stop (s)
STOP_WAIT = 2.0           # move_stop 뒤 이 시간 안에도 안 서면 '펜던트 비상정지!' (s)
MOVE_STOP_MODE = 1        # 두산 move_stop: 1 = DR_QSTOP (빠른 정지)
MOVE_STOP_ON_EXIT = False # True 면 Ctrl+C 때 move_stop 을 늘 같이 부른다 (실기 시험 뒤 정한다. 위 설명 참고)
ALARM = '!' * 64


def init_ros(args=None):
    """rclpy.init() 대신 쓴다. Ctrl+C 뒤에도 ROS 가 살아 있어야 정지 명령을 보낼 수 있다."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)


class SafeStop:
    def __init__(self, node, hold_action=HOLD_ACTION):
        self.node = node
        self.js = {}                      # 관절 이름 -> 위치 (rad). 그리퍼 joint_states 가 섞여 와도 팔 관절만 쓴다
        self.js_n, self.js_t = 0, 0.0     # 팔 관절이 든 joint_states 를 받은 횟수, 마지막으로 받은 시각
        node.create_subscription(JointState, 'joint_states', self._on_js, 50)
        self.hold_ac = ActionClient(node, FollowJointTrajectory, hold_action)
        self.gh = None                    # 실행 중인 MoveIt 목표 (ExecuteTrajectory). 실행할 때 넣고, 끝나면 None
        self._stop_cli = None
        self.last = {}                    # 마지막 정지 기록 (이유, 섰나, 걸린 시간)

    # ---------- 기본 ----------
    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.js[n] = p
        if JOINTS[0] in msg.name:
            self.js_n += 1
            self.js_t = time.monotonic()

    def get_q(self):
        """지금 관절값 (rad, JOINTS 순서). 아직 못 받았으면 KeyError -> stop() 이 '펜던트 비상정지!' 로 알린다."""
        rclpy.spin_once(self.node, timeout_sec=0.0)
        return [self.js[j] for j in JOINTS]

    def _wait(self, fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        return fut.result() if fut.done() else None

    def _spin(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def prepare(self):
        """브링업이 뜬 뒤 한 번 부른다. 서기 궤적 컨트롤러와 두산 move_stop 서비스를 미리 찾는다."""
        ok = self.hold_ac.wait_for_server(timeout_sec=5.0)
        print('[정지 준비] 서기 궤적 컨트롤러: ' + ('있음' if ok else '없음 -> Ctrl+C 로 못 세울 수 있다. 펜던트를 꼭 든다'))
        self._find_move_stop()
        return ok

    def _find_move_stop(self):
        if self._stop_cli is not None:
            return True
        try:
            from dsr_msgs2.srv import MoveStop
        except ImportError:
            print('[정지 준비] dsr_msgs2 를 못 불러 두산 move_stop 은 쓰지 않는다 (워크스페이스 source 확인).')
            return False
        names = [n for n, _ in self.node.get_service_names_and_types() if n.endswith('/motion/move_stop')]
        if not names:
            print('[정지 준비] motion/move_stop 서비스가 안 보인다. 서기 궤적만 쓴다.')
            return False
        name = sorted(names, key=len)[0]
        self._stop_cli = (self.node.create_client(MoveStop, name), MoveStop)
        print(f'[정지 준비] 두산 move_stop 서비스: {name}')
        return True

    # ---------- 정지 ----------
    def hold_here(self):
        """컨트롤러에 '지금 자리에 HOLD_S 초 안에 서라'는 궤적을 보내 실행 중인 궤적을 덮어쓴다."""
        q = list(self.get_q())
        if not self.hold_ac.wait_for_server(timeout_sec=1.0):
            print('[정지] 궤적 컨트롤러(dsr_moveit_controller)를 못 찾았다.')
            return False
        jt = JointTrajectory(joint_names=list(JOINTS))
        jt.points = [JointTrajectoryPoint(positions=q, velocities=[0.0] * 6,
                                          time_from_start=Duration(sec=0, nanosec=int(HOLD_S * 1e9)))]
        gh = self._wait(self.hold_ac.send_goal_async(FollowJointTrajectory.Goal(trajectory=jt)), 2.0)
        ok = gh is not None and gh.accepted
        print('[정지] "지금 자리에 서기" 궤적을 보냈다' + ('' if ok else ' (거절됨 또는 응답 없음)'))
        return ok

    def _cancel(self):
        gh, self.gh = self.gh, None
        if gh is None:
            return
        self._wait(gh.cancel_goal_async(), 1.0)
        print('[정지] MoveIt 실행을 취소했다.')

    def _move_stop(self):
        if not self._find_move_stop():
            return False
        cli, srv = self._stop_cli
        if not cli.wait_for_service(timeout_sec=1.0):
            print('[정지] 두산 move_stop 서비스가 응답하지 않는다.')
            return False
        res = self._wait(cli.call_async(srv.Request(stop_mode=MOVE_STOP_MODE)), 2.0)
        print('[정지] 두산 move_stop(빠른 정지)도 불렀다' + ('' if res is not None else ' (응답 없음)'))
        return res is not None

    def wait_still(self, timeout=FIRST_WAIT):
        """관절이 설 때까지 (STILL_S 동안 변화 STILL_DEG 미만) 기다린다. 반환: 섰으면 True.
        joint_states 가 안 오면 값이 안 바뀌어도 '섰다'로 보지 않는다 (확인 못 함 -> False)."""
        end = time.monotonic() + timeout
        prev, n0 = list(self.get_q()), self.js_n
        while time.monotonic() < end:
            self._spin(STILL_S)
            cur = list(self.get_q())
            fresh = self.js_n - n0 >= FRESH_N and time.monotonic() - self.js_t < 0.2
            if fresh and max(abs(math.degrees(a - b)) for a, b in zip(cur, prev)) < STILL_DEG:
                return True
            prev, n0 = cur, self.js_n
        if time.monotonic() - self.js_t >= 0.2:
            print(f'[정지] joint_states 가 {time.monotonic() - self.js_t:.1f} s 동안 안 왔다. 정지를 확인할 수 없다.')
        return False

    def stop(self, why, exiting=True):
        """로봇을 지금 자리에 세우고, 설 때까지 기다린다. 반환: 섰으면 True.
        서기 궤적 -> 취소 -> 설 때까지 기다림. FIRST_WAIT 초 안에 안 서면 move_stop 하고 한 번 더 기다림.
        exiting=True (Ctrl+C, 끝낼 때) 이고 MOVE_STOP_ON_EXIT 이면 move_stop 을 처음부터 같이 부른다."""
        old = signal.getsignal(signal.SIGINT)
        pressed = []

        def on_sigint(*_):
            pressed.append(1)
            print('\n[정지] 정지하는 중이다. 끝날 때까지 기다린다 ...')
        signal.signal(signal.SIGINT, on_sigint)
        t0 = time.monotonic()
        stopped = False
        try:
            print(f'\n[정지] {why}: 지금 자리에 세운다 ...')
            self.hold_here()
            self._cancel()
            if exiting and MOVE_STOP_ON_EXIT:
                self._move_stop()
            stopped = self.wait_still(FIRST_WAIT)
            if not stopped:
                print('[정지] 서기 궤적으로 안 섰다 -> 두산 move_stop (빠른 정지)')
                self._move_stop()
                stopped = self.wait_still(STOP_WAIT)
        except Exception as e:            # 정지 실패를 삼키지 않는다: 아래에서 크게 알린다
            print(f'[정지] 오류: {e!r}')
            stopped = False
        finally:
            signal.signal(signal.SIGINT, old)
        dt = time.monotonic() - t0
        self.last = dict(why=why, stopped=stopped, sec=dt)
        if stopped:
            print(f'[정지] 섰다 (명령부터 정지 확인까지 {dt:.2f} s)')
        else:
            print(f'\n{ALARM}\n[정지] 로봇이 아직 움직이거나 정지를 확인하지 못했다 -> 펜던트 비상정지!\n{ALARM}\n')
        if pressed and not exiting:
            raise KeyboardInterrupt          # 막힘 정지 중에 Ctrl+C 를 눌렀다: 다 세운 뒤 끝내기로 넘긴다
        return stopped
