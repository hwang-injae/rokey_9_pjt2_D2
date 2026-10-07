# -*- coding: utf-8 -*-
"""로봇을 지금 자리에 세우는 공통 기능 SafeStop — 정지 노드(safety_stop)와 집기·놓기(pick_place) 두 곳에서 쓴다.

왜 이렇게 세우나 (R-01, docs/troubleshooting/TS-01)
  - MoveIt 실행 취소와 두산 move_stop 만으로는 팔이 안 섰다 (10/3 가상).
  - 궤적 컨트롤러에 '지금 관절값에 0.3 s 안에 서라'는 점 하나짜리 궤적을 보내면 실행 중인 궤적을 덮어써서 선다.
  - 10/4 실기: 컨트롤러 토픽(dsr_moveit_controller/joint_trajectory)으로 보내고, MoveIt 목표를 취소한 뒤
    한 번 더 보내는 방식으로 Ctrl+C 바로 정지를 확인했다 (박진용, real_demo/pick_place_avoid.py).
  - rclpy 기본 신호 처리는 Ctrl+C 에 ROS 를 먼저 꺼서 정지 명령을 못 보낸다 -> init_ros() 로 시작한다.
  - move_stop 은 서기 궤적으로 안 설 때만 쓰는 예비다. 서기 궤적 바로 뒤에 부르면 에뮬레이터가
    멈춘 적이 있다 (10/4 가상, 7번 중 1번).
"""
import math
import signal
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
HOLD_TOPIC = '/dsr_moveit_controller/joint_trajectory'
HOLD_S = 0.3          # 이 시간 안에 지금 자리에 선다 (s). 더 짧으면 컨트롤러가 급정지 가속을 못 낸다
STILL_DEG = 0.05      # STILL_S 동안 관절 변화가 이보다 작으면 '섰다' (도, R-01 시험 기준)
STILL_S = 0.3
FRESH_N = 3           # 그 사이 joint_states 가 이만큼 새로 와야 '섰다'로 본다 (안 오면 값이 그대로여도 확인 못 함)
STOP_WAIT_S = 2.0     # move_stop 뒤 이 시간 안에도 안 서면 '펜던트 비상정지!'
MOVE_STOP_MODE = 1    # 두산 move_stop: 1 = DR_QSTOP (빠른 정지)
ALARM = '!' * 64


def init_ros(args=None):
    """rclpy.init() 대신 쓴다. Ctrl+C 를 ROS 가 아니라 우리 코드가 받아, 정지 명령을 보낸 뒤 끝낸다."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)


class SafeStop:
    """로봇 팔을 지금 자리에 세우고, 설 때까지 확인한다.

    받는 것: joint_states (팔 관절값)
    내보내는 것: 서기 궤적 (dsr_moveit_controller/joint_trajectory), 안 서면 두산 motion/move_stop 호출
    쓰는 쪽: MoveIt 실행 목표를 보낼 때 self.gh 에 넣어 두면 stop() 이 같이 취소한다.
    spin_node=True (혼자 도는 노드, 예: safety_stop) 면 기다리는 동안 rclpy.spin_once 로 node 를 직접 돌리고,
    False (다른 스레드의 executor 가 node 를 돌리는 노드, 예: pick_place) 면 기다리기만 한다.
    """

    def __init__(self, node, first_wait_s, spin_node=True):
        """node: 구독·발행에 쓸 rclpy 노드, first_wait_s: 서기 궤적 뒤 이 안에 안 서면 move_stop (s, robot.yaml stop.first_wait_s)."""
        self.node = node
        self.first_wait_s = first_wait_s
        self.spin_node = spin_node
        self.js = {}                      # 관절 이름 -> 위치 (rad). 그리퍼 관절이 섞여 와도 팔 관절만 쓴다
        self.js_n, self.js_t = 0, 0.0     # 팔 관절이 든 joint_states 받은 횟수, 마지막 받은 시각
        node.create_subscription(JointState, 'joint_states', self._on_js, 50)
        self.hold_pub = node.create_publisher(JointTrajectory, HOLD_TOPIC, 10)
        self.gh = None                    # 실행 중인 MoveIt 목표. 쓰는 쪽이 넣고, 끝나면 None
        self._stop_cli = None
        self.last = {}                    # 마지막 정지 기록 (why, stopped, sec)

    def _on_js(self, msg):
        """joint_states 를 받아 관절값과 받은 시각을 적는다."""
        for n, p in zip(msg.name, msg.position):
            self.js[n] = p
        if JOINTS[0] in msg.name:
            self.js_n += 1
            self.js_t = time.monotonic()

    def _spin(self, sec):
        """sec 초 동안 콜백이 돌게 한다. 다른 executor 가 node 를 돌리면 기다리기만 한다 (같은 node 를 두 번 돌리지 않게).

        node.executor 로 판단하지 않는 이유: rclpy.spin_once 가 한 번 돌린 node 에 executor 표시를 남겨서 틀린다 (10/6 시험).
        """
        if self.spin_node:
            rclpy.spin_once(self.node, timeout_sec=sec)
        else:
            time.sleep(sec)

    def get_q(self):
        """지금 팔 관절값 (rad, JOINTS 순서). 아직 못 받았으면 KeyError -> stop() 이 '펜던트 비상정지!' 로 알린다."""
        self._spin(0.0)
        return [self.js[j] for j in JOINTS]

    def _wait(self, fut, timeout):
        """future 가 끝날 때까지 node 를 돌린다. 반환: 결과, timeout 초 안에 안 끝나면 None."""
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            self._spin(0.01)
        return fut.result() if fut.done() else None

    def prepare(self):
        """브링업이 뜬 뒤 한 번 부른다. 컨트롤러가 서기 궤적 토픽을 듣는지, 두산 move_stop 이 있는지 찾아 화면에 알린다.

        반환: 서기 궤적을 받을 컨트롤러가 있으면 True. 없으면 Ctrl+C 로 못 세울 수 있으니 펜던트를 든다.
        """
        end = time.monotonic() + 5.0
        while self.hold_pub.get_subscription_count() == 0 and time.monotonic() < end:
            self._spin(0.1)
        ok = self.hold_pub.get_subscription_count() > 0
        self.node.get_logger().info('[정지 준비] 서기 궤적 컨트롤러: ' + ('있음' if ok else '없음 -> 펜던트를 꼭 든다'))
        self._find_move_stop()
        return ok

    def _find_move_stop(self):
        """두산 motion/move_stop 서비스 클라이언트를 한 번 만든다. 반환: 쓸 수 있으면 True."""
        if self._stop_cli is not None:
            return True
        try:
            from dsr_msgs2.srv import MoveStop
        except ImportError:
            self.node.get_logger().warn('[정지 준비] dsr_msgs2 를 못 불러 move_stop 은 안 쓴다 (두산 워크스페이스 source 확인)')
            return False
        names = [n for n, _ in self.node.get_service_names_and_types() if n.endswith('/motion/move_stop')]
        if not names:
            self.node.get_logger().warn('[정지 준비] motion/move_stop 이 안 보인다. 서기 궤적만 쓴다')
            return False
        name = sorted(names, key=len)[0]
        self._stop_cli = (self.node.create_client(MoveStop, name), MoveStop)
        self.node.get_logger().info(f'[정지 준비] 두산 move_stop: {name}')
        return True

    def hold_here(self):
        """컨트롤러에 '지금 자리에 HOLD_S 초 안에 서라' 궤적을 보내고, MoveIt 목표를 취소한 뒤 한 번 더 보낸다.

        바깥 영향: 로봇 팔이 선다. 두 번 보내는 이유: 취소가 처리되는 사이 MoveIt 이 남은 궤적을 다시 보낼 수 있어서 (10/4 실기).
        """
        self._publish_hold()
        gh, self.gh = self.gh, None
        if gh is not None:
            self._wait(gh.cancel_goal_async(), 1.0)
        self._publish_hold()

    def _publish_hold(self):
        """지금 관절값, 속도 0 인 점 하나짜리 궤적을 컨트롤러 토픽에 보낸다."""
        jt = JointTrajectory(joint_names=list(JOINTS))
        jt.points = [JointTrajectoryPoint(positions=self.get_q(), velocities=[0.0] * 6,
                                          time_from_start=Duration(sec=0, nanosec=int(HOLD_S * 1e9)))]
        self.hold_pub.publish(jt)

    def _move_stop(self):
        """두산 move_stop(빠른 정지)을 부른다. 반환: 응답을 받았으면 True."""
        if not self._find_move_stop():
            return False
        cli, srv = self._stop_cli
        if not cli.wait_for_service(timeout_sec=1.0):
            return False
        return self._wait(cli.call_async(srv.Request(stop_mode=MOVE_STOP_MODE)), 2.0) is not None

    def wait_still(self, timeout):
        """관절이 설 때까지 (STILL_S 동안 변화 STILL_DEG 미만) 기다린다.

        반환: 섰으면 True. joint_states 가 안 오면 값이 그대로여도 '섰다'로 보지 않는다 (확인 못 함 -> False).
        """
        end = time.monotonic() + timeout
        prev, n0 = self.get_q(), self.js_n
        while time.monotonic() < end:
            t_end = time.monotonic() + STILL_S
            while time.monotonic() < t_end:
                self._spin(0.01)
            cur = self.get_q()
            fresh = self.js_n - n0 >= FRESH_N and time.monotonic() - self.js_t < 0.2
            if fresh and max(abs(math.degrees(a - b)) for a, b in zip(cur, prev)) < STILL_DEG:
                return True
            prev, n0 = cur, self.js_n
        return False

    def stop(self, why):
        """로봇을 지금 자리에 세우고 설 때까지 기다린다.

        입력: why = 정지 이유 (화면·기록용 글자)
        순서: 서기 궤적 + 취소 -> first_wait_s 안에 안 서면 move_stop -> STOP_WAIT_S 더 기다림.
        정지하는 동안 Ctrl+C 를 또 눌러도 끊기지 않는다.
        반환: 섰으면 True. 못 세웠거나 확인 못 하면 '펜던트 비상정지!' 를 크게 띄우고 False (예외를 밖으로 던지지 않는다).
        """
        t0 = time.monotonic()
        stopped = False
        # 서기 궤적부터 보낸다 — 그 앞에서 무엇이 실패해도 세우는 것이 먼저다
        try:
            print(f'\n[정지] {why}: 지금 자리에 세운다 ...')
            self.hold_here()
        except Exception as e:
            print(f'[정지] 서기 궤적 오류: {e!r}')
        # Ctrl+C 를 또 눌러도 정지가 끊기지 않게 한다. 신호 처리기는 메인 스레드에서만 바꿀 수 있다 (pick_place 는 다른 스레드에서 부름)
        main = threading.current_thread() is threading.main_thread()
        old = signal.getsignal(signal.SIGINT) if main else None
        if main:
            signal.signal(signal.SIGINT, lambda *_: print('\n[정지] 정지하는 중이다. 끝날 때까지 기다린다 ...'))
        try:
            stopped = self.wait_still(self.first_wait_s)
            if not stopped:
                print('[정지] 서기 궤적으로 안 섰다 -> 두산 move_stop (빠른 정지)')
                self._move_stop()
                stopped = self.wait_still(STOP_WAIT_S)
        except Exception as e:            # 정지 실패를 삼키지 않는다: 아래에서 크게 알린다
            print(f'[정지] 오류: {e!r}')
        finally:
            if main:
                signal.signal(signal.SIGINT, old)
        dt = time.monotonic() - t0
        self.last = dict(why=why, stopped=stopped, sec=dt)
        if stopped:
            print(f'[정지] 섰다 (명령부터 정지 확인까지 {dt:.2f} s)')
        else:
            print(f'\n{ALARM}\n[정지] 로봇이 아직 움직이거나 정지를 확인하지 못했다 -> 펜던트 비상정지!\n{ALARM}\n')
        return stopped
