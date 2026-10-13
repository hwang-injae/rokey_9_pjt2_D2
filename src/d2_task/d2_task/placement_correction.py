"""W144 실제 중심 오차 판정과 재배치 예산 — ROS 없이 계산한다."""

import math


class PlacementCorrection:
    """축별 오차와 목표별 재배치 횟수를 관리한다. 로봇 호출·관측 추정은 하지 않는다."""

    def __init__(self, tolerance_m, max_attempts):
        """입력: 허용 오차(m), 재배치 최대 횟수. 잘못된 설정은 ValueError, 외부 영향 없음."""
        if (isinstance(tolerance_m, bool) or not isinstance(tolerance_m, (int, float))
                or not math.isfinite(tolerance_m) or tolerance_m <= 0):
            raise ValueError('배치 허용 오차는 유한한 양수(m)여야 한다')
        if isinstance(max_attempts, bool) or not isinstance(max_attempts, int) or max_attempts <= 0:
            raise ValueError('재배치 횟수는 양의 정수여야 한다')
        self.tolerance_m = tolerance_m
        self.max_attempts = max_attempts
        self._attempts = {}

    def reset_run(self):
        """새 조립 실행에서만 예산을 초기화한다. 반환 없음, 로봇·메시지 영향 없음."""
        self._attempts.clear()

    def judge(self, block_id, row):
        """측정 행(state, dx/dy/dz_m) → PASS/UNMEASURED/CORRECT/HUMAN. 예산은 소비하지 않는다.

        present와 유한한 세 축이 모두 있어야 판정한다. 부재·가림·NaN은 재집기 입력으로 쓰지 않는다.
        3 mm 등 경계와 정확히 같은 값은 통과하며, 회전·받침·충돌 검사는 호출자가 별도로 수행한다.
        실패: 잘못된 측정은 UNMEASURED. 외부 영향 없음.
        """
        self._validate_id(block_id)
        if not isinstance(row, dict) or row.get('state') != 'present':
            return 'UNMEASURED'
        values = [row.get(key) for key in ('dx_m', 'dy_m', 'dz_m')]
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in values):
            return 'UNMEASURED'
        if all(abs(value) <= self.tolerance_m for value in values):
            return 'PASS'
        return 'HUMAN' if self._attempts.get(block_id, 0) >= self.max_attempts else 'CORRECT'

    def assess(self, part_id, target_center_m, observation):
        """검증된 관측 중심/축별 불확실성 m → 판정과 실제 목표 오차. 예산/로봇 영향 없음.

        관측은 base_link · present · 신원 확인 · 정확도 검증이 필요하다. 미검증 탐색값은 거절한다.
        오차 범위 전체가 3mm 안이면 PASS, 한 축이라도 범위 전체가 밖이면 CORRECT/HUMAN,
        문턱을 걸치면 UNMEASURED. 잘못된 위치/불확실성은 UNMEASURED로 반환한다.
        """
        self._validate_id(part_id)
        fail={'decision':'UNMEASURED','reason':'UNVERIFIED_OBSERVATION','delta_m':None}
        if (not isinstance(observation,dict) or observation.get('state')!='present'
                or observation.get('frame')!='base_link'
                or observation.get('part_id')!=part_id
                or observation.get('identity_confirmed') is not True
                or observation.get('accuracy_validated') is not True):
            return fail
        center=observation.get('center_m');uncertainty=observation.get('uncertainty_m')
        if not all(self._finite_vector(v,3) for v in (center,target_center_m,uncertainty)):
            return dict(fail,reason='INVALID_COORDINATE_OR_UNCERTAINTY')
        if any(u<0 for u in uncertainty):
            return dict(fail,reason='INVALID_COORDINATE_OR_UNCERTAINTY')
        delta=[a-b for a,b in zip(center,target_center_m)]
        if not all(math.isfinite(v) for v in delta):
            return dict(fail,reason='INVALID_COORDINATE_OR_UNCERTAINTY')
        if all(abs(d)+u<=self.tolerance_m for d,u in zip(delta,uncertainty)):
            return dict(decision='PASS',reason='WITHIN_TOLERANCE',delta_m=delta)
        if not any(abs(d)-u>self.tolerance_m for d,u in zip(delta,uncertainty)):
            return dict(decision='UNMEASURED',reason='THRESHOLD_UNCERTAIN',delta_m=delta)
        decision='HUMAN' if self._attempts.get(part_id,0)>=self.max_attempts else 'CORRECT'
        return dict(decision=decision,reason='RETRY_LIMIT' if decision=='HUMAN' else 'OFFSET_OVER',delta_m=delta)

    def prepare_goal(self, part_id, target_pose, observation, grasp, open_width_m):
        """실측 자세와 안전 확인으로 내부 재배치안을 만든다. 요청/예산 소비 없음, 누락이면 거절.

        target_pose=(중심m,xyzw), observation은 assess 정보와 실측 quaternion_xyzw,
        pose_confirmed/support_confirmed/scene_clear/regrasp_allowed를 가진다. 반환은 판정과 goal.
        호출자는 정지 잠금 안에서 신선도·안전·정지를 재확인하고 reserve 성공 뒤에만 요청한다.
        """
        if not isinstance(target_pose,(list,tuple)) or len(target_pose)!=2:
            return dict(decision='UNMEASURED',reason='INVALID_TARGET_POSE',goal=None)
        center,quaternion=target_pose
        assessment=self.assess(part_id,center,observation)
        if assessment['decision']!='CORRECT':return dict(assessment,goal=None)
        if not self._quaternion(quaternion):
            return dict(assessment,decision='UNMEASURED',reason='INVALID_TARGET_POSE',goal=None)
        actual_q=observation.get('quaternion_xyzw')
        if not self._quaternion(actual_q) or observation.get('pose_confirmed') is not True:
            return dict(assessment,decision='UNMEASURED',reason='ACTUAL_POSE_UNKNOWN',goal=None)
        if any(observation.get(k) is not True for k in ('support_confirmed','scene_clear','regrasp_allowed')):
            return dict(assessment,decision='HUMAN',reason='REGRASP_UNSAFE',goal=None)
        obstacles=observation.get('obstacles')
        if not isinstance(obstacles,list) or any(
                not isinstance(p,(list,tuple)) or len(p)!=2
                or not self._finite_vector(p[0],3) or not self._quaternion(p[1]) for p in obstacles):
            return dict(assessment,decision='UNMEASURED',reason='OBSTACLES_UNKNOWN',goal=None)
        if (not isinstance(grasp,str) or grasp not in ('FLAT_SHORT','FLAT_LONG')
                or isinstance(open_width_m,bool) or not isinstance(open_width_m,(int,float))
                or not math.isfinite(open_width_m) or open_width_m<0):
            return dict(assessment,decision='UNMEASURED',reason='INVALID_GRASP',goal=None)
        # 현재 내부 task IO 계약은 block_id다. PLM 뒤 어댑터에서 part_id로 연결한다.
        goal=dict(block_id=part_id,supply_slot='',grasp=grasp,open_width_m=open_width_m,
                  pick_pose=(list(observation['center_m']),list(actual_q)),
                  place_pose=(list(center),list(quaternion)),
                  obstacles=[(list(p[0]),list(p[1])) for p in obstacles])
        return dict(assessment,goal=goal)

    @staticmethod
    def _finite_vector(value, length):
        """지정 길이의 유한한 수열이면 True. bool/누락/NaN은 False, 외부 영향 없음."""
        return (isinstance(value,(list,tuple)) and len(value)==length
                and all(not isinstance(v,bool) and isinstance(v,(int,float)) and math.isfinite(v) for v in value))

    @classmethod
    def _quaternion(cls, value):
        """단위 xyzw이면 True. 형식 오류는 False, 외부 영향 없음."""
        return cls._finite_vector(value,4) and math.isclose(sum(v*v for v in value),1,abs_tol=1e-3)

    def reserve(self, block_id):
        """재배치 요청 직전 예산 1회 소비 → 성공 bool. 호출자는 정지 잠금 안에서 호출한다.

        관측·계획 계산에는 소비하지 않고, 요청 뒤 실패·정지에도 환불하지 않아 무한 재시도를 막는다.
        로봇 호출 없음. 예산이 없으면 False, block_id가 비었으면 ValueError.
        """
        self._validate_id(block_id)
        count = self._attempts.get(block_id, 0)
        if count >= self.max_attempts:
            return False
        self._attempts[block_id] = count + 1
        return True

    @staticmethod
    def _validate_id(block_id):
        """목표 이름이 비었거나 문자열이 아니면 ValueError. 로봇·메시지 영향 없음."""
        if not isinstance(block_id, str) or not block_id.strip():
            raise ValueError('재배치 block_id가 필요하다')
