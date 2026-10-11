"""설계 읽기 캐시 RunDesignCache — 한 판(run) 안에서는 읽어 둔 설계를 쓰고, run_id 가 바뀌면 다시 읽는다 (ROS 없는 계산, W157 · E-60 ②).

wrist_block · mock_wrist_block 이 같이 쓴다. 기본 설계는 레시피 파일을 고치면 같은 design_id 로 다시 등록될 수 있어서(E-55 ①)
설계를 영원히 캐시하지 않고, 작업 관리자가 CheckProgress.run_id 에 싣는 '조립 한 판의 ID'가 바뀔 때만 다시 읽는다.
'블록 전부를 묻는 요청 = 새 판' 같은 짐작은 하지 않는다 — 판은 run_id 로만 구분한다.
"""


class RunDesignCache:
    """설계 이름(열쇠)마다 '어느 판에서 읽었는지(run_id)'와 읽은 값을 기억한다. 바깥 영향 없음(읽는 일은 부르는 쪽이 넘긴 load 가 한다).

    **스레드 안전하지 않다 — 부르는 노드가 get() 을 한 번에 하나씩만 부르게 해야 한다**(wrist_block · mock_wrist_block 은 check_progress 를
    MutuallyExclusiveCallbackGroup 으로 차례로 처리한다). 겹쳐 부르면 같은 판을 두 번 읽고, 느린 지난 판의 답이 새 판 캐시를 덮을 수 있다.
    잠금을 여기 넣지 않은 이유: load() 가 get_design 응답을 기다리는 동안 잠금을 쥐면 executor 스레드가 모자라 응답이 못 돌아 TIMEOUT 이 난다
    (2026-10-10 비교 실험 — 같은 판 요청 6개를 스레드 4개 실행기에 겹치면 3개가 TIMEOUT)."""

    def __init__(self):
        """빈 캐시를 만든다."""
        self._items = {}          # 열쇠 → (run_id, 읽은 값)

    def get(self, key, run_id, load):
        """열쇠의 설계를 돌려준다. 같은 판(run_id 가 같음)에서 이미 읽었으면 그 값을, 아니면 load() 로 새로 읽는다.

        입력: key(설계 이름 등 아무 열쇠) · run_id(글자 — CheckProgress.run_id. 빈 값 · None 은 '판 구분 없음'이라 모두 이름 없는 한 판으로 본다:
        빈 값끼리는 캐시를 쓰고, 이름 있는 판과 오가면 다시 읽는다) · load() → (값, 실패 코드). 읽기에 실패하면 값은 None.
        출력: (값, '') 또는 (None, 실패 코드). 실패 코드가 비면 'ERROR'.
        바깥 영향: load() 가 하는 일(get_design 요청 · 파일 읽기)뿐. 실패는 캐시하지 않아 다음 요청이 다시 읽는다.
        새 판이어서 다시 읽을 때는 옛 값을 먼저 버린다 — 읽기가 실패해도 지난 판의 설계로 답하지 않는다(E-55 ①).
        """
        run_id = run_id or ''
        item = self._items.get(key)
        if item is not None and item[0] == run_id:
            return item[1], ''
        self._items.pop(key, None)
        value, reason = load()
        if value is None:
            return None, reason or 'ERROR'
        self._items[key] = (run_id, value)
        return value, ''
