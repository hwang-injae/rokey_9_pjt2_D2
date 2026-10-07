# -*- coding: utf-8 -*-
"""결과 저장 요청 관리 BuildSender — TaskManager 의 미전송 요약을 save_build 로 보내고 요청 하나하나를 구분해 추적한다 (ROS 없이 동작, W119 ①).

task 노드가 타이머로 poll() 을 부르고, 서비스 호출 · 취소 · 준비 확인은 노드가 함수로 넘긴다(그래서 가짜로 시험할 수 있다).
요청은 run_id 가 아니라 **future 객체**로 구분한다: 시간 초과로 포기한 요청의 늦은 콜백이 같은 run_id 로 다시 보낸 요청의 추적을 지우지 못하게.
바깥 영향: call() 로 서비스 요청을 보낸다. 기다리지 않는다. 실패 · 늦은 답은 요약을 남기는 쪽으로만 움직인다.
"""
import threading
import time


class BuildSender:
    """미전송 build/1 요약을 보내고 답을 TaskManager 에 넘긴다.

    입력: manager(builds_to_send · build_result · build_failed 를 가진 TaskManager), call(summary dict) → future,
    cancel(future), ready() → 서버가 떠 있나, service_s(제한 시간 = 다시 보내기 간격, robot.yaml timeout.service_s), clock(시험용).
    """

    def __init__(self, manager, call, cancel, ready, service_s, clock=time.monotonic):
        """요청 추적표를 비워 둔다."""
        self.manager, self._call, self._cancel, self._ready = manager, call, cancel, ready
        self.service_s, self.clock = service_s, clock
        self._saving = {}                      # run_id → (future, 보낸 시각) — 지금 답을 기다리는 요청
        self._lock = threading.Lock()

    def poll(self):
        """답이 늦은 요청은 포기하고(요약은 남김), 보낼 요약이 있으면 보낸다. 기다리지 않는다. 서버가 안 떠 있으면 보내지 않는다."""
        now = self.clock()
        with self._lock:
            late = [(r, f) for r, (f, t0) in self._saving.items() if now - t0 >= self.service_s]
            for run_id, _ in late:
                del self._saving[run_id]
        for run_id, future in late:
            self._cancel(future)
            self.manager.build_failed(run_id, f'{self.service_s} 초 안에 답이 없다(TIMEOUT)')
        if not self._ready():
            return
        for run_id, summary in self.manager.builds_to_send(now, self.service_s):
            try:
                future = self._call(summary)
            except Exception as e:   # noqa: BLE001 — 못 보내도 요약은 남는다
                self.manager.build_failed(run_id, f'요청을 못 만들었다: {e!r}')
                continue
            with self._lock:
                self._saving[run_id] = (future, now)
            future.add_done_callback(lambda f, r=run_id: self._on_done(r, f))

    def _on_done(self, run_id, future):
        """답이 왔다. 지금 추적 중인 그 요청(같은 future 객체)일 때만 적용한다.

        이미 포기했거나 새 요청으로 바뀐 것의 늦은 성공 · 실패 · 예외는 모두 무시한다 — 현재 요청 추적과 전송 중 표시를 건드리지 않는다.
        """
        with self._lock:
            current = self._saving.get(run_id)
            if current is None or current[0] is not future:
                return
            del self._saving[run_id]
        try:
            res = future.result()
        except Exception as e:   # noqa: BLE001
            self.manager.build_failed(run_id, f'호출 실패: {e!r}')
            return
        self.manager.build_result(run_id, res.success, res.response_json)

    def waiting(self):
        """답을 기다리는 run_id 들(시험 · 점검용)."""
        with self._lock:
            return sorted(self._saving)
