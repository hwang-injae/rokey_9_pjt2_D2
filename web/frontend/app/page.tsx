'use client';
// 메인 페이지(W047 최소형) — 연결 표시 · 로봇 패널 · 상태 로그. 설계 목록 · 3D 미리보기 · 생성(W112)과 스캔 비교(W116)는 뒤에 붙는다.
import ConnectionBadge from '@/components/ConnectionBadge';
import RobotPanel from '@/components/RobotPanel';
import { useRobot } from '@/lib/ws';

export default function Home() {
  const [robot, addLog] = useRobot();
  return (
    <main>
      <header>
        <h1>D2 젠가 가구 — 로봇 조립</h1>
        <ConnectionBadge robot={robot} />
      </header>

      <RobotPanel robot={robot} onLog={addLog} />

      {robot.state?.state === 'SCAN_REVIEW' && robot.scan && (
        <section className="panel">
          <h2>스캔 결과</h2>
          <p>
            블록 {robot.scan.blocks.blocks.length}개(가려져 추정한 블록 {robot.scan.inferred_count}개). 비교 화면 · [그대로 저장] · [AI로 고치기]는 스캔 비교
            페이지(W116)에서 붙습니다. 지금은 [다시 스캔] · [취소]만 됩니다.
          </p>
        </section>
      )}

      <section className="panel">
        <h2>상태 로그</h2>
        <ol className="log">
          {robot.log.map((l, i) => (
            <li key={`${l.t}-${i}`} className={`log-${l.kind}`}>
              <time>{new Date(l.t).toTimeString().slice(0, 8)}</time> {l.text}
            </li>
          ))}
          {robot.log.length === 0 && <li className="muted">아직 없음</li>}
        </ol>
      </section>
    </main>
  );
}
