'use client';
// 위쪽 가운데 진행도 — 놓은 블록 / 전체 = %, DONE 이면 100 %(web/README 1장 ⑧). 진행표가 다른 설계 것이면 보이지 않는다.
import type { Robot } from '@/lib/ws';

export default function ProgressTop({ robot }: { robot: Robot }) {
  const st = robot.state;
  const p = robot.progress && robot.progress.design_id === st?.design_id ? robot.progress : null;
  const total = p?.blocks.length ?? 0;
  const present = p?.blocks.filter((b) => b.state === 'present').length ?? 0;
  const pct = st?.state === 'DONE' && total ? 100 : total ? Math.round((present / total) * 100) : 0;

  return (
    <div className="top-progress" aria-label="진행도">
      <div className="top-progress-head">
        <span className="muted">{st?.design_id ?? '설계 없음'}</span>
        <strong>{total ? `${present} / ${total} 블록 · ${pct}%` : '진행표 없음'}</strong>
      </div>
      <div className="bar">
        <div className="fill" style={{ width: `${pct}%` }} />
      </div>
      <div className="top-progress-sub muted">{st?.block_id ? `지금 블록 ${st.block_id}` : ' '}</div>
    </div>
  );
}
