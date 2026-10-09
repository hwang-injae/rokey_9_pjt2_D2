'use client';
// 손목 카메라 블록 검출 화면(web/README 1장 — E-67). 손목 비전(YOLO-seg)이 블록을 찾을 때마다 윤곽 · 글자까지 그려 보낸 JPEG 를
// 그대로 보여 준다(화면은 덧그리지 않음). 그림이 오면 /ws 로 번호만 오고, 그림은 /api/robot/wrist.jpg 로 받는다.
import { useEffect, useState } from 'react';
import { wristUrl } from '@/lib/api';
import type { Robot } from '@/lib/ws';

export default function WristCamera({ robot }: { robot: Robot }) {
  const w = robot.wrist;
  const [now, setNow] = useState(() => Date.now());

  useEffect(() => {
    const t = setInterval(() => setNow(Date.now()), 1000); // '몇 초 전' 글을 1초마다 고친다
    return () => clearInterval(t);
  }, []);

  const age = w ? Math.max(0, Math.round((now - w.at) / 1000)) : 0;
  return (
    <section className="panel">
      <div className="panel-head">
        <h2>손목 카메라 — 블록 검출</h2>
        <span className="muted">{w ? `마지막 그림 ${new Date(w.at).toTimeString().slice(0, 8)} · ${age}초 전` : '그림 없음'}</span>
      </div>
      <div className="cam">
        {w ? (
          <img src={wristUrl(w.seq)} alt="손목 카메라 블록 검출 그림" />
        ) : (
          <div className="cam-empty">손목 비전이 흩뿌린 공급 영역에서 블록을 찾을 때(find_blocks)마다 그림이 와요 — 공급 칸 방식(slots)에서는 오지 않아요</div>
        )}
      </div>
    </section>
  );
}
