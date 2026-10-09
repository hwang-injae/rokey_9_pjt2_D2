'use client';
// 연결 표시 — 화면 ↔ backend(/ws) · backend ↔ 브로커 · 로봇 PC 다리(연결 신호 3초, IRD 10.3). 로봇 PC 가 끊기면 큰 배너.
import type { Robot } from '@/lib/ws';

function Dot({ ok, label }: { ok: boolean; label: string }) {
  return (
    <span className={`dot ${ok ? 'ok' : 'bad'}`} title={ok ? '연결됨' : '끊김'}>
      ● {label}
    </span>
  );
}

export default function ConnectionBadge({ robot }: { robot: Robot }) {
  const robotPcLost = robot.ws && robot.broker && !robot.bridge;
  return (
    <>
      <div className="badges">
        <Dot ok={robot.ws} label="화면 ↔ 웹 서버" />
        <Dot ok={robot.ws && robot.broker} label="웹 서버 ↔ 브로커" />
        <Dot ok={robot.ws && robot.bridge} label="로봇 PC" />
      </div>
      {!robot.ws && <div className="banner warn">웹 서버(backend) 연결 끊김 — 다시 붙는 중. 정지는 로봇 PC 키 · 펜던트로</div>}
      {robot.ws && !robot.broker && <div className="banner warn">브로커 연결 끊김 — 웹 PC 의 mosquitto 를 확인하세요</div>}
      {robotPcLost && (
        <div className="banner danger">로봇 PC 연결 끊김 — [출발] · [스캔]을 막았어요. 정지는 로봇 PC 키 · 펜던트로</div>
      )}
    </>
  );
}
