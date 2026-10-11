'use client';
// 연결 표시 — 화면 ↔ backend(/ws) · backend ↔ 브로커 · 로봇 PC 다리(연결 신호 3초, IRD 10.3).
// 점 3개는 위 띠에, 끊김 배너는 그 아래 크게(로봇 PC 가 끊기면 [출발] · [스캔] 막음 — 정지는 키 · 펜던트).
import type { Robot } from '@/lib/ws';

function Dot({ ok, label }: { ok: boolean; label: string }) {
  return (
    <span className={`dot ${ok ? 'ok' : 'bad'}`} title={ok ? '연결됨' : '끊김'}>
      <i /> {label}
    </span>
  );
}

export function ConnectionDots({ robot }: { robot: Robot }) {
  return (
    <div className="badges">
      <Dot ok={robot.ws} label="웹 서버" />
      <Dot ok={robot.ws && robot.broker} label="브로커" />
      <Dot ok={robot.ws && robot.bridge} label="로봇 PC" />
    </div>
  );
}

export function ConnectionBanners({ robot }: { robot: Robot }) {
  const robotPcLost = robot.ws && robot.broker && !robot.bridge;
  return (
    <>
      {!robot.ws && <div className="banner warn">웹 서버(backend) 연결 끊김 — 다시 붙는 중. 정지는 로봇 PC 키 · 펜던트로</div>}
      {robot.ws && !robot.broker && <div className="banner warn">브로커 연결 끊김 — 웹 PC 의 mosquitto 를 확인하세요</div>}
      {robotPcLost && (
        <div className="banner danger">로봇 PC 연결 끊김 — [출발] · [스캔]을 막았어요. 정지는 로봇 PC 키 · 펜던트로</div>
      )}
    </>
  );
}
