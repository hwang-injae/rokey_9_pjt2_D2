'use client';
// 로봇 패널 — 상태 줄 · 정지 이유 · [설계 선택] · 버튼 · 그리퍼 (web/README 1 · 3장, W047). 진행도는 위쪽 가운데(ProgressTop).
// 버튼 하나 = REST 하나(lib/api). 판단은 로봇 쪽이 한다 — 여기서는 상태표대로 켜고 끄기만 한다. 자동 재전송 없음.
// 설계는 오른쪽 설계 3D 칸의 [설계 고르기](열 보기)에서 고르면 3D 로 '보기'만 하고, 로봇에 가는 것은 여기 [설계 선택]뿐.
// [출발]은 로봇에 선택된 설계로 간다(미리보기 중인 설계가 아님).
import { useState } from 'react';
import * as api from '@/lib/api';
import type { CmdResult } from '@/lib/types';
import type { Robot } from '@/lib/ws';

const STATE_KO: Record<string, string> = {
  IDLE: '대기', READY: '출발 대기', CHECK: '관측 중', SELECT: '다음 블록 고르는 중', PICK_PLACE: '집고 놓는 중',
  VERIFY: '놓은 블록 확인 중', WAIT_SUPPLY: '공급 기다림', WAIT_HMI: '웹 연결 기다림', SCAN_MOVE: '스캔 — 이동',
  SCAN_CAPTURE: '스캔 — 촬영', SCAN_INFER: '스캔 — 계산', SCAN_REVIEW: '스캔 결과 확인', STOPPED: '멈춤',
  RECOVER: '다시 시작 중', ERROR: '오류', DONE: '완성',
};

/** 정지 이유(IRD 7장) → 누가 · 왜 */
function stopReason(reason: string): string {
  if (reason.startsWith('ROBOT_ALARM:')) return `로봇 알람(${reason.slice('ROBOT_ALARM:'.length)})`;
  const ko: Record<string, string> = {
    STOP_WEB: '화면 정지 버튼', STOP_KEY: '로봇 PC 키', STOP_TASK: '작업 관리자', TIMEOUT: '시간 초과 — 작업 관리자가 먼저 세움',
    CTRL_C: '프로그램 종료(Ctrl+C)',
  };
  return ko[reason] ?? (reason || '이유 모름');
}

const has = (list: string[], s?: string) => !!s && list.includes(s);

interface Props {
  robot: Robot;
  onLog: (kind: string, text: string) => void;
  picked: string | null; // 열 보기에서 눌러 3D 로 보는 설계(아직 로봇에 안 보냈을 수 있음)
}

export default function RobotPanel({ robot, onLog, picked }: Props) {
  const st = robot.state?.state;
  const locked = !!robot.safety?.locked;
  const robotDesign = robot.state?.design_id ?? null;
  const target = picked ?? robotDesign ?? ''; // [설계 선택]이 보낼 설계
  const [busy, setBusy] = useState<string | null>(null); // 보내는 중인 버튼(정지는 따로 — 늘 누를 수 있게)
  const [stopping, setStopping] = useState(false);
  const [last, setLast] = useState<{ label: string; r: CmdResult } | null>(null);

  async function send(label: string, call: () => Promise<CmdResult>, isStop = false): Promise<CmdResult> {
    isStop ? setStopping(true) : setBusy(label);
    const r = await call();
    isStop ? setStopping(false) : setBusy(null); // 응답이나 시간 초과 뒤 바로 푼다 — '보내는 중'에 머물지 않게
    setLast({ label, r });
    onLog('button', `${label} → ${r.success ? '받음' : '거절'}${r.reason ? ' ' + r.reason : ''}${r.message ? ' — ' + r.message : ''}`);
    return r;
  }


  // 상태표(web/README 3장, SDD 5.1). 로봇 PC 가 끊기면 출발 · 계속 · 스캔은 막는다(IRD 10.3)
  const bridge = robot.bridge;
  const can = {
    select: !busy && !locked && has(['IDLE', 'READY', 'DONE'], st),
    start: !busy && !locked && bridge && st === 'READY',
    cont: !busy && !locked && bridge && has(['WAIT_SUPPLY', 'WAIT_HMI'], st),
    scan: !busy && !locked && bridge && has(['IDLE', 'DONE', 'SCAN_REVIEW'], st),
    cancel: !busy && !locked && has(['READY', 'ERROR', 'SCAN_REVIEW'], st),
    resume: !busy && (locked || has(['STOPPED', 'ERROR'], st)),
  };

  const width = robot.gripper?.width_m;

  return (
    <section className="panel">
      <div className={`status ${locked ? 'locked' : st === 'ERROR' ? 'error' : ''}`}>
        <div className="status-main">
          <span className="state-name">{st ? STATE_KO[st] ?? st : '작업 관리자 상태를 기다리는 중'}</span>
          {st && <span className="state-code">{st}</span>}
          {st && !robot.bridge && <span className="state-stale">로봇 PC 끊김 — 마지막으로 받은 상태</span>}
        </div>
        {robot.state?.message && <div className="status-msg">{robot.state.message}</div>}
        {locked && <div className="status-stop">멈춘 이유: {stopReason(robot.safety!.reason)}</div>}
        {robot.state?.design_id && <div className="status-sub">설계 {robot.state.design_id}{robot.state.run_id ? ` · 실행 ${robot.state.run_id}` : ''}</div>}
      </div>

      <div className="design-row">
        <div className="design-now">
          <span className="muted">설계</span>
          <strong>{target || '아직 안 고름'}</strong>
          {target && (target === robotDesign ? <span className="tag ok">로봇에 선택됨</span> : <span className="tag">미리보기만</span>)}
        </div>
        <div className="btn-col">
          <button disabled={!can.select || !target} onClick={() => send('설계 선택', () => api.command('select_design', target))}>
            {busy === '설계 선택' ? '보내는 중…' : '설계 선택'}
          </button>
          <small>오른쪽 설계 3D 의 [설계 고르기]에서 고른 설계를 로봇에 보내요</small>
        </div>
      </div>

      <div className="buttons">
        <div className="btn-col">
          <button className="primary" disabled={!can.start} onClick={() => send('출발', () => api.command('start', robotDesign ?? ''))}>
            {busy === '출발' ? '보내는 중…' : '출발'}
          </button>
          <small>사람이 로봇 작업 영역 밖인지 확인하고 누르세요</small>
        </div>
        <button disabled={!can.cont} onClick={() => send('계속', () => api.command('start', robotDesign ?? ''))}>
          {busy === '계속' ? '보내는 중…' : '계속'}
        </button>
        <button disabled={!can.scan} onClick={() => send('스캔', () => api.command('scan'))}>
          {busy === '스캔' ? '보내는 중…' : st === 'SCAN_REVIEW' ? '다시 스캔' : '스캔'}
        </button>
        <button disabled={!can.cancel} onClick={() => send('취소', () => api.command('cancel'))}>
          {busy === '취소' ? '보내는 중…' : '취소'}
        </button>
      </div>

      <div className="buttons safety">
        <button className="stop" disabled={stopping} onClick={() => send('정지', api.stop, true)}>
          {stopping ? '보내는 중…' : '정지'}
        </button>
        <div className="btn-col">
          <button className="resume" disabled={!can.resume} onClick={() => send('다시 시작', api.resume)}>
            {busy === '다시 시작' ? '보내는 중…' : '다시 시작'}
          </button>
          <small>로봇 작업 영역에서 손을 빼고 누르세요. 조립 중이었으면 놓인 블록은 건너뛰고 이어서 해요</small>
        </div>
      </div>

      {last && (
        <div className={`result ${last.r.success ? 'ok' : 'bad'}`}>
          {last.label}: {last.r.success ? '받았어요' : '거절'}
          {last.r.reason ? ` (${last.r.reason})` : ''}
          {last.r.message ? ` — ${last.r.message}` : ''}
          {!last.r.success && !last.r.reason && !last.r.message ? ' — 상태 줄의 안내를 보세요' : ''}
        </div>
      )}

      <div className="status-sub">
        그리퍼: {robot.gripper ? `${width == null ? '폭 모름' : `폭 ${(width * 1000).toFixed(1)} mm`} · ${robot.gripper.grasped ? '잡음' : '안 잡음'}` : '신호 없음'}
      </div>
    </section>
  );
}
