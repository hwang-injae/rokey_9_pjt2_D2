'use client';
// 로봇 패널 — 상태 · 로봇 설계 · 버튼 · 그리퍼 (web/README 1 · 3장, W047). 진행도는 위쪽 가운데(ProgressTop).
// 버튼 하나 = REST 하나(lib/api). 판단은 로봇 쪽이 한다 — 여기서는 상태표대로 보이고 켜고 끄기만 한다. 자동 재전송 없음.
// 조립 시작은 오른쪽 설계 3D 의 [조립] 하나(보는 곳에서 보냄 — select_design + start, 10/13 PL). 여기에는 [출발]이 없다.
// 그때만 쓰는 버튼은 그때만 보인다(10/10 황인재): [계속] = 공급 · 웹 연결을 기다릴 때, [다시 시작] = 멈췄을 때. 긴 설명은 title(마우스).
// 멈춘 뒤에는 [정지] 자리가 [초기화](W183 · W184 — 10/13 16시 PL 확정): 한 번 더 확인 → cancel(작업 관리자가 잠긴 채 이번 조립 기록을 닫음,
// STOPPED 유지) → '블록을 치워 주세요' 알림 — 치우는 동안 잠금 유지 → [정리 끝] → resume(→ RECOVER → IDLE, 로봇 안 움직임).
// cancel 이 거절되면 잠금을 풀지 않는다(그대로 풀면 조립을 이어 가므로). [그리퍼 열기] = 잠김 + 그리퍼가 쥐고 있을 때만 — 한 사람이
// 블록을 받쳤는지 확인한 뒤 cmd gripper_open(작업 관리자가 STOPPED 일 때만 연다). 여는 동안(state/2 message_id gripper_opening — 10/13 18시
// PL)은 조립 · 스캔 · 다시 열기를 막는다(lib/ws gripperOpening — 새로고침해도 retained 로 그대로).
import { useState } from 'react';
import * as api from '@/lib/api';
import { designName, stateName, stopReasonKo } from '@/lib/names';
import type { CmdResult } from '@/lib/types';
import { gripperOpening, type Robot } from '@/lib/ws';

const has = (list: string[], s?: string) => !!s && list.includes(s);

interface Props {
  robot: Robot;
  onLog: (kind: string, text: string) => void;
}

export default function RobotPanel({ robot, onLog }: Props) {
  const st = robot.state?.state;
  const locked = !!robot.safety?.locked;
  const robotDesign = robot.state?.design_id ?? null;
  const [busy, setBusy] = useState<string | null>(null); // 보내는 중인 버튼(정지는 따로 — 늘 누를 수 있게)
  const [stopping, setStopping] = useState(false);
  const [last, setLast] = useState<{ label: string; r: CmdResult } | null>(null);
  const [asking, setAsking] = useState(false); // [초기화] 확인 한 번 더(되돌릴 수 없어서)
  const [clearNotice, setClearNotice] = useState(false); // 초기화 뒤 '블록을 치워 주세요' 알림(잠금 유지 — [정리 끝]까지)
  const [noticeErr, setNoticeErr] = useState('');
  const [askOpen, setAskOpen] = useState(false); // [그리퍼 열기] 확인 — 블록이 떨어지지 않게 사람이 받쳤나

  async function send(label: string, call: () => Promise<CmdResult>, isStop = false): Promise<CmdResult> {
    isStop ? setStopping(true) : setBusy(label);
    const r = await call();
    isStop ? setStopping(false) : setBusy(null); // 응답이나 시간 초과 뒤 바로 푼다 — '보내는 중'에 머물지 않게
    setLast({ label, r });
    onLog('button', `${label} → ${r.success ? '받음' : '거절'}${r.reason ? ' ' + r.reason : ''}${r.message ? ' — ' + r.message : ''}`);
    return r;
  }

  // 상태표(web/README 3장, SDD 5.1). 로봇 PC 가 끊기면 계속 · 스캔은 막는다(IRD 10.3)
  const bridge = robot.bridge;
  const opening = gripperOpening(robot); // 그리퍼 여는 중 — 조립 · 스캔 · 다시 열기를 막는다(10/13 18시 PL)
  const waiting = has(['WAIT_SUPPLY', 'WAIT_HMI'], st);
  const can = {
    cont: !busy && !locked && bridge && waiting,
    scan: !busy && !locked && !opening && bridge && has(['IDLE', 'DONE', 'SCAN_REVIEW'], st),
    cancel: !busy && !locked && has(['READY', 'ERROR', 'SCAN_REVIEW'], st),
  };
  const showResume = locked || has(['STOPPED', 'ERROR'], st);
  const stopped = locked || st === 'STOPPED';
  const label = (name: string, text: string) => (busy === name ? '보내는 중…' : text);

  const grasped = !!robot.gripper?.grasped;

  /** [초기화]: cancel → 받으면 '블록을 치워 주세요'(잠금 유지). 바깥 영향: 이번 조립 기록을 닫는다(로봇은 안 움직임) */
  async function reset() {
    setAsking(false);
    const r = await send('초기화', () => api.command('cancel'));
    if (r.success) {
      setNoticeErr('');
      setClearNotice(true);
    }
  }
  /** [정리 끝]: 그때 잠금을 푼다 → 작업 관리자 RECOVER → IDLE(조립 중이 아니게 됐으므로 이어 가지 않음) */
  async function cleared() {
    const u = await send('정리 끝 — 잠금 풀기', api.resume);
    if (u.success) setClearNotice(false);
    else setNoticeErr(`잠금을 못 풀었어요 — ${u.message ?? u.reason}`);
  }
  /** [그리퍼 열기]: 사람이 블록을 받친 뒤 — cmd gripper_open(작업 관리자가 STOPPED 일 때만 연다). 바깥 영향: 그리퍼가 열림 */
  async function openGripper() {
    setAskOpen(false);
    const r = await send('그리퍼 열기', () => api.command('gripper_open'));
    if (!r.success) setNoticeErr(`그리퍼를 못 열었어요 — ${r.message ?? r.reason}`);
  }
  const gripperButton = locked && grasped && (
    <button className="warn" disabled={!!busy || opening} onClick={() => setAskOpen(true)} title="정지 중에 그리퍼가 쥔 블록을 빼요 — 한 사람이 블록을 받치고 누르세요">
      {opening ? '그리퍼 여는 중…' : label('그리퍼 열기', '그리퍼 열기')}
    </button>
  );
  const width = robot.gripper?.width_m;

  return (
    <section className="panel robot-panel">
      <div className={`status ${locked ? 'locked' : st === 'ERROR' ? 'error' : st === 'DONE' ? 'done' : waiting ? 'wait' : ''}`}>
        <div className="status-title">
          {st ? stateName(st) : '로봇 상태를 기다리는 중'}
          {st && !bridge && <span className="tag">마지막으로 받은 상태</span>}
        </div>
        {locked ? (
          <div className="status-msg">{stopReasonKo(robot.safety!.reason)}. 원인을 없앤 뒤 [다시 시작]을 누르세요.</div>
        ) : (
          robot.state?.message && <div className="status-msg">{robot.state.message}</div>
        )}
      </div>

      <div className="kv">
        <span className="muted">로봇 설계</span>
        <strong>{robotDesign ? designName(robotDesign) : '아직 없음'}</strong>
      </div>
      {!robotDesign && <p className="hint">오른쪽 [설계 목록]에서 고르고 [조립]을 누르세요</p>}

      <div className="btn-stack">
        {waiting && (
          <button className="warn big" disabled={!can.cont}
            title={st === 'WAIT_SUPPLY' ? '공급 영역에 블록을 채운 뒤 누르세요' : '웹이 다시 붙었으면 누르세요 — 진행 확인부터 이어 가요'}
            onClick={() => send('계속', () => api.command('start', robotDesign ?? ''))}>
            {label('계속', st === 'WAIT_SUPPLY' ? '블록 채웠어요 · 계속' : '계속')}
          </button>
        )}
        <div className="btn-row">
          <button className="secondary" disabled={!can.scan} title="사람이 쌓은 구조를 찍어 설계도로 만들어요"
            onClick={() => send('스캔', () => api.command('scan'))}>
            {label('스캔', st === 'SCAN_REVIEW' ? '다시 스캔' : '스캔')}
          </button>
          <button className="secondary" disabled={!can.cancel} title="출발 대기 · 오류 · 스캔 확인을 그만두고 대기로"
            onClick={() => send('취소', () => api.command('cancel'))}>
            {label('취소', '취소')}
          </button>
        </div>
      </div>

      <div className="btn-row safety">
        {stopped ? (
          <button className="reset" disabled={!!busy} onClick={() => setAsking(true)}
            title="이번 조립을 버리고 처음(대기)으로 — 로봇은 움직이지 않아요">
            {label('초기화', '초기화')}
          </button>
        ) : (
          <button className="stop" disabled={stopping} onClick={() => send('정지', api.stop, true)}>
            {stopping ? '보내는 중…' : '정지'}
          </button>
        )}
        {showResume && (
          <button className="resume" disabled={!!busy} title="로봇 작업 영역에서 손을 빼고 누르세요 — 놓인 블록은 건너뛰고 이어서 해요"
            onClick={() => send('다시 시작', api.resume)}>
            {label('다시 시작', '다시 시작')}
          </button>
        )}
      </div>

      {gripperButton && !clearNotice && <div className="btn-row">{gripperButton}</div>}

      {asking && (
        <div className="confirm" role="alertdialog" aria-label="초기화 확인">
          <p>이번 조립을 버리고 처음(대기)으로 돌아갈까요? 놓은 블록은 사람이 치워야 해요.</p>
          <div className="btn-row">
            <button className="reset" onClick={reset}>초기화</button>
            <button className="secondary" onClick={() => setAsking(false)}>아니요</button>
          </div>
        </div>
      )}

      {/* 창은 하나만 — [초기화] 알림 안에서 [그리퍼 열기]를 누르면 같은 창이 확인 질문으로 바뀌고, 답하면 알림으로 돌아온다.
          창 두 개를 겹치면 뒤에 연 질문이 알림 뒤에 가려져 안 보였다(10/13 황인재 확인). 기본 초점은 [아니요] — Enter 한 번에 블록이 떨어지지 않게 */}
      {(askOpen || clearNotice) && (
        <div className="modal-back" role="presentation">
          <div className="modal" role="alertdialog" aria-modal="true" aria-labelledby={askOpen ? 'grip-title' : 'clear-title'}>
            {askOpen ? (
              <>
                <h2 id="grip-title">블록을 손으로 받쳤나요?</h2>
                <p>그리퍼를 열면 쥔 블록이 떨어져요. 한 사람이 블록을 받친 뒤 여세요.</p>
                <div className="btn-row">
                  <button className="warn" onClick={openGripper}>받쳤어요 · 열기</button>
                  <button className="secondary" autoFocus onClick={() => setAskOpen(false)}>아니요</button>
                </div>
              </>
            ) : (
              <>
                <h2 id="clear-title">작업대의 블록을 치워 주세요</h2>
                <p>이번 조립을 버렸어요. 로봇은 정지(잠금) 상태 그대로예요 — 작업대(조립 자리) 위 블록을 모두 치워 주세요.</p>
                <p className="muted">
                  그리퍼에 블록이 있으면 한 사람이 받치고 [그리퍼 열기]를 누르세요. 다 치우면 [정리 끝] — 그때 잠금이 풀리고 대기로 가요.
                </p>
                {noticeErr && <p className="result bad">{noticeErr}</p>}
                <div className="btn-row">
                  {gripperButton}
                  <button className="primary big" autoFocus disabled={!!busy} onClick={cleared}>
                    {label('정리 끝 — 잠금 풀기', '정리 끝')}
                  </button>
                </div>
              </>
            )}
          </div>
        </div>
      )}

      {last && !last.r.success && (
        <div className="result bad" title={[last.r.reason, last.r.message].filter(Boolean).join(' — ')}>
          {last.label} 안 됨{last.r.message ? ` — ${last.r.message}` : ' — 위 상태 안내를 보세요'}
        </div>
      )}

      <div className="foot muted">
        그리퍼 {opening ? '여는 중… · ' : ''}{robot.gripper ? `${robot.gripper.grasped ? '잡음' : '안 잡음'}${width == null ? '' : ` · ${(width * 1000).toFixed(0)} mm`}` : '신호 없음'}
      </div>
    </section>
  );
}
