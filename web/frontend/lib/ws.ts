'use client';
// /ws 받기 한 곳 — 붙으면 backend 가 들고 있는 값이 먼저 오고, 그 뒤 바뀔 때마다 온다. 끊기면 2초마다 다시 붙는다.
import { useEffect, useRef, useState } from 'react';
import { BASE, setReqTimeout } from './api';
import { stateName, stopReasonKo } from './names';
import type { BlobInfo, GenEvent, GripperState, Intent, Progress, SafetyState, ScanPreview, ScanResult, ScanSaveJob, TaskState, WsEvent } from './types';

const RETRY_MS = 2000;
const LOG_MAX = 100; // 화면 로그 줄 수(오래된 것부터 지움)

export interface LogLine {
  t: number; // 받은 시각(ms, 이 브라우저 시계 — 두 PC 시계를 맞추지 않는다)
  kind: string;
  text: string;
}

export interface Robot {
  ws: boolean; // 브라우저 ↔ backend
  broker: boolean; // backend ↔ 브로커
  bridge: boolean; // 브로커 ↔ 로봇 PC 다리(연결 신호 3초)
  state: TaskState | null;
  safety: SafetyState | null;
  progress: Progress | null;
  gripper: GripperState | null;
  scan: ScanResult | null;
  intent: Intent | null;
  wrist: { seq: number; at: number } | null; // 손목 검출 그림 번호 · 웹 PC 가 받은 시각(ms)
  gen: GenEvent | null; // 마지막 AI 생성 알림 — 생성 패널이 받으면 job 을 다시 받는다(W112)
  scanImage: BlobInfo | null; // 마지막 스캔 사진 번호 · run_id(그림은 /api/robot/scan.jpg)
  scanCloud: BlobInfo | null; // 마지막 스캔 점군 번호 · run_id(PLY 는 /api/robot/scan_cloud.ply)
  scanPreview: { run_id: string; capture?: ScanPreview; live?: ScanPreview } | null; // 스캔 중 점군 미리보기(E-82) — kind 별 마지막 하나
  scanSave: (ScanSaveJob & { stage: string }) | null; // 마지막 스캔 [저장] 알림 — 비교 화면이 받으면 job 을 다시 받는다
  log: LogLine[];
}

const EMPTY: Robot = {
  ws: false, broker: false, bridge: false, state: null, safety: null, progress: null, gripper: null, scan: null, intent: null,
  wrist: null, gen: null, scanImage: null, scanCloud: null, scanPreview: null, scanSave: null, log: [],
};

/** 작업 관리자가 그리퍼를 여는 중인가(state/2 message_id gripper_opening — 10/13 18시 PL, W183 · W184). 이 동안 화면은 조립 · 스캔 ·
 *  다시 열기를 막는다. retained 마지막 state/2 로 판정하므로 새로고침 · 다시 붙어도 그대로이고, gripper_done 이나 그 뒤 다른 값이
 *  오면 풀린다(완료 알림을 놓쳐도). message_id 로만 가르고 문구는 비교하지 않는다. 열렸는지는 gripper/state(잡힘 · 폭)로 따로 본다 */
export const gripperOpening = (r: Robot) => r.state?.message_id === 'gripper_opening';

function wsUrl(): string {
  const base = BASE || window.location.origin;
  return base.replace(/^http/, 'ws') + '/ws';
}

/** 화면 쪽 로봇 상태 하나. addLog 는 버튼 결과를 같은 로그에 남길 때 쓴다 */
export function useRobot(): [Robot, (kind: string, text: string) => void] {
  const [robot, setRobot] = useState<Robot>(EMPTY);
  const closed = useRef(false);

  const addLog = (kind: string, text: string) =>
    setRobot((r) => ({ ...r, log: [{ t: Date.now(), kind, text }, ...r.log].slice(0, LOG_MAX) }));

  useEffect(() => {
    closed.current = false;
    let sock: WebSocket | null = null;
    let timer: ReturnType<typeof setTimeout> | undefined;

    const connect = () => {
      sock = new WebSocket(wsUrl());
      sock.onopen = () => setRobot((r) => ({ ...r, ws: true }));
      sock.onclose = () => {
        setRobot((r) => ({ ...r, ws: false }));
        if (!closed.current) timer = setTimeout(connect, RETRY_MS);
      };
      sock.onmessage = (m) => {
        let e: WsEvent;
        try {
          e = JSON.parse(m.data) as WsEvent;
        } catch {
          return;
        }
        if (e.type === 'timing') setReqTimeout(e.data.req_timeout_s); // 버튼 기다림 시간(lib/api) — 화면 상태는 아님
        setRobot((r) => apply(r, e));
      };
    };
    connect();
    return () => {
      closed.current = true;
      clearTimeout(timer);
      sock?.close();
    };
  }, []);

  return [robot, addLog];
}

/** 이벤트 하나를 상태에 반영하고, 사람이 볼 만한 변화만 로그에 남긴다(같은 상태 · 같은 글이면 안 남김) */
function apply(r: Robot, e: WsEvent): Robot {
  // kind = 기록 글 색(globals.css .log-<kind>). 연결됨 · 정지 풀림 같은 좋은 소식은 'ok' 로 — 빨강은 끊김 · 정지에만
  const log = (text: string, kind: string = e.type) => [{ t: Date.now(), kind, text }, ...r.log].slice(0, LOG_MAX);
  switch (e.type) {
    case 'state': {
      const s = e.data;
      const same = r.state && r.state.state === s.state && r.state.message === s.message;
      return { ...r, state: s, log: same ? r.log : log(`${stateName(s.state)}${s.message ? ' — ' + s.message : ''}`) };
    }
    case 'safety': {
      const s = e.data;
      const same = r.safety && r.safety.locked === s.locked && r.safety.reason === s.reason;
      return { ...r, safety: s, log: same ? r.log : s.locked ? log(stopReasonKo(s.reason)) : log('정지 풀림', 'ok') };
    }
    case 'progress':
      return { ...r, progress: e.data };
    case 'gripper':
      return { ...r, gripper: e.data };
    case 'scan_result':
      return { ...r, scan: e.data, log: log(`스캔 결과 — 블록 ${e.data.blocks.blocks.length}개(추정 ${e.data.inferred_count})`) };
    case 'intent':
      return { ...r, intent: e.data, log: log(`음성 ${e.data.intent}${e.data.text ? ' — ' + e.data.text : ''}`) };
    case 'bridge_alive':
      return { ...r, bridge: e.data.alive, log: r.bridge === e.data.alive ? r.log : e.data.alive ? log('로봇 PC 연결됨', 'ok') : log('로봇 PC 연결 끊김') };
    case 'broker':
      return { ...r, broker: e.data.connected };
    case 'wrist_image': // 그림 자체는 /api/robot/wrist.jpg 로 받는다 — 로그에는 남기지 않음(검출 때마다 와서)
      return { ...r, wrist: { seq: e.data.seq, at: e.data.stamp * 1000 } };
    case 'scan_image': // 사진 · 점군 바이트는 REST 로 받는다 — 여기서는 번호 · 어느 스캔 것인지만
      return { ...r, scanImage: e.data };
    case 'scan_cloud':
      return { ...r, scanCloud: e.data };
    case 'scan_preview': {
      // 새 스캔(run_id)이면 앞 미리보기를 버린다. 같은 kind 는 최신 index 로만 바꾼다(backend 도 거르지만 붙을 때 다시 오는 것까지)
      const p = e.data;
      const cur = r.scanPreview?.run_id === p.run_id ? r.scanPreview : { run_id: p.run_id };
      if ((cur[p.kind]?.index ?? 0) > p.index) return r;
      return { ...r, scanPreview: { ...cur, [p.kind]: p } };
    }
    case 'scan_save': {
      const j = e.data;
      const text = j.stage === 'saved' ? '스캔 설계 저장됨' : j.stage === 'failed' ? `스캔 저장 실패 — ${j.message ?? ''}` : '';
      return { ...r, scanSave: j, log: text ? log(text) : r.log };
    }
    case 'gen': {
      const g = e.data;
      const text = g.stage === 'done' ? 'AI 후보 준비됨 — 고르세요' : g.stage === 'failed' ? `AI 생성 실패 — ${g.message ?? ''}` : g.stage === 'saved' ? '설계 저장됨' : '';
      return { ...r, gen: g, log: text ? log(text) : r.log };
    }
    default:
      return r;
  }
}
