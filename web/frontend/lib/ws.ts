'use client';
// /ws 받기 한 곳 — 붙으면 backend 가 들고 있는 값이 먼저 오고, 그 뒤 바뀔 때마다 온다. 끊기면 2초마다 다시 붙는다.
import { useEffect, useRef, useState } from 'react';
import { BASE, setReqTimeout } from './api';
import type { GripperState, Intent, Progress, SafetyState, ScanResult, TaskState, WsEvent } from './types';

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
  log: LogLine[];
}

const EMPTY: Robot = { ws: false, broker: false, bridge: false, state: null, safety: null, progress: null, gripper: null, scan: null, intent: null, wrist: null, log: [] };

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
  const log = (text: string) => [{ t: Date.now(), kind: e.type, text }, ...r.log].slice(0, LOG_MAX);
  switch (e.type) {
    case 'state': {
      const s = e.data;
      const same = r.state && r.state.state === s.state && r.state.message === s.message;
      return { ...r, state: s, log: same ? r.log : log(`${s.state}${s.message ? ' — ' + s.message : ''}`) };
    }
    case 'safety': {
      const s = e.data;
      const same = r.safety && r.safety.locked === s.locked && r.safety.reason === s.reason;
      return { ...r, safety: s, log: same ? r.log : log(s.locked ? `멈춤 — ${s.reason}` : '정지 풀림') };
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
      return { ...r, bridge: e.data.alive, log: r.bridge === e.data.alive ? r.log : log(e.data.alive ? '로봇 PC 연결됨' : '로봇 PC 연결 끊김') };
    case 'broker':
      return { ...r, broker: e.data.connected };
    case 'wrist_image': // 그림 자체는 /api/robot/wrist.jpg 로 받는다 — 로그에는 남기지 않음(검출 때마다 와서)
      return { ...r, wrist: { seq: e.data.seq, at: e.data.stamp * 1000 } };
    default:
      return r;
  }
}
