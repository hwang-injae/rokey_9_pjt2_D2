// REST 한 곳(SDD 3.3 — 화면은 backend 의 REST · /ws 만 쓴다, E-41).
// 운영: backend 가 이 화면을 같은 주소에서 내려 주므로 BASE 는 빈 글자. 개발(next dev :3000): NEXT_PUBLIC_BACKEND=http://localhost:8000
import type { Cmd, CmdResult, Design, DesignSummary, Rules } from './types';

export const BASE = process.env.NEXT_PUBLIC_BACKEND ?? '';
const TIMEOUT_MS = 8000; // backend 가 MQTT 응답을 5초(req_timeout_s)까지 기다린 뒤 답하므로 그보다 길게. 넘으면 실패로 보고 버튼을 푼다

async function post(path: string, body?: unknown): Promise<CmdResult> {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), TIMEOUT_MS);
  try {
    const res = await fetch(BASE + path, {
      method: 'POST',
      headers: body === undefined ? undefined : { 'Content-Type': 'application/json' },
      body: body === undefined ? undefined : JSON.stringify(body),
      signal: ctrl.signal,
    });
    if (!res.ok) return { success: false, reason: 'ERROR', message: `backend 응답 ${res.status}` };
    return (await res.json()) as CmdResult;
  } catch {
    return { success: false, reason: 'ERROR', message: 'backend 에 닿지 않음' };
  } finally {
    clearTimeout(timer);
  }
}

/** /d2/hmi/command — 설계 선택 · 출발([계속]) · 스캔 · 취소. 자동으로 다시 보내지 않는다(IRD 10.1) */
export const command = (cmd: Cmd, designId = '') => post('/api/robot/command', { cmd, design_id: designId, mode: 'auto' });
/** /d2/safety/stop — 화면 정지(reason 비움 → 정지 노드가 STOP_WEB) */
export const stop = () => post('/api/robot/stop');
/** /d2/safety/resume — 다시 시작(잠금 풀기 한 번) */
export const resume = () => post('/api/robot/resume');

async function getJson<T>(path: string): Promise<T | null> {
  try {
    const res = await fetch(BASE + path);
    return res.ok ? ((await res.json()) as T) : null;
  } catch {
    return null;
  }
}

/** 설계 요약 목록(트리용). 못 받으면 null — 화면이 '못 받음'을 보여 준다 */
export const listDesigns = () => getJson<DesignSummary[]>('/api/designs');
/** design/2.0 하나(3D 보기용). 없으면 null */
export const getDesign = (id: string) => getJson<Design>('/api/designs/' + encodeURIComponent(id));
/** 설계 규칙 숫자 — 3D 블록 크기(robot.yaml block_size_m 한 곳, 10/9 PL E-78) */
export const getRules = () => getJson<Rules>('/api/designs/rules');
/** 손목 검출 그림 주소 — seq 가 바뀔 때마다 새로 받는다(브라우저가 옛 그림을 다시 쓰지 않게) */
export const wristUrl = (seq: number) => `${BASE}/api/robot/wrist.jpg?seq=${seq}`;
