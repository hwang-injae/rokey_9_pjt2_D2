// REST 한 곳(SDD 3.3 — 화면은 backend 의 REST · /ws 만 쓴다, E-41).
// 운영: backend 가 이 화면을 같은 주소에서 내려 주므로 BASE 는 빈 글자. 개발(next dev :3000): NEXT_PUBLIC_BACKEND=http://localhost:8000
import type { Cmd, CmdResult } from './types';

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

/** 설계 이름 목록(설계 선택 상자용). 저장소(W111) 전에는 빈 목록 — 직접 입력 */
export async function listDesignIds(): Promise<string[]> {
  try {
    const res = await fetch(BASE + '/api/designs');
    if (!res.ok) return [];
    const rows = (await res.json()) as { design_id: string }[];
    return Array.isArray(rows) ? rows.map((r) => r.design_id) : [];
  } catch {
    return [];
  }
}
