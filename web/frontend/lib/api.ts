// REST 한 곳(SDD 3.3 — 화면은 backend 의 REST · /ws 만 쓴다, E-41).
// 운영: backend 가 이 화면을 같은 주소에서 내려 주므로 BASE 는 빈 글자. 개발(next dev :3000): NEXT_PUBLIC_BACKEND=http://localhost:8000
import type { Cmd, CmdResult, Design, DesignSummary, GenJob, Rules, TemplateChoice } from './types';

export const BASE = process.env.NEXT_PUBLIC_BACKEND ?? '';
// 버튼 답을 기다리는 시간 = backend 의 req_timeout_s(robot.yaml 한 곳 — /ws 가 붙을 때 timing 으로 알려 줌) + 여유.
// backend 는 MQTT 답을 req_timeout_s 까지 기다린 뒤 답하므로 그보다 길어야 한다. 넘으면 실패로 보고 버튼을 푼다
const MARGIN_MS = 3000; // backend 가 시간 초과를 정한 뒤 그 답이 화면까지 오는 여유
let reqTimeoutS: number | null = null;

/** /ws 가 붙을 때 backend 가 알려 준 요청 시간 제한(초)을 기억한다(ws.ts 가 부름) */
export function setReqTimeout(s: number) {
  reqTimeoutS = s;
}

async function post(path: string, body?: unknown): Promise<CmdResult> {
  if (reqTimeoutS == null) {
    // 화면을 막 열어 아직 웹 서버와 한 번도 못 붙음 — 배너와 같은 안내(얼마나 기다릴지도 모르는 채 보내지 않는다)
    return { success: false, reason: 'ERROR', message: '웹 서버와 아직 연결 전 — 잠시 뒤 다시. 급하면 정지는 로봇 PC 키 · 펜던트로' };
  }
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), reqTimeoutS * 1000 + MARGIN_MS);
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

// ---------- AI 생성(W108 · W112) ----------
const AI_TIMEOUT_MS = 45000; // Template 고르기는 GPT 를 한 번 부른다(호출당 30초 + 다시 1번) — 버튼 명령보다 길게

/** POST 하고 JSON 을 받는다. 실패면 {error} — FastAPI 의 {detail} 글을 그대로(예: 409 '이미 설계를 만드는 중이에요') */
async function postJson<T>(path: string, body: unknown, timeoutMs = AI_TIMEOUT_MS): Promise<{ data?: T; error?: string }> {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), timeoutMs);
  try {
    const res = await fetch(BASE + path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
      signal: ctrl.signal,
    });
    const json = await res.json().catch(() => null);
    if (!res.ok) return { error: (json && typeof json.detail === 'string' ? json.detail : null) ?? `웹 서버 응답 ${res.status}` };
    return { data: json as T };
  } catch {
    return { error: '웹 서버에 닿지 않거나 너무 오래 걸려요' };
  } finally {
    clearTimeout(timer);
  }
}

/** 요청 문장 → AI 가 고른 모양(Template) + 이유 + 고를 수 있는 목록 */
export const chooseTemplate = (text: string) => postJson<TemplateChoice>('/api/designs/template', { text });
/** 후보 3개 만들기 시작 → {job_id}(한 번에 하나 — 이미 만드는 중이면 error) */
export const startGenerate = (text: string, template: string) =>
  postJson<{ job_id: string }>('/api/designs/generate', { text, template }, 10000);
/** 생성 작업 하나의 지금 모습. 없으면 null */
export const getGenJob = (jobId: string) => getJson<GenJob>('/api/designs/generate/' + encodeURIComponent(jobId));
/** 고른 후보 저장 → {design_id, version, parent_id, index}. 떨어진 후보 · 이미 저장이면 error */
export const pickCandidate = (jobId: string, index: number, madeBy: 'web' | 'voice' = 'web') =>
  postJson<{ design_id: string; version: string; parent_id: string | null; index: number }>(
    `/api/designs/generate/${encodeURIComponent(jobId)}/pick`, { index, made_by: madeBy }, 15000);

