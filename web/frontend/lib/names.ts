// 화면에 보일 이름 한 곳 — 코드 이름(설계 ID · 블록 이름 · 상태 코드 · 정지 이유 · 검사 오류)을 사람이 읽는 말로 바꾼다
// (10/10 황인재: 코드 같은 글 · 긴 글은 화면에 안 보이게). 모양(Template) 이름은 backend 한 곳(GET /api/designs/templates)에서 받는다.

let templateNames: Record<string, string> = {};

/** backend 가 준 모양 이름을 기억한다(page 가 처음 붙을 때 한 번) */
export function setTemplateNames(list: { id: string; name: string }[]) {
  templateNames = Object.fromEntries(list.map((t) => [t.id, t.name]));
}

const DESIGN_RE = /^(\d{3}(?:_[A-Z]+)+)_V(\d{3})$/; // <Template ID>_V<3자리>(E-84)

/** '001_CHAIR_BENCH_V001' → '벤치 V001', V000 → '벤치 (기본)'. 꼴이 다르면 받은 그대로 */
export function designName(id?: string | null): string {
  if (!id) return '';
  const m = DESIGN_RE.exec(id);
  if (!m) return id;
  const name = templateNames[m[1]] ?? m[1];
  return m[2] === '000' ? `${name} (기본)` : `${name} V${m[2]}`;
}

/** 역할 → 한국어 · 3D 색(설계 3D · 후보 미리보기 · 블록 이름이 같이 쓴다) */
export const ROLE: Record<string, { ko: string; color: string }> = {
  LEG: { ko: '다리', color: '#b7794a' },
  SEAT: { ko: '좌판', color: '#e3b341' },
  BACK: { ko: '등받이', color: '#5b8fd6' },
  BEAM: { ko: '보', color: '#8f7ad6' },
  TOP: { ko: '상판', color: '#e8a33c' },
  BASE: { ko: '바닥 받침', color: '#6fa36b' },
  COLUMN: { ko: '기둥', color: '#d0705f' },
};
export const roleOf = (role?: string) => (role ?? '').split('_')[0]; // LEG_WHEEL → LEG (옵션은 나누지 않음)

/** '001_CHAIR_BENCH_V001_LEG_001_03' → '다리 1-3'(부품 1의 3번째 블록) */
export function blockName(blockId?: string | null, designId?: string | null): string {
  if (!blockId) return '';
  const rest = designId && blockId.startsWith(designId + '_') ? blockId.slice(designId.length + 1) : blockId;
  const m = /^([A-Z]+)(?:_[A-Z]+)?_(\d{3})_(\d{2})$/.exec(rest);
  return m ? `${ROLE[m[1]]?.ko ?? m[1]} ${Number(m[2])}-${Number(m[3])}` : rest;
}

export const MADE_KO: Record<string, string> = { cad: '도면', web: '웹 글', voice: '음성', scan: '스캔' };

const STATE_KO: Record<string, string> = {
  IDLE: '대기', READY: '출발 대기', CHECK: '관측 중', SELECT: '다음 블록 고르는 중', PICK_PLACE: '집고 놓는 중',
  VERIFY: '놓은 블록 확인 중', WAIT_SUPPLY: '블록 공급 기다림', WAIT_HMI: '웹 연결 기다림', SCAN_MOVE: '스캔 — 이동',
  SCAN_CAPTURE: '스캔 — 촬영', SCAN_INFER: '스캔 — 계산', SCAN_REVIEW: '스캔 결과 확인', STOPPED: '멈춤',
  RECOVER: '다시 시작 중', ERROR: '오류', DONE: '완성',
};

/** 작업 관리자 상태 코드 → 한국어(모르는 코드는 그대로) */
export const stateName = (s?: string | null) => (s ? STATE_KO[s] ?? s : '');

/** 정지 이유(IRD 7장) → 누가 · 왜 */
export function stopReasonKo(reason: string): string {
  if (reason.startsWith('ROBOT_ALARM:')) return '로봇 알람으로 멈췄어요';
  const ko: Record<string, string> = {
    STOP_WEB: '화면의 정지 버튼으로 멈췄어요', STOP_KEY: '로봇 PC 키로 멈췄어요', STOP_TASK: '작업 관리자가 멈췄어요',
    TIMEOUT: '시간 초과 — 작업 관리자가 먼저 세웠어요', CTRL_C: '프로그램 종료로 멈췄어요',
  };
  return ko[reason] ?? '멈췄어요';
}

/** 검사 오류 한 줄 → 짧은 말(자세한 글은 마우스를 올리면 보이게 부르는 쪽이 title 로) */
export function checkReasonKo(detail: string): string {
  const table: [string, string][] = [
    ['공중', '받침 없는 블록'], ['여유', '넘어질 수 있음'], ['겹침', '블록이 겹침'], ['grasp', '로봇이 잡기 어려움'],
    ['잡을 면', '로봇이 잡기 어려움'], ['작업영역', '작업 공간을 넘음'], ['작업대', '작업 공간을 넘음'], ['역할', '이름 규칙 위반'],
    ['part', '부품 묶음 규칙 위반'], ['형식', '형식 오류'], ['상한', '블록이 너무 많음'],
  ];
  return table.find(([k]) => detail.includes(k))?.[1] ?? '규칙 위반';
}
