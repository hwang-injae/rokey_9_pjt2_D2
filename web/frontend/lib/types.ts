// 화면이 받는 JSON 꼴 — IRD 6장 칸 그대로(정본은 IRD). backend 는 MQTT 로 받은 JSON 을 고치지 않고 넘긴다.

export type Cmd = 'select_design' | 'start' | 'scan' | 'cancel';

/** state/1 — 작업 관리자 상태(IRD 2장 state · message_id) */
export interface TaskState {
  schema: string;
  stamp?: number;
  mode?: string;
  state: string;
  run_id: string | null;
  design_id: string | null;
  block_id: string | null;
  message_id: string | null;
  message: string;
}

/** safety_state/1 — 정지 노드. reason = STOP_WEB · STOP_KEY · STOP_TASK · ROBOT_ALARM:<상태> · CTRL_C · TIMEOUT (IRD 7장) */
export interface SafetyState {
  schema: string;
  stamp?: number;
  stopped: boolean;
  locked: boolean;
  reason: string;
}

/** progress/1.1 — 진행표(1.1 = 보낸 시각 stamp 더함, E-75). 판정은 없다(dx · dy 는 측정값, 지금은 NaN). 형식은 앞자리만 본다 */
export interface Progress {
  schema: string;
  stamp?: number;
  design_id: string | null;
  run_id: string | null;
  blocks: { block_id: string; state: 'present' | 'absent' | 'occluded' | 'unknown'; by?: string }[];
}

/** gripper_state/1 — 폭을 못 읽으면 width_m null(E-62) */
export interface GripperState {
  schema: string;
  width_m: number | null;
  grasped: boolean;
}

/** scan_result/1.2 — 추론 블록(blocks/1, inferred 표시) · 사진 · 점군 경로(cloud_path 선택 칸) · stamp(1.1, E-75) ·
 *  nearest_base(1.2, E-79 선택 칸). poses_used 개수는 정해져 있지 않다(W166 — 3개를 가정하지 않는다) */
export interface ScanResult {
  schema: string;
  stamp?: number;
  run_id: string;
  blocks: { schema: string; design_id: string; family: string; blocks: { order: number; x: number; y: number; z: number; ori: string; inferred: boolean }[] };
  inferred_count: number;
  image_path?: string;
  cloud_path?: string;
  nearest_base?: string;
  poses_used?: string[];
}

/** intent/1 — 음성 의도 */
export interface Intent {
  schema: string;
  intent: string;
  design_id: string | null;
  text: string;
  confidence: number;
}

/** /ws 이벤트 — 모두 {type, data} 한 겹(web/README 2장) */
export type WsEvent =
  | { type: 'state'; data: TaskState }
  | { type: 'safety'; data: SafetyState }
  | { type: 'progress'; data: Progress }
  | { type: 'gripper'; data: GripperState }
  | { type: 'scan_result'; data: ScanResult }
  | { type: 'intent'; data: Intent }
  | { type: 'bridge_alive'; data: { alive: boolean } }
  | { type: 'broker'; data: { connected: boolean } }
  | { type: 'wrist_image'; data: { seq: number; stamp: number } }
  | { type: 'timing'; data: { req_timeout_s: number } }
  | { type: 'gen'; data: GenEvent };

/** 버튼 응답 — 다리가 넘긴 ROS 응답 칸 그대로(명령은 success · reason, 정지 · 다시 시작은 success · message) */
export interface CmdResult {
  success: boolean;
  reason?: string;
  message?: string;
}

/** blocks/2.0 블록 하나(IRD 6장) — mm · 설계 좌표계(바닥 외곽 가운데 = (0,0), x 오른쪽 · y 뒤 · z 위 = 아랫면 높이) */
export interface Block2 {
  order: number;
  x: number;
  y: number;
  z: number;
  ori: string; // x · y · xe · ye · zx · zy (IRD 2장)
  role?: string;
  part?: number;
  stage?: number;
  grasp?: string;
  inferred?: boolean; // 스캔 설계만
}

/** design/2.0 — 화면이 쓰는 칸만(recipe 는 그리지 않는다). placements.steps 로 블록 이름 ↔ order(= sequence) 를 잇는다 */
export interface Design {
  schema: string;
  design_id: string;
  family: string;
  version: string;
  parent_id: string | null;
  made_by: string;
  blocks: { schema: string; blocks: Block2[] };
  placements: { steps: { block_id: string; sequence: number }[] };
}

/** GET /api/designs 한 줄 — 트리 · 목록용 요약(DesignStore.list_designs) */
export interface DesignSummary {
  design_id: string;
  family: string;
  version: string;
  parent_id: string | null;
  made_by: string;
  block_count: number;
  size_mm: [number, number, number];
}

/** GET /api/designs/rules — robot.yaml 에서 backend 가 읽어 준 설계 규칙 숫자(10/9 PL E-78) */
export interface Rules {
  block_size_mm: [number, number, number];
  assembly_area_half_mm: number;
}

/** AI 생성 — 후보 하나의 검사 결과(check_result/2.0 중 화면이 쓰는 칸, 레시피는 backend 에만) */
export interface GenCheck {
  ok: boolean;
  reason: string;
  min_margin_mm: number | null;
  errors: { block: number | null; reason: string; detail: string }[];
}

/** AI 후보 하나 — 블록(3D) · 한 줄 생각 · 검사 결과(검사 전이면 없음) */
export interface GenCandidate {
  idea: string;
  blocks: { schema: string; design_id: string; family: string; blocks: Block2[] };
  check?: GenCheck;
}

/** GET /api/designs/generate/{job_id} — 생성 작업 하나(후보는 backend 메모리에만, E-84 ⑥) */
export interface GenJob {
  job_id: string;
  state: 'running' | 'ready' | 'failed' | 'saved';
  text: string;
  template: string;
  candidates: GenCandidate[];
  reading?: string[]; // AI 가 도구로 읽은 참고 설계(E-72)
  attempt?: number; // 몇 번째 만들기(다 떨어지면 다시 — 최대 3)
  design_id?: string;
  parent_id?: string;
  reference?: string[];
  attempts?: number;
  elapsed_s?: number;
  code?: string; // 실패 코드(OUT_OF_SCOPE · GEN_FAILED · ERROR · TIMEOUT)
  message?: string;
  saved?: { design_id: string; version: string; parent_id: string | null; index: number };
}

/** /ws {type: gen} — 생성 진행 알림. 화면은 이것을 받으면 job 을 GET 으로 다시 받는다(알림이 몰려와도 빠짐없이) */
export interface GenEvent {
  job_id: string;
  stage: 'reading' | 'candidates' | 'checked' | 'retry' | 'done' | 'failed' | 'saved';
  state?: GenJob['state'];
  design_id?: string;
  message?: string;
}

/** POST /api/designs/template — AI 가 고른 모양(Template) + 이유 + 고를 수 있는 목록(E-84 ③, W163) */
export interface TemplateChoice {
  success: boolean;
  template?: string | null; // null = 의자 · 책상 밖이거나 맞는 모양 없음
  reason?: string;
  templates?: { id: string; label: string }[];
  code?: string;
  message?: string;
}

