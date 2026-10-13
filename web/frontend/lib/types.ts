// 화면이 받는 JSON 꼴 — IRD 6장 칸 그대로(정본은 IRD). backend 는 MQTT 로 받은 JSON 을 고치지 않고 넘긴다.

export type Cmd = 'select_design' | 'start' | 'scan' | 'cancel';

/** state/2 — 작업 관리자 상태(IRD 2장 state · message_id) */
export interface TaskState {
  schema: string;
  stamp?: number;
  mode?: string;
  state: string;
  run_id: string | null;
  design_id: string | null;
  part_id: string | null;
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

/** progress/2.0 — 진행표(2.0 = 부품 목록 이름 parts · part_id, 10/13 · 1.1 = 보낸 시각 stamp 더함, E-75). 판정은 없다(dx · dy 는 측정값, 지금은 NaN). 형식은 앞자리만 본다 */
export interface Progress {
  schema: string;
  stamp?: number;
  design_id: string | null;
  run_id: string | null;
  parts: { part_id: string; state: 'present' | 'absent' | 'occluded' | 'unknown'; by?: string }[];
}

/** gripper_state/1 — 폭을 못 읽으면 width_m null(E-62) */
export interface GripperState {
  schema: string;
  width_m: number | null;
  grasped: boolean;
}

/** scan_result/2.0 — 추론 블록(blocks/2, inferred 표시) · 사진 · 점군 경로(cloud_path 선택 칸) · stamp(1.1, E-75) ·
 *  nearest_base(1.2, E-79 선택 칸). poses_used 개수는 정해져 있지 않다(W166 — 3개를 가정하지 않는다) */
export interface ScanResult {
  schema: string;
  stamp?: number;
  run_id: string;
  blocks: { schema: string; design_id: string; family: string; blocks: { scan_num: number; x: number; y: number; z: number; ori: string; inferred: boolean }[] };
  inferred_count: number;
  image_path?: string;
  cloud_path?: string;
  nearest_base?: string;
  poses_used?: string[];
}

/** 스캔 '추천 설계' 한 줄(GET /api/designs/scan/recommend — 10/13 플랜 B 1단계). rate = matched ÷ max(scan_count, part_count) */
export interface ScanMatch {
  design_id: string;
  family: string;
  made_by: string;
  matched: number;
  scan_count: number;
  part_count: number;
  rate: number;
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
  | { type: 'scan_image' | 'scan_cloud'; data: BlobInfo }
  | { type: 'timing'; data: { req_timeout_s: number } }
  | { type: 'gen'; data: GenEvent }
  | { type: 'scan_save'; data: ScanSaveJob & { stage: string } };

/** 스캔 사진 · 점군 알림 — 내용은 /api/robot/scan.jpg · scan_cloud.ply(seq 로 새로 받음). run_id = 어느 스캔 것인지(IRD 10.5) */
export interface BlobInfo {
  seq: number;
  stamp: number;
  bytes: number;
  run_id?: string;
}

/** 스캔 [그대로 저장] 작업(IRD 8.4 ⑦ — GET /api/designs/scan/save/{job_id}). 실패면 actions = 남길 버튼(scope = 취소만) */
export interface ScanSaveJob {
  job_id: string;
  state: 'running' | 'saved' | 'failed';
  run_id?: string;
  attempt?: number;
  code?: string;
  message?: string;
  actions?: ('cancel' | 'rescan')[];
  saved?: { design_id: string; version: string; parent_id: string | null };
}

/** 버튼 응답 — 다리가 넘긴 ROS 응답 칸 그대로(명령은 success · reason, 정지 · 다시 시작은 success · message) */
export interface CmdResult {
  success: boolean;
  reason?: string;
  message?: string;
}

/** parts/3.0 부품 하나(IRD 6장 — 10/13 이름 규칙: 설계 안 부품 = part) — mm · 설계 좌표계(바닥 외곽 가운데 = (0,0), x 오른쪽 · y 뒤 · z 위 = 아랫면 높이) */
export interface DesignPart {
  order: number;
  x: number;
  y: number;
  z: number;
  ori: string; // x · y · xe · ye · zx · zy (IRD 2장)
  role?: string;
  group?: number;
  stage?: number;
  grasp?: string;
  inferred?: boolean; // 스캔 설계만
}

/** design/3.0 — 화면이 쓰는 칸만(recipe 는 그리지 않는다). placements.steps 로 블록 이름 ↔ order(= sequence) 를 잇는다 */
export interface Design {
  schema: string;
  design_id: string;
  family: string;
  version: string;
  parent_id: string | null;
  made_by: string;
  parts: { schema: string; parts: DesignPart[] };
  placements: { steps: { part_id: string; sequence: number }[] };
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

/** AI 생성 — 후보 하나의 검사 결과(check_result/3.0 중 화면이 쓰는 칸, 레시피는 backend 에만) */
export interface GenCheck {
  ok: boolean;
  reason: string;
  min_margin_mm: number | null;
  errors: { part: number | null; reason: string; detail: string }[];
}

/** AI 후보 하나 — 블록(3D) · 한 줄 생각 · 검사 결과(검사 전이면 없음) */
export interface GenCandidate {
  idea: string;
  parts: { schema: string; design_id: string; parts: DesignPart[] };
  check?: GenCheck;
}

/** GET /api/designs/generate/{job_id} — 생성 작업 하나(후보는 backend 메모리에만, E-84 ⑥) */
export interface GenJob {
  job_id: string;
  state: 'running' | 'ready' | 'failed' | 'saved';
  text: string;
  base_id?: string; // 사람이 설계 목록에서 고른 설계 = 부모(10/13 PL)
  template: string; // base_id 앞부분
  candidates: GenCandidate[];
  reading?: string[]; // AI 가 도구로 참고만 읽은 같은 Template 의 다른 설계(E-72)
  attempt?: number; // 몇 번째 만들기(처음 + 모자란 만큼 더 2번 — 최대 3)
  count?: number; // 이번에 검사하는 후보 수(그림은 끝난 뒤 통과한 것만 — 10/11 PL)
  passed?: number; // 지금까지 통과한 후보 수
  need?: number; // 모자라서 더 만드는 수
  design_id?: string;
  parent_id?: string;
  reference?: string[];
  attempts?: number;
  elapsed_s?: number;
  code?: string; // 실패 코드(OUT_OF_SCOPE · GEN_FAILED · TEMPLATE_SHAPE · TEMPLATE_RANGE · REQUEST_INVALID · ERROR · TIMEOUT)
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

