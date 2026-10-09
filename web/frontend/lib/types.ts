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

/** progress/1.1 — 진행표. 판정은 없다(dx · dy 는 측정값, 지금은 NaN) */
export interface Progress {
  schema: string;
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

/** scan_result/1.1 — 추론 블록(blocks/1, inferred 표시) · 사진 · 점군 경로(cloud_path 는 선택 칸) */
export interface ScanResult {
  schema: string;
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
  | { type: 'wrist_image'; data: { seq: number; stamp: number } };

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
