'use client';
// 설계 3D 패널 — [설계 목록](열 보기 DesignTree)에서 고른 설계(없으면 로봇에 선택된 설계)를 그린다(web/README 1장 ⑥ ⑦ ⑧).
// 보는 곳에서 보낸다(10/10 황인재): 보고 있는 설계 바로 아래 [조립] 한 번 = select_design → 이어서 start(10/13 PL — 준비 · 출발 두 단계를
// 합침). 이 누름이 출발 확인(SR-09 — 사람이 작업 영역 밖인지 확인하고 누름). 같은 설계로 AI 생성은 아래 패널(GeneratePanel).
// 색: 평소엔 역할(다리 · 좌판 …)별. 그 설계로 조립 중이면 진행표(progress/2.0) 색 — 놓음 초록 · 지금 블록 노랑 · 확인 못 함 주황 · 아직 흐린 회색.
// 판정은 하지 않는다(진행표가 준 state 그대로). 블록 이름 ↔ 3D 블록은 placements.steps 의 sequence = parts.order 로 잇는다.
import { useEffect, useMemo, useState } from 'react';
import * as api from '@/lib/api';
import type { DesignPart, CmdResult, Design, Rules } from '@/lib/types';
import type { Robot } from '@/lib/ws';
import { MADE_KO, ROLE, designName, roleOf, stateName } from '@/lib/names';
import DesignTree from './DesignTree';
import Preview3D, { type Item3D } from './Preview3D';

const OTHER = '#a8a29e'; // 목록에 없는 새 역할(AI 가 지은 이름 — roles.json PR 전)
const PROG = { present: '#22c55e', current: '#facc15', unsure: '#f97316', todo: '#9ca3af' };

/** 블록 → 3D 상자(역할별 색, 스캔 추정 블록은 반투명). 설계 3D · AI 후보 미리보기(GeneratePanel)가 같이 쓴다 */
export const roleItems = (parts: DesignPart[]): Item3D[] =>
  parts.map((b) => ({ x: b.x, y: b.y, z: b.z, ori: b.ori, color: ROLE[roleOf(b.role)]?.color ?? OTHER, opacity: b.inferred ? 0.45 : 1 }));

interface Props {
  robot: Robot;
  viewId: string | null; // 보여 줄 설계 = 고른 설계 ?? 로봇에 선택된 설계
  rules: Rules | null;
  dark: boolean;
  onPick: (id: string) => void;
  onLog: (kind: string, text: string) => void;
}

const SELECTABLE = ['IDLE', 'READY', 'DONE']; // 설계를 고르고 출발할 수 있는 작업 관리자 상태(web/README 3장 상태표)

export default function DesignView({ robot, viewId, rules, dark, onPick, onLog }: Props) {
  const [design, setDesign] = useState<Design | null>(null);
  const [failed, setFailed] = useState(false);
  const [pickerOpen, setPickerOpen] = useState(false);
  const [sending, setSending] = useState(false);
  const [sent, setSent] = useState<CmdResult | null>(null); // [조립] 답(select_design · start 중 거절된 것)
  const robotDesign = robot.state?.design_id ?? null;

  useEffect(() => {
    setPickerOpen(false); // 로봇 설계가 바뀌면([조립] · 스캔 저장) 열 보기를 닫는다 — 고르기가 끝났으니
  }, [robotDesign]);

  useEffect(() => {
    setFailed(false);
    if (!viewId) {
      setDesign(null);
      return;
    }
    let live = true; // 빨리 여러 개를 눌렀을 때 늦게 온 옛 답이 덮어쓰지 않게
    api.getDesign(viewId).then((d) => {
      if (!live) return;
      setDesign(d);
      setFailed(!d);
    });
    return () => {
      live = false;
    };
  }, [viewId]);

  const st = robot.state;
  const onRobot = !!design && st?.design_id === design.design_id;
  const canSend = !!design && !sending && !robot.safety?.locked && robot.bridge && SELECTABLE.includes(st?.state ?? '');

  useEffect(() => setSent(null), [viewId]); // 다른 설계를 보면 앞 결과는 지운다

  /** [조립]: select_design → 받으면 start. 이미 이 설계로 출발 대기(READY — 스캔 저장 뒤 등)면 start 만. 바깥 영향: 로봇 출발 */
  async function assemble() {
    if (!design) return;
    setSending(true);
    const { step, r } = await api.assemble(design.design_id, onRobot && st?.state === 'READY');
    setSending(false);
    setSent(r);
    onLog('button', `조립(${design.design_id}) → ${r.success ? '출발' : `${step} 거절`}${r.reason ? ' ' + r.reason : ''}${r.message ? ' — ' + r.message : ''}`);
  }
  const running = onRobot && !!st?.run_id && !st.state.startsWith('SCAN');

  const items: Item3D[] = useMemo(() => {
    if (!design) return [];
    const idBySeq = new Map((design.placements?.steps ?? []).map((s) => [s.sequence, s.part_id]));
    const prog = running && robot.progress?.run_id === st?.run_id ? robot.progress : null;
    const stateById = new Map((prog?.parts ?? []).map((b) => [b.part_id, b.state]));
    if (!running) return roleItems(design.parts.parts);
    return design.parts.parts.map((b) => {
      const at = { x: b.x, y: b.y, z: b.z, ori: b.ori };
      const id = idBySeq.get(b.order);
      const s = id ? stateById.get(id) : undefined;
      if (id && id === st?.part_id) return { ...at, color: PROG.current };
      if (s === 'present') return { ...at, color: PROG.present };
      if (s === 'occluded' || s === 'unknown') return { ...at, color: PROG.unsure };
      return { ...at, color: PROG.todo, opacity: 0.22 };
    });
  }, [design, running, st?.part_id, st?.run_id, robot.progress]);

  const roles = design ? [...new Set(design.parts.parts.map((b) => roleOf(b.role)))] : [];
  const loading = !!viewId && !failed && design?.design_id !== viewId;

  let body: React.ReactNode;
  if (!viewId) body = <div className="view3d-empty">[설계 목록]에서 고르거나 위에서 AI로 만들면 여기에 보여요</div>;
  else if (failed) body = <div className="view3d-empty">설계를 못 받았어요</div>;
  else if (!rules) body = <div className="view3d-empty">블록 크기(robot.yaml)를 아직 못 받았어요</div>;
  else if (design) body = <Preview3D items={items} blockMm={rules.block_size_mm} frameKey={design.design_id} dark={dark} />;
  else body = <div className="view3d-empty">받는 중…</div>;

  return (
    <section className="panel view-panel">
      <div className="panel-head">
        <h2>설계 3D</h2>
        {design && (
          <span className="head-sub" title={design.design_id}>
            <strong>{designName(design.design_id)}</strong>
            <span className="muted"> · {MADE_KO[design.made_by] ?? design.made_by} · 블록 {design.parts.parts.length}개{loading ? ' · 바꾸는 중…' : ''}</span>
          </span>
        )}
        {design && onRobot && <span className="tag ok">로봇에 선택됨</span>}
        <button className="head-btn list-btn" aria-expanded={pickerOpen} onClick={() => setPickerOpen((o) => !o)}>
          {pickerOpen ? '목록 닫기 ▴' : '설계 목록 ▾'}
        </button>
      </div>
      {pickerOpen && <DesignTree picked={viewId} onRobot={robotDesign} onPick={onPick} />}
      {body}
      {design && !running && (
        <div className="send-row">
          <button className="primary big" disabled={!canSend} onClick={assemble}
            title="이 설계로 로봇이 바로 조립을 시작해요. 대기 · 출발 대기 · 완성일 때만 누를 수 있어요">
            {sending ? '보내는 중…' : '조립'}
          </button>
          <small className={canSend ? 'warn-text' : 'muted'}>
            {canSend || sending
              ? '누르면 로봇이 움직여요 — 작업 영역에 사람이 없는지 확인하세요'
              : robot.safety?.locked
                ? '멈춤 — 먼저 왼쪽 [다시 시작]'
                : !robot.bridge
                  ? '로봇 PC 연결 끊김'
                  : `지금은 조립할 수 없어요(${stateName(st?.state) || '연결 전'})`}
          </small>
          {sent && !sent.success && (
            <span className="result bad">
              거절{sent.reason ? ` (${sent.reason})` : ''}{sent.message ? ` — ${sent.message}` : ''}
            </span>
          )}
        </div>
      )}
      {design && (
        <div className="legend">
          {running ? (
            <>
              <span><i style={{ background: PROG.present }} />놓음</span>
              <span><i style={{ background: PROG.current }} />지금 블록</span>
              <span><i style={{ background: PROG.unsure }} />확인 못 함</span>
              <span><i className="ghost" style={{ background: PROG.todo }} />아직</span>
            </>
          ) : (
            roles.map((r) => (
              <span key={r || 'none'}>
                <i style={{ background: ROLE[r]?.color ?? OTHER }} />
                {ROLE[r]?.ko ?? (r || '역할 없음')}
              </span>
            ))
          )}
          {!running && design.parts.parts.some((b) => b.inferred) && <span><i className="ghost" style={{ background: OTHER }} />스캔 추정</span>}
        </div>
      )}
    </section>
  );
}
