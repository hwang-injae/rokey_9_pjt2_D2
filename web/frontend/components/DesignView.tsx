'use client';
// 설계 3D 패널 — [설계 목록](열 보기 DesignTree)에서 고른 설계(없으면 로봇에 선택된 설계)를 그린다(web/README 1장 ⑥ ⑦ ⑧).
// 보는 곳에서 보낸다(10/10 황인재): 보고 있는 설계 바로 아래 [이 설계로 조립 준비](= select_design)가 로봇에 보내는 유일한 버튼.
// 그다음 [출발]은 왼쪽 로봇 패널(사람이 영역 밖 확인 — SR-09).
// 색: 평소엔 역할(다리 · 좌판 …)별. 그 설계로 조립 중이면 진행표(progress/1.1) 색 — 놓음 초록 · 지금 블록 노랑 · 확인 못 함 주황 · 아직 흐린 회색.
// 판정은 하지 않는다(진행표가 준 state 그대로). 블록 이름 ↔ 3D 블록은 placements.steps 의 sequence = blocks.order 로 잇는다.
import { useEffect, useMemo, useState } from 'react';
import * as api from '@/lib/api';
import type { Block2, CmdResult, Design, Rules } from '@/lib/types';
import type { Robot } from '@/lib/ws';
import DesignTree, { MADE_KO } from './DesignTree';
import Preview3D, { type Item3D } from './Preview3D';

const ROLE: Record<string, { ko: string; color: string }> = {
  LEG: { ko: '다리', color: '#b7794a' },
  SEAT: { ko: '좌판', color: '#e3b341' },
  BACK: { ko: '등받이', color: '#5b8fd6' },
  BEAM: { ko: '보', color: '#8f7ad6' },
  TOP: { ko: '상판', color: '#e8a33c' },
  BASE: { ko: '바닥 받침', color: '#6fa36b' },
  COLUMN: { ko: '기둥', color: '#d0705f' },
};
const OTHER = '#a8a29e'; // 목록에 없는 새 역할(AI 가 지은 이름 — roles.json PR 전)
const PROG = { present: '#22c55e', current: '#facc15', unsure: '#f97316', todo: '#9ca3af' };
const roleOf = (role?: string) => (role ?? '').split('_')[0]; // LEG_WHEEL → LEG (옵션은 색을 나누지 않음)

/** 블록 → 3D 상자(역할별 색, 스캔 추정 블록은 반투명). 설계 3D · AI 후보 미리보기(GeneratePanel)가 같이 쓴다 */
export const roleItems = (blocks: Block2[]): Item3D[] =>
  blocks.map((b) => ({ x: b.x, y: b.y, z: b.z, ori: b.ori, color: ROLE[roleOf(b.role)]?.color ?? OTHER, opacity: b.inferred ? 0.45 : 1 }));

interface Props {
  robot: Robot;
  viewId: string | null; // 보여 줄 설계 = 고른 설계 ?? 로봇에 선택된 설계
  rules: Rules | null;
  dark: boolean;
  onPick: (id: string) => void;
  onLog: (kind: string, text: string) => void;
}

const SELECTABLE = ['IDLE', 'READY', 'DONE']; // 설계를 바꿀 수 있는 작업 관리자 상태(web/README 3장 상태표)

export default function DesignView({ robot, viewId, rules, dark, onPick, onLog }: Props) {
  const [design, setDesign] = useState<Design | null>(null);
  const [failed, setFailed] = useState(false);
  const [pickerOpen, setPickerOpen] = useState(false);
  const [sending, setSending] = useState(false);
  const [sent, setSent] = useState<CmdResult | null>(null); // [이 설계로 조립 준비] 답(성공이면 상태가 READY 로 바뀐다)
  const robotDesign = robot.state?.design_id ?? null;

  useEffect(() => {
    setPickerOpen(false); // 로봇 설계가 바뀌면([이 설계로 조립 준비] 받음 · 음성) 열 보기를 닫는다 — 고르기가 끝났으니
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
  const canSend = !!design && !onRobot && !sending && !robot.safety?.locked && SELECTABLE.includes(st?.state ?? '');

  useEffect(() => setSent(null), [viewId]); // 다른 설계를 보면 앞 결과는 지운다

  async function sendToRobot() {
    if (!design) return;
    setSending(true);
    const r = await api.command('select_design', design.design_id);
    setSending(false);
    setSent(r);
    onLog('button', `조립 준비(${design.design_id}) → ${r.success ? '받음' : '거절'}${r.reason ? ' ' + r.reason : ''}${r.message ? ' — ' + r.message : ''}`);
  }
  const running = onRobot && !!st?.run_id && !st.state.startsWith('SCAN');

  const items: Item3D[] = useMemo(() => {
    if (!design) return [];
    const idBySeq = new Map((design.placements?.steps ?? []).map((s) => [s.sequence, s.block_id]));
    const prog = running && robot.progress?.run_id === st?.run_id ? robot.progress : null;
    const stateById = new Map((prog?.blocks ?? []).map((b) => [b.block_id, b.state]));
    if (!running) return roleItems(design.blocks.blocks);
    return design.blocks.blocks.map((b) => {
      const at = { x: b.x, y: b.y, z: b.z, ori: b.ori };
      const id = idBySeq.get(b.order);
      const s = id ? stateById.get(id) : undefined;
      if (id && id === st?.block_id) return { ...at, color: PROG.current };
      if (s === 'present') return { ...at, color: PROG.present };
      if (s === 'occluded' || s === 'unknown') return { ...at, color: PROG.unsure };
      return { ...at, color: PROG.todo, opacity: 0.22 };
    });
  }, [design, running, st?.block_id, st?.run_id, robot.progress]);

  const roles = design ? [...new Set(design.blocks.blocks.map((b) => roleOf(b.role)))] : [];
  const loading = !!viewId && !failed && design?.design_id !== viewId;

  let body: React.ReactNode;
  if (!viewId) body = <div className="view3d-empty">[설계 목록]에서 설계를 누르거나 위에서 AI로 만들면 여기에 3D로 보여요</div>;
  else if (failed) body = <div className="view3d-empty">{viewId} 설계를 못 받았어요 — 저장소에 없거나 웹 서버 문제</div>;
  else if (!rules) body = <div className="view3d-empty">블록 크기(robot.yaml)를 아직 못 받았어요</div>;
  else if (design) body = <Preview3D items={items} blockMm={rules.block_size_mm} frameKey={design.design_id} dark={dark} />;
  else body = <div className="view3d-empty">받는 중…</div>;

  return (
    <section className="panel view-panel">
      <div className="panel-head">
        <h2>설계 3D</h2>
        {design && (
          <span className="muted">
            {design.design_id} · {design.version} · {MADE_KO[design.made_by] ?? design.made_by} · 블록 {design.blocks.blocks.length}개
            {loading ? ' (바꾸는 중…)' : ''}
          </span>
        )}
        {design && onRobot && <span className="tag ok">로봇에 선택됨 — 왼쪽 [출발]</span>}
        <button className="head-btn list-btn" aria-expanded={pickerOpen} onClick={() => setPickerOpen((o) => !o)}>
          {pickerOpen ? '목록 닫기 ▴' : '설계 목록 ▾'}
        </button>
      </div>
      {pickerOpen && <DesignTree picked={viewId} onRobot={robotDesign} onPick={onPick} />}
      {body}
      {design && !onRobot && (
        <div className="send-row">
          <button className="primary" disabled={!canSend} onClick={sendToRobot}>
            {sending ? '보내는 중…' : '이 설계로 조립 준비'}
          </button>
          <small className="muted">
            {canSend || sending
              ? '로봇에 이 설계를 보내요(로봇은 아직 안 움직여요) — 그다음 왼쪽 [출발]'
              : robot.safety?.locked
                ? '멈춤 상태예요 — 왼쪽 [다시 시작]부터'
                : `로봇이 ${st?.state ?? '연결 전'} 상태라 지금은 설계를 바꿀 수 없어요(대기 · 출발 대기 · 완성일 때만)`}
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
          {!running && design.blocks.blocks.some((b) => b.inferred) && <span><i className="ghost" style={{ background: OTHER }} />스캔 추정</span>}
          <span className="muted hint">끌기: 돌리기 · 휠: 확대 · 오른쪽 끌기: 옮기기</span>
        </div>
      )}
    </section>
  );
}
