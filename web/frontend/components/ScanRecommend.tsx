'use client';
// 스캔 '추천 설계'(10/13 플랜 B 1단계) — 스캔 확인(SCAN_REVIEW) 때 저장된 설계 중 스캔과 가장 비슷한 것을 보여 주고,
// 사람이 [조립]을 누르면 그 설계 ID 로 select_design(SCAN_REVIEW 에서 받음) → 이어서 start(10/13 PL — 한 번 누름 = 출발 확인, SR-09).
// 이미 그 설계로 출발 대기(READY)면 start 만(lib/api assemble — 설계 3D 의 [조립]과 같음). 이미 검사를 통과한 저장 설계라
// AI · 새 검사 · 새 저장이 필요 없다 — AI 가 막혀도 스캔 복제가 이어진다. 일치율 계산은 backend(비전 match_rate 와 같은 식).
// 다른 블록은 3D 로 비교해 사람이 확인한다(추천 설계 그대로 쌓으므로 스캔과 다른 블록은 설계대로 놓인다).
import { useEffect, useState } from 'react';
import * as api from '@/lib/api';
import { designName } from '@/lib/names';
import type { CmdResult, ScanMatch } from '@/lib/types';
import type { Robot } from '@/lib/ws';

const GOOD_RATE = 0.9; // V-48 기준(일치율 ≥ 90 %) — 이보다 낮으면 '많이 달라요'를 띄운다(막지는 않음, 사람이 정함)

interface Props {
  robot: Robot;
  onView: (designId: string) => void;
  onLog: (kind: string, text: string) => void;
}

export default function ScanRecommend({ robot, onView, onLog }: Props) {
  const runId = robot.scan?.run_id ?? null;
  const [list, setList] = useState<ScanMatch[] | null | 'fail'>(null);
  const [sending, setSending] = useState(false);
  const [sent, setSent] = useState<CmdResult | null>(null);

  useEffect(() => {
    // 새 스캔마다 다시 받는다 — 옛 스캔의 추천이 남지 않게
    setList(null);
    setSent(null);
    if (!runId) return;
    let alive = true;
    api.getScanRecommend(runId).then((r) => alive && setList(r ? r.candidates : 'fail'));
    return () => {
      alive = false;
    };
  }, [runId]);

  if (!runId) return null;
  if (list === null) return <p className="hint">비슷한 저장 설계를 찾는 중…</p>;
  if (list === 'fail' || list.length === 0) return <p className="hint">추천할 저장 설계를 못 찾았어요 — [다시 스캔]이나 [취소]를 누르세요</p>;

  const best = list[0];
  const st = robot.state;
  const ready = st?.state === 'READY' && st.design_id === best.design_id; // 앞에서 설계 선택만 되고 출발이 거절된 경우
  const canSend = !sending && !robot.safety?.locked && robot.bridge && (st?.state === 'SCAN_REVIEW' || ready);
  const line = (m: ScanMatch) => `부품 ${m.matched}/${Math.max(m.scan_count, m.part_count)} 같음 (${Math.round(m.rate * 100)} %)`;

  async function assemble() {
    setSending(true);
    onView(best.design_id); // 조립하는 설계를 오른쪽 3D 에 — 진행 색을 보며 따라가게
    const { step, r } = await api.assemble(best.design_id, ready);
    setSending(false);
    setSent(r);
    // 출발하면 스캔 칸은 닫힌다(SCAN_REVIEW 를 벗어남) — 진행 색이 보이는 설계 3D 로 화면을 옮겨 '사라졌다'고 느끼지 않게(10/13 황인재)
    if (r.success) document.querySelector('.view-panel')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
    onLog('button', `추천 설계로 조립(${best.design_id}) → ${r.success ? '출발' : `${step} 거절`}${r.reason ? ' ' + r.reason : ''}${r.message ? ' — ' + r.message : ''}`);
  }

  return (
    <div className="scan-recommend">
      <div className="kv">
        <span className="muted">가장 비슷한 저장 설계</span>
        <strong>{designName(best.design_id)}</strong>
      </div>
      <p className={best.rate >= GOOD_RATE ? 'hint' : 'result bad'}>
        {line(best)}
        {best.rate < GOOD_RATE && ' — 많이 달라요. 3D로 비교해 보고, 다르면 [다시 스캔]을 권해요'}
      </p>
      <div className="btn-row">
        <button className="secondary" onClick={() => onView(best.design_id)} title="오른쪽 3D 에서 추천 설계를 보고 스캔한 구조와 비교해요">
          3D로 보기
        </button>
        <button className="primary" disabled={!canSend}
          title="추천 설계를 그대로 쌓아요(스캔과 다른 블록은 설계대로) — 구조물을 치우고 누르세요"
          onClick={assemble}>
          {sending ? '보내는 중…' : '조립'}
        </button>
      </div>
      <small className={canSend ? 'warn-text' : 'muted'}>누르면 로봇이 움직여요 — 작업 영역에 사람이 없는지 확인하세요</small>
      {sent && !sent.success && <div className="result bad">조립 안 됨{sent.message ? ` — ${sent.message}` : ''}</div>}
      {list.length > 1 && (
        <p className="hint">
          다음 후보:{' '}
          {list.slice(1).map((m, i) => (
            <span key={m.design_id}>
              {i > 0 && ' · '}
              <a href="#" onClick={(e) => { e.preventDefault(); onView(m.design_id); }}>{designName(m.design_id)}</a> {Math.round(m.rate * 100)} %
            </span>
          ))}
        </p>
      )}
    </div>
  );
}
