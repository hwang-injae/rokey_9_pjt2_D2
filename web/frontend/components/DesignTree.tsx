'use client';
// 설계 트리 — 저장소(DesignStore)의 설계를 가구별로, 부모 → 파생 순으로 보여 준다(web/README 1장 ⑥).
// 누르면 3D 로 보기만 한다. 로봇에 보내는 것은 [설계 선택] 버튼 하나(RobotPanel) — 고르는 것과 로봇에 보내는 것을 나눈다.
import { useEffect, useState } from 'react';
import * as api from '@/lib/api';
import type { DesignSummary } from '@/lib/types';

const FAMILY_KO: Record<string, string> = { chair: '의자', desk: '책상' };
export const MADE_KO: Record<string, string> = { cad: '도면', web: '웹 글', voice: '음성', scan: '스캔' };

export default function DesignTree({ picked, onRobot, onPick }: { picked: string | null; onRobot: string | null; onPick: (id: string) => void }) {
  const [rows, setRows] = useState<DesignSummary[] | null | undefined>(undefined); // undefined = 받는 중 · null = 못 받음

  useEffect(() => {
    api.listDesigns().then(setRows); // 열 때마다 새로 받는다 — 방금 저장한 설계도 보이게
  }, []);

  if (rows === undefined) return <div className="tree muted">설계 목록 받는 중…</div>;
  if (rows === null) return <div className="tree bad-text">설계 목록을 못 받았어요 — 웹 서버(backend)를 확인하세요</div>;
  if (!rows.length) return <div className="tree muted">저장된 설계가 없어요</div>;

  // 부모가 목록에 없으면(지워졌거나 아직 안 옴) 맨 위에 둔다 — 설계가 트리에서 사라지지 않게
  const ids = new Set(rows.map((r) => r.design_id));
  const kids = new Map<string, DesignSummary[]>();
  for (const r of rows) {
    const p = r.parent_id && ids.has(r.parent_id) ? r.parent_id : '';
    kids.set(p, [...(kids.get(p) ?? []), r]);
  }
  const roots = kids.get('') ?? [];
  const families = [...new Set(roots.map((r) => r.family))];

  const node = (r: DesignSummary) => {
    const children = kids.get(r.design_id) ?? [];
    return (
      <li key={r.design_id}>
        <button type="button" className={`tree-node${picked === r.design_id ? ' picked' : ''}`} onClick={() => onPick(r.design_id)}>
          <span className="tree-id">{r.design_id}</span>
          <span className="tree-meta">
            v{r.version} · {MADE_KO[r.made_by] ?? r.made_by} · 블록 {r.block_count}
          </span>
          {onRobot === r.design_id && <span className="tag ok">로봇</span>}
        </button>
        {children.length > 0 && <ul>{children.map(node)}</ul>}
      </li>
    );
  };

  return (
    <div className="tree">
      {families.map((f) => (
        <div key={f}>
          <div className="tree-family">{FAMILY_KO[f] ?? f}</div>
          <ul>{roots.filter((r) => r.family === f).map(node)}</ul>
        </div>
      ))}
    </div>
  );
}
