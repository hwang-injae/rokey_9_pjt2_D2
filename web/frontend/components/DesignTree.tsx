'use client';
// 설계 목록 — 열 보기(Miller columns, 10/9 황인재가 고른 CodePen 모양). 가구 → 기본 설계 → 파생 → 그 파생 … 을 왼쪽에서 오른쪽 열로.
// 한 열에서 고르면 오른쪽에 그 설계에서 파생된 설계들이 열린다. 설계가 많아져도 한 열에는 한 부모의 자식만 보여 덜 복잡하다.
// 누르면 3D 로 보기만 한다. 로봇에 보내는 것은 3D 아래 [이 설계로 조립 준비] 하나(DesignView) — 보는 것과 보내는 것을 나눈다.
import { useEffect, useMemo, useRef, useState } from 'react';
import * as api from '@/lib/api';
import { MADE_KO, designName } from '@/lib/names';
import type { DesignSummary } from '@/lib/types';

const FAMILY_KO: Record<string, string> = { chair: '의자', desk: '책상' };

interface Column {
  title: string;
  items: { key: string; label: string; meta?: string; kids: number; design?: DesignSummary }[];
}

export default function DesignTree({ picked, onRobot, onPick }: { picked: string | null; onRobot: string | null; onPick: (id: string) => void }) {
  const [rows, setRows] = useState<DesignSummary[] | null | undefined>(undefined); // undefined = 받는 중 · null = 못 받음
  const [path, setPath] = useState<string[]>([]); // [가구, 기본 설계, 파생, …] — 열마다 고른 것
  const box = useRef<HTMLDivElement>(null);

  useEffect(() => {
    api.listDesigns().then(setRows); // 열 때마다 새로 받는다 — 방금 저장한 설계도 보이게
  }, []);

  // 부모 → 자식 표. 부모가 목록에 없으면(지워졌거나 아직 안 옴) 그 가구의 맨 앞 열에 둔다 — 설계가 사라지지 않게
  const { byId, kids, families } = useMemo(() => {
    const byId = new Map((rows ?? []).map((r) => [r.design_id, r]));
    const kids = new Map<string, DesignSummary[]>();
    for (const r of rows ?? []) {
      const p = r.parent_id && byId.has(r.parent_id) ? r.parent_id : `family:${r.family}`;
      kids.set(p, [...(kids.get(p) ?? []), r]);
    }
    return { byId, kids, families: [...new Set((rows ?? []).map((r) => r.family))] };
  }, [rows]);

  // 처음 열 때: 보고 있는 설계(없으면 첫 가구)까지 열을 펼쳐 둔다
  useEffect(() => {
    if (!rows?.length) return;
    const chain: string[] = [];
    for (let d = picked ? byId.get(picked) : undefined; d; d = d.parent_id ? byId.get(d.parent_id) : undefined) {
      if (chain.includes(d.design_id)) break; // 부모가 돌고 도는 잘못된 기록이면 멈춘다
      chain.unshift(d.design_id);
    }
    const family = chain.length ? byId.get(chain[0])!.family : families[0];
    setPath([family, ...chain]);
  }, [rows]); // 목록을 받았을 때 한 번만 — 그 뒤에는 사람이 고른 길을 따른다

  // 열이 늘면 오른쪽 끝(새 열)이 보이게
  useEffect(() => {
    box.current?.scrollTo({ left: box.current.scrollWidth, behavior: 'smooth' });
  }, [path.length]);

  if (rows === undefined) return <div className="tree muted">설계 목록 받는 중…</div>;
  if (rows === null) return <div className="tree bad-text">설계 목록을 못 받았어요 — 웹 서버(backend)를 확인하세요</div>;
  if (!rows.length) return <div className="tree muted">저장된 설계가 없어요</div>;

  const item = (d: DesignSummary) => ({
    key: d.design_id,
    label: designName(d.design_id), // 코드 ID 대신 '벤치 V001'(10/10) — ID 는 title 로
    meta: `${MADE_KO[d.made_by] ?? d.made_by} · 블록 ${d.block_count}`,
    kids: kids.get(d.design_id)?.length ?? 0,
    design: d,
  });
  const columns: Column[] = [
    {
      title: '가구',
      items: families.map((f) => {
        const n = kids.get(`family:${f}`)?.length ?? 0;
        return { key: f, label: FAMILY_KO[f] ?? f, meta: `기본 설계 ${n}개`, kids: n };
      }),
    },
  ];
  if (path[0]) columns.push({ title: '기본 설계', items: (kids.get(`family:${path[0]}`) ?? []).map(item) });
  for (let i = 1; i < path.length; i++) {
    const children = kids.get(path[i]) ?? [];
    if (children.length) columns.push({ title: `${designName(path[i])}에서 만든 것`, items: children.map(item) });
  }

  const choose = (col: number, key: string, design?: DesignSummary) => {
    setPath([...path.slice(0, col), key]); // 앞 열을 다시 고르면 그 오른쪽 열은 닫힌다
    if (design) onPick(design.design_id);
  };

  return (
    <div className="miller" ref={box}>
      {columns.map((c, ci) => (
        <div key={`${ci}-${c.title}`} className="mcol">
          <div className="mcol-title">{c.title}</div>
          {c.items.map((it) => (
            <button
              key={it.key}
              type="button"
              className={`mitem${path[ci] === it.key ? ' selected' : ''}`}
              onClick={() => choose(ci, it.key, it.design)}
              title={it.design ? it.design.design_id : undefined}
            >
              <span className="mitem-name">
                {it.label}
                {onRobot === it.key && <span className="tag ok">로봇</span>}
              </span>
              {it.meta && (
                <span className="mitem-meta">
                  {it.meta}
                  {it.design && it.kids > 0 && <b> · {it.kids} ▸</b>}
                </span>
              )}
            </button>
          ))}
        </div>
      ))}
    </div>
  );
}
