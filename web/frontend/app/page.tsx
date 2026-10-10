'use client';
// 메인 페이지 — 위 띠(제목 · 연결 점 · 진행도 가운데 · 다크 모드) · 끊김 배너 · 왼쪽(로봇 패널 · 손목 카메라) ·
// 오른쪽(AI 설계 만들기 W112 · 설계 3D) · 상태 로그. 정지 버튼이 있는 로봇 패널은 왼쪽 맨 위에 둔다(늘 보임). 스캔 비교(W116)는 뒤에 붙는다.
import { useEffect, useState } from 'react';
import { ConnectionBanners, ConnectionDots } from '@/components/ConnectionBadge';
import DesignView from '@/components/DesignView';
import GeneratePanel from '@/components/GeneratePanel';
import ProgressTop from '@/components/ProgressTop';
import RobotPanel from '@/components/RobotPanel';
import ThemeToggle from '@/components/ThemeToggle';
import WristCamera from '@/components/WristCamera';
import * as api from '@/lib/api';
import { useTheme } from '@/lib/theme';
import type { Rules } from '@/lib/types';
import { useRobot } from '@/lib/ws';

export default function Home() {
  const [robot, addLog] = useRobot();
  const [dark, setDark] = useTheme();
  const [picked, setPicked] = useState<string | null>(null);
  const [rules, setRules] = useState<Rules | null>(null);
  const robotDesign = robot.state?.design_id ?? null;

  useEffect(() => {
    if (robot.ws && !rules) api.getRules().then(setRules); // backend 가 늦게 떠도 붙는 순간 다시 받는다
  }, [robot.ws, rules]);
  useEffect(() => {
    setPicked(null); // 로봇 설계가 바뀌면(설계 선택 · 음성) 3D 도 로봇 설계를 따라간다
  }, [robotDesign]);

  return (
    <>
      <header className="topbar">
        <div className="top-left">
          <h1>D2 젠가 가구</h1>
          <ConnectionDots robot={robot} />
        </div>
        <ProgressTop robot={robot} />
        <ThemeToggle dark={dark} onChange={setDark} />
      </header>

      <main>
        <ConnectionBanners robot={robot} />

        <div className="layout">
          <div className="col">
            <RobotPanel robot={robot} onLog={addLog} />
            <WristCamera robot={robot} />
          </div>
          <div className="col">
            <GeneratePanel robot={robot} rules={rules} dark={dark} onSaved={setPicked} onLog={addLog} />
            <DesignView robot={robot} viewId={picked ?? robotDesign} rules={rules} dark={dark} onPick={setPicked} onLog={addLog} />
          </div>
        </div>

        {robot.state?.state === 'SCAN_REVIEW' && robot.scan && (
          <section className="panel">
            <h2>스캔 결과</h2>
            <p>
              블록 {robot.scan.blocks.blocks.length}개(가려져 추정한 블록 {robot.scan.inferred_count}개). 비교 화면 · [그대로 저장] · [AI로 고치기]는 스캔 비교
              페이지(W116)에서 붙습니다. 지금은 [다시 스캔] · [취소]만 됩니다.
            </p>
          </section>
        )}

        <section className="panel">
          <h2>상태 로그</h2>
          <ol className="log">
            {robot.log.map((l, i) => (
              <li key={`${l.t}-${i}`} className={`log-${l.kind}`}>
                <time>{new Date(l.t).toTimeString().slice(0, 8)}</time> {l.text}
              </li>
            ))}
            {robot.log.length === 0 && <li className="muted">아직 없음</li>}
          </ol>
        </section>
      </main>
    </>
  );
}
