'use client';
// 메인 페이지 — 위 띠(제목 · 연결 점 · 진행도 가운데 · 다크 모드) · 끊김 배너 · 왼쪽(로봇 패널 · 손목 카메라) ·
// 오른쪽(설계 3D — 목록에서 고르고 [조립] · 그 아래 고른 설계를 바탕으로 AI 설계 만들기, 10/13 PL) · 스캔 비교(SCAN_REVIEW 때만 — W112) ·
// 기록(접어 둠 — 마지막 한 줄만).
// 정지 버튼이 있는 로봇 패널은 왼쪽 맨 위에 둔다(늘 보임).
import { useEffect, useState } from 'react';
import { ConnectionBanners, ConnectionDots } from '@/components/ConnectionBadge';
import DesignView from '@/components/DesignView';
import GeneratePanel from '@/components/GeneratePanel';
import ProgressTop from '@/components/ProgressTop';
import RobotPanel from '@/components/RobotPanel';
import ScanReview from '@/components/ScanReview';
import ScanRecommend from '@/components/ScanRecommend';
import ThemeToggle from '@/components/ThemeToggle';
import WristCamera from '@/components/WristCamera';
import * as api from '@/lib/api';
import { setTemplateNames } from '@/lib/names';
import { useTheme } from '@/lib/theme';
import type { Rules } from '@/lib/types';
import { useRobot } from '@/lib/ws';

export default function Home() {
  const [robot, addLog] = useRobot();
  const [dark, setDark] = useTheme();
  const [picked, setPicked] = useState<string | null>(null);
  const [rules, setRules] = useState<Rules | null>(null);
  const [named, setNamed] = useState(false); // 모양 이름을 받았나 — 받으면 다시 그려 설계 ID 대신 '벤치 V001'
  const robotDesign = robot.state?.design_id ?? null;

  useEffect(() => {
    if (robot.ws && !rules) api.getRules().then(setRules); // backend 가 늦게 떠도 붙는 순간 다시 받는다
    if (robot.ws && !named)
      api.getTemplates().then((t) => {
        if (!t) return;
        setTemplateNames(t);
        setNamed(true);
      });
  }, [robot.ws, rules, named]);
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
            <DesignView robot={robot} viewId={picked ?? robotDesign} rules={rules} dark={dark} onPick={setPicked} onLog={addLog} />
            <GeneratePanel robot={robot} baseId={picked ?? robotDesign} rules={rules} dark={dark} onSaved={setPicked} onLog={addLog} />
          </div>
        </div>

        {robot.state?.state === 'SCAN_REVIEW' && robot.scan && (
          <>
            <ScanReview robot={robot} rules={rules} dark={dark} onSaved={setPicked} onLog={addLog} />
            <section className="panel">
              <h2 title="저장된 설계 중 스캔과 가장 비슷한 것 — GPT 없이 바로 조립(플랜 B)">추천 설계</h2>
              <ScanRecommend robot={robot} onView={setPicked} onLog={addLog} />
            </section>
          </>
        )}

        <details className="panel log-panel">
          <summary>
            <h2>기록</h2>
            <span className="muted small">{robot.log[0] ? robot.log[0].text : '아직 없음'}</span>
          </summary>
          <ol className="log">
            {robot.log.map((l, i) => (
              <li key={`${l.t}-${i}`} className={`log-${l.kind}`}>
                <time>{new Date(l.t).toTimeString().slice(0, 8)}</time> {l.text}
              </li>
            ))}
            {robot.log.length === 0 && <li className="muted">아직 없음</li>}
          </ol>
        </details>
      </main>
    </>
  );
}
