'use client';
// AI 설계 만들기 패널(W112 · W163) — 위 [설계 목록]에서 고른 설계를 바탕(= 부모)으로 글 · 음성 요청 → 후보 3개 3D + 검사 결과 → 사람이 1개 고름 → 저장
// (web/README 4-A · IRD 8.3 · 10/13 PL 화면 결정: AI 모양 추천 없음 — 사람이 설계를 고르니까, Template = 고른 설계 앞부분).
// 글: 글 창에 쓰고 [AI 생성]. 음성: 글 창 옆 마이크 → voice.py 가 듣고 문장을 보내면(/ws intent request_design) 글 창에 넣고 바로 생성
// ([AI 생성] 안 누름 — 사람이 문장 확인하는 단계 없앰). 설계를 고르기 전에는 둘 다 막는다.
// 로봇을 움직이지 않는다 — 저장까지만. 조립은 설계 3D 의 [조립](사람, SR-09).
// 진행: /ws {type: gen} 알림이 오면 job 을 GET 으로 다시 받는다(알림이 몰려와도 빠짐없이). 알림이 끊겨도 만드는 동안 1초마다 다시 받는다.
import { useEffect, useMemo, useRef, useState } from 'react';
import * as api from '@/lib/api';
import { checkReasonKo, designName } from '@/lib/names';
import type { GenCandidate, GenJob, Intent, Rules } from '@/lib/types';
import type { Robot } from '@/lib/ws';
import { roleItems } from './DesignView';
import Preview3D from './Preview3D';

const POLL_MS = 1000; // /ws 알림이 끊겼을 때를 위한 다시 받기 간격(만드는 동안만)
const LISTEN_MS = 20000; // 마이크를 누른 뒤 문장을 기다리는 시간(녹음 + Whisper) — 넘으면 voice.py 가 안 도는 것으로 보고 안내
const MAX_TRIES = 3; // 처음 + 모자란 만큼 더 2번(10/11 PL N3 · A5)
const MAX_CANDS = 3; // 후보 수(E-67)
const FAIL_KO: Record<string, string> = {
  OUT_OF_SCOPE: '지금 만들 수 있는 모양이 아니에요',
  GEN_FAILED: '설계를 만들지 못했어요',
  TEMPLATE_SHAPE: '만들 수 없는 모양이에요', // 구성 형태 어긋남(21번) — 취소하고 다시 요청(C5)
  TEMPLATE_RANGE: '지금 만들 수 있는 범위가 아니에요', // 치수 범위 밖(21번) — backend 글에 범위가 붙음(C6)
  REQUEST_INVALID: '검사 요청 오류(웹)', // 웹 코드 쪽 잘못 — 다시 만들지 않음(D10)
  TIMEOUT: '검사 응답이 늦어요',
  ERROR: '오류가 났어요',
};
const OWN_MESSAGE = new Set(['TEMPLATE_SHAPE', 'TEMPLATE_RANGE', 'REQUEST_INVALID']); // backend 글이 안내 전부라 앞 이름을 겹쳐 붙이지 않음

interface Props {
  robot: Robot;
  baseId: string | null; // 위 [설계 목록]에서 고른 설계(없으면 로봇에 선택된 설계) — 새 설계의 부모
  rules: Rules | null;
  dark: boolean;
  onSaved: (designId: string) => void; // 저장한 설계를 위 설계 3D 에 띄운다
  onLog: (kind: string, text: string) => void;
}

export default function GeneratePanel({ robot, baseId, rules, dark, onSaved, onLog }: Props) {
  const [text, setText] = useState('');
  const [busy, setBusy] = useState(false); // 버튼 하나의 답을 기다리는 중(시작 · 저장 · 마이크)
  const [listening, setListening] = useState(false);
  const listenFrom = useRef<Intent | null | undefined>(undefined); // 마이크를 누를 때 이미 있던 음성 알림 — 그 뒤에 온 것만 받는다
  const [madeBy, setMadeBy] = useState<'web' | 'voice'>('web');
  const [job, setJob] = useState<GenJob | null>(null);
  const [error, setError] = useState('');
  const jobId = job?.job_id;
  const running = job?.state === 'running';

  useEffect(() => {
    // 생성 알림이 오면 이 job 을 다시 받는다(다른 job 알림은 무시)
    const g = robot.gen;
    if (!g || !jobId || g.job_id !== jobId) return;
    api.getGenJob(jobId).then((j) => j && setJob(j));
  }, [robot.gen, jobId]);

  useEffect(() => {
    // 만드는 동안 1초마다 — /ws 가 끊겨도 화면이 '만드는 중'에 머물지 않게
    if (!running || !jobId) return;
    const t = setInterval(() => api.getGenJob(jobId).then((j) => j && setJob(j)), POLL_MS);
    return () => clearInterval(t);
  }, [running, jobId]);

  const start = async (t: string, by: 'web' | 'voice') => {
    if (!baseId || !t) return;
    setBusy(true);
    setError('');
    setJob(null);
    setMadeBy(by);
    const r = await api.startGenerate(t, baseId);
    setBusy(false);
    if (!r.data) return setError(r.error ?? '웹 서버 답이 없어요');
    setJob({ job_id: r.data.job_id, state: 'running', text: t, base_id: baseId, template: '', candidates: [] });
    onLog('gen', `후보 만들기 시작 — 바탕 ${designName(baseId)}${by === 'voice' ? '(음성)' : ''}`);
  };

  // 음성: 마이크를 누른 뒤 온 request_design 문장만 받아 글 창에 넣고 바로 생성(사람 확인 단계 없음 — 10/13 PL)
  const intent = robot.intent;
  useEffect(() => {
    if (!listening || intent === listenFrom.current || !intent) return;
    if (intent.intent !== 'request_design' || !intent.text?.trim()) return;
    setListening(false);
    setText(intent.text);
    void start(intent.text.trim(), 'voice'); // start 는 deps 에 넣지 않는다 — 문장이 온 그 순간의 바탕 설계로 한 번만
  }, [intent, listening]);
  useEffect(() => {
    if (!listening) return;
    const t = setTimeout(() => {
      setListening(false);
      setError('음성을 못 받았어요 — 마이크 · 음성 프로그램(voice.py)을 확인하고 다시 눌러 주세요');
    }, LISTEN_MS);
    return () => clearTimeout(t);
  }, [listening]);

  const listen = async () => {
    setBusy(true);
    setError('');
    listenFrom.current = robot.intent;
    const r = await api.voiceListen();
    setBusy(false);
    if (!r.success) return setError(`마이크를 켜지 못했어요 — ${r.message ?? r.reason}`);
    setListening(true);
    onLog('voice', '듣는 중 — 만들고 싶은 것을 말하세요');
  };

  const pick = async (index: number) => {
    if (!jobId) return;
    setBusy(true);
    setError('');
    const r = await api.pickCandidate(jobId, index, madeBy);
    setBusy(false);
    if (!r.data) return setError(r.error ?? '웹 서버 답이 없어요');
    const j = await api.getGenJob(jobId);
    if (j) setJob(j);
    onSaved(r.data.design_id);
    onLog('gen', `저장: ${r.data.design_id}(부모 ${r.data.parent_id ?? '없음'})`);
  };

  const blocked = busy || running || listening;
  return (
    <section className="panel gen-panel">
      <div className="panel-head">
        <h2 title="고른 설계를 바탕으로 의자 · 책상만 만들어요. 저장까지만 하고, 조립은 위 설계 3D 의 [조립]">AI 설계 만들기</h2>
        <span className="head-sub muted">{baseId ? `바탕: ${designName(baseId)}` : ''}</span>
        {(job || error) && (
          <button className="head-btn" onClick={() => (setJob(null), setError(''))} disabled={blocked}>
            새로 만들기
          </button>
        )}
      </div>

      {!baseId && <p className="hint">먼저 위 [설계 목록]에서 바탕이 될 설계를 고르세요</p>}
      <div className="gen-input">
        <textarea
          value={text}
          rows={2}
          placeholder="예: 다리를 한 층 더 높여 줘"
          onChange={(e) => setText(e.target.value)}
          disabled={blocked || !baseId}
        />
        <button className="mic" onClick={listen} disabled={blocked || !baseId} aria-label="음성으로 요청"
          title="누르고 만들고 싶은 것을 말하세요 — 문장이 글 창에 들어가고 바로 만들기 시작해요">
          {listening ? '듣는 중…' : '🎤 말하기'}
        </button>
        <button className="primary" onClick={() => start(text.trim(), 'web')} disabled={blocked || !baseId || !text.trim()}>
          {busy && !job ? '시작하는 중…' : 'AI 생성'}
        </button>
      </div>
      {error && <p className="result bad">{error}</p>}

      {job && <JobView job={job} rules={rules} dark={dark} canPick={job.state === 'ready' && !busy} onPick={pick} />}
    </section>
  );
}

/** 생성 작업 하나 — 진행 한 줄 · 참고한 설계 · 통과한 후보(끝난 뒤에만 그림 — 10/11 PL A5) */
function JobView({ job, rules, dark, canPick, onPick }: { job: GenJob; rules: Rules | null; dark: boolean; canPick: boolean; onPick: (i: number) => void }) {
  const again = (job.attempt ?? 1) > 1 ? ` (모자란 만큼 더 만들기 ${job.attempt}/${MAX_TRIES})` : '';
  const code = job.code ?? '';
  let status: string;
  if (job.state === 'running') {
    if (job.count) status = `후보 검사 중… 통과 ${job.passed ?? 0}/${MAX_CANDS}` + again;
    else status = (job.reading?.length ? '참고 설계를 읽고 ' : '') + '후보 만드는 중…' + again;
  } else if (job.state === 'ready') {
    const n = job.candidates.length;
    status = n < MAX_CANDS ? `통과한 후보 ${n}개(${MAX_CANDS}개를 다 못 채웠어요) — 하나를 고르세요` : '통과한 후보 하나를 고르세요';
  } else if (job.state === 'saved') status = `${designName(job.saved?.design_id)} 저장했어요 — 위 설계 3D 에서 [조립]`;
  else if (OWN_MESSAGE.has(code)) status = job.message || FAIL_KO[code];
  else status = `${FAIL_KO[code] ?? code} — ${job.message ?? ''}`;

  return (
    <>
      <p className={`result ${job.state === 'failed' ? 'bad' : job.state === 'running' ? '' : 'ok'}`}>{status}</p>
      {job.state !== 'running' && job.base_id && (
        <p className="muted small">
          바탕 {designName(job.base_id)}
          {job.reference?.length ? ` · 참고한 설계 ${job.reference.map((r) => designName(r)).join(', ')}` : ''}
          {job.elapsed_s ? ` · ${job.elapsed_s}초` : ''}
        </p>
      )}
      {job.candidates.length > 0 && (
        <div className="cands">
          {job.candidates.map((c, i) => (
            <CandidateCard
              key={`${job.design_id ?? ''}-${job.attempt ?? 1}-${i}`}
              cand={c}
              index={i}
              rules={rules}
              dark={dark}
              canPick={canPick}
              picked={job.saved?.index === i}
              onPick={() => onPick(i)}
            />
          ))}
        </div>
      )}
    </>
  );
}

/** 후보 하나 — 3D · AI 의 한 줄 생각 · 검사 결과(불합격은 회색, 고를 수 없음) */
function CandidateCard(props: { cand: GenCandidate; index: number; rules: Rules | null; dark: boolean; canPick: boolean; picked: boolean; onPick: () => void }) {
  const { cand, index, rules, dark, canPick, picked, onPick } = props;
  const ck = cand.check;
  const items = useMemo(() => roleItems(cand.parts.parts), [cand.parts.parts]);
  let verdict = '검사 중…';
  if (ck?.ok) verdict = '합격';
  else if (ck) verdict = `불합격 · ${[...new Set(ck.errors.map((e) => checkReasonKo(e.detail ?? '')))].slice(0, 2).join(', ') || '규칙 위반'}`;
  const detail = ck ? (ck.ok ? `최소 여유 ${ck.min_margin_mm ?? '–'} mm` : ck.errors.map((e) => e.detail).join('\n')) : undefined;
  return (
    <div className={`cand${ck && !ck.ok ? ' failed' : ''}${picked ? ' picked' : ''}`}>
      <div className="cand-head">
        <b>후보 {index + 1}</b>
        <span className="muted small">블록 {cand.parts.parts.length}개</span>
      </div>
      {rules ? (
        <Preview3D items={items} blockMm={rules.block_size_mm} frameKey={`${cand.parts.design_id}-${index}-${cand.parts.parts.length}`} dark={dark} />
      ) : (
        <div className="view3d-empty">블록 크기를 아직 못 받았어요</div>
      )}
      <p className="cand-idea">{cand.idea}</p>
      <p className={`verdict ${ck ? (ck.ok ? 'ok-text' : 'bad-text') : 'muted'}`} title={detail}>{verdict}</p>
      <button className={ck?.ok ? 'primary' : ''} disabled={!canPick || !ck?.ok || picked} onClick={onPick}>
        {picked ? '저장됨' : '이걸로'}
      </button>
    </div>
  );
}
