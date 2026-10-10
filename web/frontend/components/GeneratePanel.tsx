'use client';
// AI 설계 만들기 패널(W112 · W163) — 글상자 → AI 가 고른 모양(Template) 확인 · 바꾸기 → 후보 3개 3D + 검사 결과 → 사람이 1개 고름 → 저장
// (web/README 4-A · IRD 8.3 · E-84 ③). 로봇을 움직이지 않는다 — 저장까지만. 로봇에 보내는 것은 왼쪽 [설계 선택] → [출발](사람, SR-09).
// 진행: /ws {type: gen} 알림이 오면 job 을 GET 으로 다시 받는다(알림이 몰려와도 빠짐없이). 알림이 끊겨도 만드는 동안 1초마다 다시 받는다.
import { useEffect, useMemo, useState } from 'react';
import * as api from '@/lib/api';
import type { GenCandidate, GenJob, Rules, TemplateChoice } from '@/lib/types';
import type { Robot } from '@/lib/ws';
import { roleItems } from './DesignView';
import Preview3D from './Preview3D';

const POLL_MS = 1000; // /ws 알림이 끊겼을 때를 위한 다시 받기 간격(만드는 동안만)
const MAX_TRIES = 3; // 처음 + 다시 2번(SDD 6.6)
const FAIL_KO: Record<string, string> = {
  OUT_OF_SCOPE: '지금 만들 수 있는 모양이 아니에요',
  GEN_FAILED: '설계를 만들지 못했어요',
  TIMEOUT: '검사 응답이 늦어요',
  ERROR: '오류가 났어요',
};

interface Props {
  robot: Robot;
  rules: Rules | null;
  dark: boolean;
  onSaved: (designId: string) => void; // 저장한 설계를 오른쪽 3D 에 띄운다
  onLog: (kind: string, text: string) => void;
}

export default function GeneratePanel({ robot, rules, dark, onSaved, onLog }: Props) {
  const [text, setText] = useState('');
  const [busy, setBusy] = useState(false); // 버튼 하나의 답을 기다리는 중(모양 고르기 · 시작 · 저장)
  const [choice, setChoice] = useState<TemplateChoice | null>(null);
  const [template, setTemplate] = useState('');
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

  const reset = () => {
    setChoice(null);
    setTemplate('');
    setJob(null);
    setError('');
  };

  const askTemplate = async () => {
    const t = text.trim();
    if (!t) return;
    reset();
    setBusy(true);
    const r = await api.chooseTemplate(t);
    setBusy(false);
    if (!r.data) return setError(r.error ?? '웹 서버 답이 없어요');
    if (!r.data.success) return setError(`${FAIL_KO[r.data.code ?? ''] ?? r.data.code} — ${r.data.message ?? ''}`);
    setChoice(r.data);
    setTemplate(r.data.template ?? '');
    onLog('gen', r.data.template ? `AI가 고른 모양: ${r.data.template}` : 'AI: 맞는 모양이 없어요');
  };

  const start = async () => {
    setBusy(true);
    setError('');
    const t = text.trim();
    const r = await api.startGenerate(t, template);
    setBusy(false);
    if (!r.data) return setError(r.error ?? '웹 서버 답이 없어요');
    setJob({ job_id: r.data.job_id, state: 'running', text: t, template, candidates: [] });
    onLog('gen', `후보 만들기 시작(${template})`);
  };

  const pick = async (index: number) => {
    if (!jobId) return;
    setBusy(true);
    setError('');
    const r = await api.pickCandidate(jobId, index);
    setBusy(false);
    if (!r.data) return setError(r.error ?? '웹 서버 답이 없어요');
    const j = await api.getGenJob(jobId);
    if (j) setJob(j);
    onSaved(r.data.design_id);
    onLog('gen', `저장: ${r.data.design_id}(부모 ${r.data.parent_id ?? '없음'})`);
  };

  const label = (id?: string | null) => choice?.templates?.find((x) => x.id === id)?.label ?? id ?? '';

  return (
    <section className="panel gen-panel">
      <div className="panel-head">
        <h2>AI 설계 만들기</h2>
        <span className="muted">의자 · 책상만 · 저장까지만 해요 — 로봇은 [설계 선택] · [출발]로</span>
        {(choice || job || error) && (
          <button className="head-btn" onClick={reset} disabled={busy || running}>
            새로 만들기
          </button>
        )}
      </div>

      <div className="gen-input">
        <textarea
          value={text}
          rows={2}
          placeholder="예: 다리를 한 층 더 높인 벤치를 만들어 줘"
          onChange={(e) => setText(e.target.value)}
          disabled={busy || running}
        />
        <button className="primary" onClick={askTemplate} disabled={busy || running || !text.trim()}>
          {busy && !choice && !job ? '모양 고르는 중…' : '만들기'}
        </button>
      </div>
      {error && <p className="result bad">{error}</p>}

      {choice && !job && (
        <div className="gen-confirm">
          {choice.template ? (
            <p>
              AI가 고른 모양: <b>{label(choice.template)}</b> <span className="muted">({choice.template})</span> — {choice.reason}
            </p>
          ) : (
            <p className="bad-text">지금 만들 수 있는 모양이 아니에요 — {choice.reason} 꼭 만들려면 아래에서 모양을 직접 고르세요.</p>
          )}
          <div className="gen-confirm-row">
            <label>
              모양{' '}
              <select value={template} onChange={(e) => setTemplate(e.target.value)} disabled={busy}>
                {!template && <option value="">— 고르세요 —</option>}
                {choice.templates?.map((x) => (
                  <option key={x.id} value={x.id}>
                    {x.label}
                  </option>
                ))}
              </select>
            </label>
            <button className="primary" onClick={start} disabled={busy || !template}>
              이 모양으로 후보 3개 만들기
            </button>
          </div>
        </div>
      )}

      {job && <JobView job={job} rules={rules} dark={dark} canPick={job.state === 'ready' && !busy} onPick={pick} />}
    </section>
  );
}

/** 생성 작업 하나 — 진행 한 줄 · 참고한 설계 · 후보 3개 */
function JobView({ job, rules, dark, canPick, onPick }: { job: GenJob; rules: Rules | null; dark: boolean; canPick: boolean; onPick: (i: number) => void }) {
  const again = (job.attempt ?? 1) > 1 ? ` (다시 만들기 ${job.attempt}/${MAX_TRIES})` : '';
  let status: string;
  if (job.state === 'running') {
    if (job.candidates.length) status = '후보를 검사하는 중…' + again;
    else if (job.reading?.length) status = `참고 설계를 읽었어요(${job.reading.join(', ')}) — 후보를 쓰는 중…` + again;
    else status = '디자인 만드는 중 — 참고할 설계를 고르는 중…';
  } else if (job.state === 'ready') status = `합격한 후보 중 하나를 고르세요 · ${job.elapsed_s ?? '?'}초`;
  else if (job.state === 'saved') status = `저장됨: ${job.saved?.design_id} — 오른쪽 설계 3D 에 보여요. 로봇에 보내려면 왼쪽 [설계 선택]`;
  else status = `${FAIL_KO[job.code ?? ''] ?? job.code} — ${job.message ?? ''}`;

  return (
    <>
      <p className={`result ${job.state === 'failed' ? 'bad' : job.state === 'running' ? '' : 'ok'}`}>{status}</p>
      {job.reference && job.reference.length > 0 && <p className="muted">AI가 참고한 설계: {job.reference.join(', ')}</p>}
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
  const items = useMemo(() => roleItems(cand.blocks.blocks), [cand.blocks.blocks]);
  let verdict = '검사 중…';
  if (ck?.ok) verdict = `합격 · 최소 여유 ${ck.min_margin_mm ?? '–'} mm`;
  else if (ck) verdict = `불합격 — ${ck.errors[0]?.detail ?? ck.reason}${ck.errors.length > 1 ? ` 외 ${ck.errors.length - 1}개` : ''}`;
  return (
    <div className={`cand${ck && !ck.ok ? ' failed' : ''}${picked ? ' picked' : ''}`}>
      <div className="cand-head">
        <b>후보 {index + 1}</b>
        <span className="muted">블록 {cand.blocks.blocks.length}개</span>
      </div>
      {rules ? (
        <Preview3D items={items} blockMm={rules.block_size_mm} frameKey={`${cand.blocks.design_id}-${index}-${cand.blocks.blocks.length}`} dark={dark} />
      ) : (
        <div className="view3d-empty">블록 크기를 아직 못 받았어요</div>
      )}
      <p className="cand-idea">{cand.idea}</p>
      <p className={ck ? (ck.ok ? 'ok-text' : 'bad-text') : 'muted'}>{verdict}</p>
      <button className={ck?.ok ? 'primary' : ''} disabled={!canPick || !ck?.ok || picked} onClick={onPick}>
        {picked ? '저장됨' : '이걸로'}
      </button>
    </div>
  );
}
