'use client';
// AI 설계 만들기 패널(W112 · W163) — 글상자 → AI 가 고른 모양(Template) 확인 · 바꾸기 → 후보 3개 3D + 검사 결과 → 사람이 1개 고름 → 저장
// (web/README 4-A · IRD 8.3 · E-84 ③). 모양은 기본 설계 4개를 작은 3D 카드로 보여 주고 누르면 고른다(AI 추천 표시).
// 로봇을 움직이지 않는다 — 저장까지만. 로봇에 보내는 것은 설계 3D 의 [이 설계로 조립 준비] → 왼쪽 [출발](사람, SR-09).
// 진행: /ws {type: gen} 알림이 오면 job 을 GET 으로 다시 받는다(알림이 몰려와도 빠짐없이). 알림이 끊겨도 만드는 동안 1초마다 다시 받는다.
import { useEffect, useMemo, useState } from 'react';
import * as api from '@/lib/api';
import { checkReasonKo, designName } from '@/lib/names';
import type { Design, GenCandidate, GenJob, Rules, TemplateChoice } from '@/lib/types';
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
    if (!r.data.success) {
      // AI 가 모양을 못 골라도(크레딧 · 네트워크) 사람이 카드에서 직접 고를 수 있게 목록은 보여 준다
      setError(`${FAIL_KO[r.data.code ?? ''] ?? r.data.code} — ${r.data.message ?? ''}`);
      if (r.data.templates?.length) setChoice({ ...r.data, template: null });
      return;
    }
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
        <h2 title="의자 · 책상만 만들어요. 저장까지만 하고, 조립은 아래 [이 설계로 조립 준비] → 왼쪽 [출발]">AI 설계 만들기</h2>
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
            <p title={choice.reason}>
              AI 추천: <b>{title(label(choice.template))}</b> <span className="muted">— 다른 모양을 눌러 바꿀 수 있어요</span>
            </p>
          ) : choice.success ? (
            <p className="bad-text" title={choice.reason}>지금 만들 수 있는 모양이 아니에요 — 꼭 만들려면 직접 고르세요</p>
          ) : (
            <p>AI가 모양을 고르지 못했어요 — 아래에서 직접 고르세요.</p>
          )}
          <TemplateCards
            templates={choice.templates ?? []}
            aiPick={choice.template ?? null}
            value={template}
            onChange={setTemplate}
            rules={rules}
            dark={dark}
            disabled={busy}
          />
          <div className="gen-confirm-row">
            <button className="primary" onClick={start} disabled={busy || !template}>
              {template ? `'${title(label(template))}' 모양으로 후보 3개 만들기` : '모양을 고르세요'}
            </button>
          </div>
        </div>
      )}

      {job && <JobView job={job} rules={rules} dark={dark} canPick={job.state === 'ready' && !busy} onPick={pick} />}
    </section>
  );
}

/** Template 설명 '벤치 — 등받이 없는 의자(…) (기본 설계 …)' 에서 앞 이름만 */
const title = (labelText: string) => labelText.split(' — ')[0];

/** 모양(Template) 고르기 카드 — 기본 설계(V000) 작은 3D · 이름 · 설명. 누르면 고름(E-84 ③ 사용자 확인 — W163) */
function TemplateCards(props: {
  templates: { id: string; label: string }[];
  aiPick: string | null;
  value: string;
  onChange: (id: string) => void;
  rules: Rules | null;
  dark: boolean;
  disabled: boolean;
}) {
  const { templates, aiPick, value, onChange, rules, dark, disabled } = props;
  const [designs, setDesigns] = useState<Record<string, Design | null>>({});
  useEffect(() => {
    let live = true;
    for (const t of templates) {
      api.getDesign(`${t.id}_V000`).then((d) => live && setDesigns((m) => ({ ...m, [t.id]: d })));
    }
    return () => {
      live = false;
    };
  }, [templates]);
  return (
    <div className="tpl-cards">
      {templates.map((t) => {
        const d = designs[t.id];
        const [name, ...rest] = t.label.split(' — ');
        const desc = rest.join(' — ').split(' (기본 설계')[0]; // 블록 수 · 크기 글은 빼고 생김새만
        return (
          <div
            key={t.id}
            role="button"
            tabIndex={0}
            aria-pressed={value === t.id}
            className={`tpl${value === t.id ? ' selected' : ''}${disabled ? ' disabled' : ''}`}
            onClick={() => !disabled && onChange(t.id)}
            onKeyDown={(e) => (e.key === 'Enter' || e.key === ' ') && !disabled && onChange(t.id)}
          >
            <div className="tpl-head">
              <b>{name}</b>
              {aiPick === t.id && <span className="tag ok">AI 추천</span>}
            </div>
            {d && rules ? (
              <TemplatePreview design={d} rules={rules} dark={dark} />
            ) : (
              <div className="view3d-empty">{d === null ? '그림을 못 받았어요' : '그림 받는 중…'}</div>
            )}
            <small className="muted">{desc}</small>
          </div>
        );
      })}
    </div>
  );
}

function TemplatePreview({ design, rules, dark }: { design: Design; rules: Rules; dark: boolean }) {
  const items = useMemo(() => roleItems(design.blocks.blocks), [design]);
  return <Preview3D items={items} blockMm={rules.block_size_mm} frameKey={design.design_id} dark={dark} />;
}

/** 생성 작업 하나 — 진행 한 줄 · 참고한 설계 · 후보 3개 */
function JobView({ job, rules, dark, canPick, onPick }: { job: GenJob; rules: Rules | null; dark: boolean; canPick: boolean; onPick: (i: number) => void }) {
  const again = (job.attempt ?? 1) > 1 ? ` (다시 만들기 ${job.attempt}/${MAX_TRIES})` : '';
  let status: string;
  if (job.state === 'running') {
    if (job.candidates.length) status = '후보 검사 중…' + again;
    else if (job.reading?.length) status = '후보 만드는 중…' + again;
    else status = '참고할 설계 고르는 중…';
  } else if (job.state === 'ready') status = '합격한 후보 하나를 고르세요';
  else if (job.state === 'saved') status = `${designName(job.saved?.design_id)} 저장했어요 — 아래에서 [이 설계로 조립 준비]`;
  else status = `${FAIL_KO[job.code ?? ''] ?? job.code} — ${job.message ?? ''}`;

  return (
    <>
      <p className={`result ${job.state === 'failed' ? 'bad' : job.state === 'running' ? '' : 'ok'}`}>{status}</p>
      {job.reference && job.reference.length > 0 && (
        <p className="muted small">참고한 설계: {job.reference.map((r) => designName(r)).join(', ')}{job.elapsed_s ? ` · ${job.elapsed_s}초` : ''}</p>
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
  const items = useMemo(() => roleItems(cand.blocks.blocks), [cand.blocks.blocks]);
  let verdict = '검사 중…';
  if (ck?.ok) verdict = '합격';
  else if (ck) verdict = `불합격 · ${[...new Set(ck.errors.map((e) => checkReasonKo(e.detail ?? '')))].slice(0, 2).join(', ') || '규칙 위반'}`;
  const detail = ck ? (ck.ok ? `최소 여유 ${ck.min_margin_mm ?? '–'} mm` : ck.errors.map((e) => e.detail).join('\n')) : undefined;
  return (
    <div className={`cand${ck && !ck.ok ? ' failed' : ''}${picked ? ' picked' : ''}`}>
      <div className="cand-head">
        <b>후보 {index + 1}</b>
        <span className="muted small">블록 {cand.blocks.blocks.length}개</span>
      </div>
      {rules ? (
        <Preview3D items={items} blockMm={rules.block_size_mm} frameKey={`${cand.blocks.design_id}-${index}-${cand.blocks.blocks.length}`} dark={dark} />
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
