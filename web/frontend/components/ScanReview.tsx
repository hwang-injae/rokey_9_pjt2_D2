'use client';
// 스캔 칸(W112 · W116 — IRD 8.4 · E-80 · E-82): 스캔을 누른 뒤 이동 · 촬영 · 계산 · 결과 확인까지 한 자리.
// 스캔하는 동안은 점군 미리보기(scan_preview — live 가 오면 그것, 아니면 자세마다 모은 capture 누적본)가 계속 바뀌고, 계산이 끝나
// 결과 확인(SCAN_REVIEW)이 되면 같은 칸에 최종 점군 · 추론 설계가 나온다(10/13 황인재). 손목 실물 RGB(사진)는 보이지 않는다
// (10/13 16시 PL — 화질 · 각도 문제로 점군 + 설계도만).
// 결과 확인에서 사람이 [그대로 저장] · [다시 스캔] · [취소].
// [그대로 저장] = backend 가 AI 칸 채우기(W177) → 검사 → 저장(made_by scan). 저장되면 select_design(⑧)으로 출발 대기(설계 3D 의 [조립]),
// 실패면 backend 가 준 버튼만 남긴다(블록 수 초과 = [취소]만 · 그 밖 = [다시 스캔] · [취소] — 레시피 초안 7-1). [AI로 고치기]는 챌린지(10/14 뒤).
import { useEffect, useMemo, useRef, useState } from 'react';
import * as api from '@/lib/api';
import { designName } from '@/lib/names';
import { stateName } from '@/lib/names';
import type { Cmd, Rules, ScanPreview, ScanSaveJob } from '@/lib/types';
import type { Robot } from '@/lib/ws';
import CloudView from './CloudView';
import Preview3D, { type Item3D } from './Preview3D';

const POLL_MS = 1000; // /ws 알림이 끊겼을 때를 위한 다시 받기 간격(저장하는 동안만)
const MAX_TRIES = 3; // 처음 + AI 칸 다시 채우기 2번(design_gen SCAN_REFILLS)
const LIVE_FRESH_MS = 3000; // live 미리보기가 이 안에 왔으면 live 를, 아니면 capture 누적본을 그린다(live 는 1 ~ 2 Hz — W168)
const SCAN_STEPS = ['SCAN_MOVE', 'SCAN_CAPTURE', 'SCAN_INFER', 'SCAN_REVIEW'];
export const SCAN_STATES = SCAN_STEPS; // page 가 이 칸을 그릴 때
const SEEN = '#8f9bab'; // 카메라에 보인 블록
const GUESS = '#e8a33c'; // 가려져서 받침 규칙으로 추정한 블록(반투명)

interface Props {
  robot: Robot;
  rules: Rules | null;
  dark: boolean;
  onSaved: (designId: string) => void; // 저장한 설계를 오른쪽 설계 3D 에 띄운다
  onLog: (kind: string, text: string) => void;
}

/** 스캔 한 번(run_id)의 실시간 보기 → 비교 · 저장. 스캔 상태(SCAN_*)에서만 그린다(page).
 *  바깥 영향: 저장 요청 · hmi/command(select_design · scan · cancel) — 저장 · 버튼은 결과 확인 때만 */
export default function ScanReview({ robot, rules, dark, onSaved, onLog }: Props) {
  const st = robot.state;
  const live = st?.state !== 'SCAN_REVIEW';
  const runId = st?.run_id || robot.scanPreview?.run_id || robot.scan?.run_id || ''; // 이번 스캔(작업 관리자 run_id)
  const scan = robot.scan && (!live || robot.scan.run_id === runId) ? robot.scan : null; // 앞 스캔 결과는 섞지 않는다
  const [job, setJob] = useState<ScanSaveJob | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const [now, setNow] = useState(() => Date.now());
  const selected = useRef(''); // select_design 을 보낸 저장 작업(한 번만)
  const items: Item3D[] = useMemo(
    () => (scan?.blocks.blocks ?? []).map((b) => ({ x: b.x, y: b.y, z: b.z, ori: b.ori, color: b.inferred ? GUESS : SEEN, opacity: b.inferred ? 0.45 : 1 })),
    [scan],
  );
  const panel = useRef<HTMLElement>(null);
  useEffect(() => {
    // 새 스캔이 시작되면 이 칸으로 화면을 옮긴다 — 왼쪽 위 [스캔]을 누르면 칸이 아래에 생겨 안 보이므로(10/13 황인재)
    if (live) panel.current?.scrollIntoView({ behavior: 'smooth', block: 'start' });
    // live 는 deps 에 넣지 않는다 — 결과 확인으로 바뀔 때는 화면을 옮기지 않게
  }, [runId]);
  useEffect(() => {
    if (!live) return;
    const t = setInterval(() => setNow(Date.now()), 1000); // live 가 끊기면 capture 로 돌아가게
    return () => clearInterval(t);
  }, [live]);

  // 점군: 결과 확인이면 최종 점군(run_id 짝 — 다른 스캔 것은 안 보임, IRD 10.5), 없으면(또는 스캔 중) 미리보기 — live 가 3초 안에 왔으면 live, 아니면 capture 누적본
  const pv = robot.scanPreview?.run_id === runId ? robot.scanPreview : null;
  const preview: ScanPreview | undefined =
    pv?.live && now - pv.live.stamp * 1000 < LIVE_FRESH_MS && live ? pv.live : (pv?.capture ?? pv?.live);
  const finalCloud = scan?.cloud_path && robot.scanCloud?.run_id === scan.run_id ? robot.scanCloud : null; // cloud_path 없으면 안 띄움(E-67)
  const cloudUrl = !live && finalCloud ? api.scanCloudUrl(finalCloud.seq) : preview ? api.scanPreviewUrl(preview.kind, preview.seq) : null;
  const cloudKey = `${runId}-${!live && finalCloud ? 'final' : (pv?.capture?.index ?? 0)}`; // 자세마다 · 최종에서만 카메라를 다시 맞춘다
  const step = Math.max(0, SCAN_STEPS.indexOf(st?.state ?? ''));

  useEffect(() => {
    setJob(null); // 다시 스캔하면 새 비교
    setError('');
  }, [runId]);

  // 저장 작업 따라가기: /ws 알림이 오면 바로, 알림이 끊겨도 1초마다
  const jobId = job?.job_id;
  const running = job?.state === 'running';
  useEffect(() => {
    if (jobId && robot.scanSave?.job_id === jobId) api.getScanSave(jobId).then((j) => j && setJob(j));
  }, [robot.scanSave, jobId]);
  useEffect(() => {
    if (!jobId || !running) return;
    const t = setInterval(() => api.getScanSave(jobId).then((j) => j && setJob(j)), POLL_MS);
    return () => clearInterval(t);
  }, [jobId, running]);

  // 저장되면 그 설계로 조립 준비(IRD 8.4 ⑧ — 사람이 구조물을 치우고 설계 3D 의 [조립])
  useEffect(() => {
    const id = job?.state === 'saved' ? job.saved?.design_id : undefined;
    if (!id || selected.current === job?.job_id) return;
    selected.current = job!.job_id;
    onSaved(id);
    api.command('select_design', id).then((r) =>
      onLog('button', r.success ? `${designName(id)} 저장 · 조립 준비 — 구조물을 치우고 [조립]` : `조립 준비 실패 — ${r.message ?? r.reason}`),
    );
  }, [job, onSaved, onLog]);

  const save = async () => {
    if (!scan) return;
    setBusy(true);
    setError('');
    const r = await api.scanSave(scan.run_id);
    setBusy(false);
    if (!r.data) return setError(r.error ?? '웹 서버 답이 없어요');
    setJob({ job_id: r.data.job_id, state: 'running', run_id: scan.run_id });
    onLog('button', '스캔 저장 — AI가 역할 · 순서 · 잡기를 채우고 검사해요');
  };
  const send = async (label: string, cmd: Cmd) => {
    setBusy(true);
    const r = await api.command(cmd);
    setBusy(false);
    onLog('button', r.success ? label : `${label} 실패 — ${r.message ?? r.reason}`);
  };

  let status = '점군과 추론 설계를 비교하고 고르세요';
  if (live) status = st?.message || stateName(st?.state);
  else if (job?.state === 'running')
    status = 'AI가 역할 · 순서 · 잡기를 채우고 검사하는 중…' + ((job.attempt ?? 1) > 1 ? ` (다시 채우기 ${job.attempt}/${MAX_TRIES})` : '');
  else if (job?.state === 'saved') status = `${designName(job.saved?.design_id)}(으)로 저장했어요 — 구조물을 치우고 위 설계 3D 에서 [조립]`;
  else if (job?.state === 'failed') status = job.message || '저장하지 못했어요';
  const failed = job?.state === 'failed';
  const can = (a: 'cancel' | 'rescan') => !failed || (job?.actions ?? ['rescan', 'cancel']).includes(a);

  const poseText = preview?.pose_id ? `자세 ${preview.pose_id}` : '';
  const pointsText = preview?.points ? `점 ${preview.points.toLocaleString()}개` : '';
  return (
    <section className="panel scan-panel" ref={panel}>
      <div className="panel-head">
        <h2>스캔</h2>
        <ol className="steps" aria-label="스캔 단계">
          {['이동', '촬영', '계산', '결과 확인'].map((name, i) => (
            <li key={name} className={i < step ? 'done' : i === step ? 'now' : ''}>{name}</li>
          ))}
        </ol>
        <span className="muted small">
          {scan
            ? `블록 ${scan.blocks.blocks.length}개 · 가려져 추정 ${scan.inferred_count}개${scan.nearest_base ? ` · 가장 비슷한 기본 설계 ${designName(scan.nearest_base)}` : ''}`
            : [poseText, pointsText].filter(Boolean).join(' · ')}
        </span>
      </div>
      <div className="scan-grid">
        <figure>
          <figcaption>
            점군 {live ? <span className="muted small">— {preview?.kind === 'live' ? '지금 보이는 것' : '지금까지 모은 것'}</span> : null}
          </figcaption>
          {cloudUrl ? (
            <CloudView url={cloudUrl} frameKey={cloudKey} dark={dark} />
          ) : (
            <div className="view3d-empty scan-empty">{live ? '촬영하면 점군이 쌓여요' : '점군이 없어요'}</div>
          )}
        </figure>
        <figure>
          <figcaption>
            추론 설계 <span className="muted small">— 반투명 = 가려져서 추정한 블록</span>
          </figcaption>
          {scan && rules ? (
            <Preview3D items={items} blockMm={rules.block_size_mm} frameKey={scan.run_id} dark={dark} />
          ) : (
            <div className="view3d-empty scan-empty">
              {st?.state === 'SCAN_INFER' ? <span className="spin">블록 위치를 계산하는 중…</span> : live ? '촬영이 끝나면 계산해요' : '블록 크기를 아직 못 받았어요'}
            </div>
          )}
        </figure>
      </div>
      <p className={`result ${failed ? 'bad' : job?.state === 'saved' ? 'ok' : ''}`}>{status}</p>
      {error && <p className="bad-text">{error}</p>}
      {!live && scan && job?.state !== 'saved' && (
        <div className="send-row">
          {!failed && (
            <button className="primary" disabled={busy || running} onClick={save} title="AI가 역할 · 순서 · 잡기를 채우고 검사를 통과하면 새 설계로 저장해요">
              그대로 저장
            </button>
          )}
          {can('rescan') && (
            <button disabled={busy || running} onClick={() => send('다시 스캔', 'scan')}>
              다시 스캔
            </button>
          )}
          {can('cancel') && (
            <button className="secondary" disabled={busy || running} onClick={() => send('스캔 취소', 'cancel')}>
              취소
            </button>
          )}
        </div>
      )}
    </section>
  );
}
