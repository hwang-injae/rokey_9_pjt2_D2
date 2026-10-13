'use client';
// 스캔 비교 화면(W112 · W116 — IRD 8.4 ⑥ ~ ⑧ · E-80): 실물 사진 · 점군 · 추론 설계를 나란히 보고 사람이 [그대로 저장] · [다시 스캔] · [취소].
// [그대로 저장] = backend 가 AI 칸 채우기(W177) → 검사 → 저장(made_by scan). 저장되면 select_design(⑧)으로 출발 대기(설계 3D 의 [조립]),
// 실패면 backend 가 준 버튼만 남긴다(블록 수 초과 = [취소]만 · 그 밖 = [다시 스캔] · [취소] — 레시피 초안 7-1). [AI로 고치기]는 챌린지(10/14 뒤).
import { useEffect, useMemo, useRef, useState } from 'react';
import * as api from '@/lib/api';
import { designName } from '@/lib/names';
import type { Cmd, Rules, ScanSaveJob } from '@/lib/types';
import type { Robot } from '@/lib/ws';
import CloudView from './CloudView';
import Preview3D, { type Item3D } from './Preview3D';

const POLL_MS = 1000; // /ws 알림이 끊겼을 때를 위한 다시 받기 간격(저장하는 동안만)
const MAX_TRIES = 3; // 처음 + AI 칸 다시 채우기 2번(design_gen SCAN_REFILLS)
const SEEN = '#8f9bab'; // 카메라에 보인 블록
const GUESS = '#e8a33c'; // 가려져서 받침 규칙으로 추정한 블록(반투명)

interface Props {
  robot: Robot;
  rules: Rules | null;
  dark: boolean;
  onSaved: (designId: string) => void; // 저장한 설계를 오른쪽 설계 3D 에 띄운다
  onLog: (kind: string, text: string) => void;
}

/** 스캔 결과 한 번(run_id)의 비교 · 저장. SCAN_REVIEW 상태에서만 그린다(page). 바깥 영향: 저장 요청 · hmi/command(select_design · scan · cancel) */
export default function ScanReview({ robot, rules, dark, onSaved, onLog }: Props) {
  const scan = robot.scan!;
  const runId = scan.run_id;
  const [job, setJob] = useState<ScanSaveJob | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const selected = useRef(''); // select_design 을 보낸 저장 작업(한 번만)
  const items: Item3D[] = useMemo(
    () => scan.blocks.blocks.map((b) => ({ x: b.x, y: b.y, z: b.z, ori: b.ori, color: b.inferred ? GUESS : SEEN, opacity: b.inferred ? 0.45 : 1 })),
    [scan],
  );
  const image = robot.scanImage?.run_id === runId ? robot.scanImage : null; // 다른 스캔의 사진 · 점군은 보이지 않는다(IRD 10.5 짝짓기)
  const cloud = scan.cloud_path && robot.scanCloud?.run_id === runId ? robot.scanCloud : null; // cloud_path 가 없으면 점군 창을 안 띄움(E-67)

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
    setBusy(true);
    setError('');
    const r = await api.scanSave(runId);
    setBusy(false);
    if (!r.data) return setError(r.error ?? '웹 서버 답이 없어요');
    setJob({ job_id: r.data.job_id, state: 'running', run_id: runId });
    onLog('button', '스캔 저장 — AI가 역할 · 순서 · 잡기를 채우고 검사해요');
  };
  const send = async (label: string, cmd: Cmd) => {
    setBusy(true);
    const r = await api.command(cmd);
    setBusy(false);
    onLog('button', r.success ? label : `${label} 실패 — ${r.message ?? r.reason}`);
  };

  let status = '실물 · 점군과 추론 설계를 비교하고 고르세요';
  if (job?.state === 'running')
    status = 'AI가 역할 · 순서 · 잡기를 채우고 검사하는 중…' + ((job.attempt ?? 1) > 1 ? ` (다시 채우기 ${job.attempt}/${MAX_TRIES})` : '');
  else if (job?.state === 'saved') status = `${designName(job.saved?.design_id)}(으)로 저장했어요 — 구조물을 치우고 위 설계 3D 에서 [조립]`;
  else if (job?.state === 'failed') status = job.message || '저장하지 못했어요';
  const failed = job?.state === 'failed';
  const can = (a: 'cancel' | 'rescan') => !failed || (job?.actions ?? ['rescan', 'cancel']).includes(a);

  return (
    <section className="panel scan-panel">
      <div className="panel-head">
        <h2>스캔 결과 비교</h2>
        <span className="muted small">
          블록 {scan.blocks.blocks.length}개 · 가려져 추정 {scan.inferred_count}개
          {scan.nearest_base ? ` · 가장 비슷한 기본 설계 ${designName(scan.nearest_base)}` : ''}
        </span>
      </div>
      <div className="scan-grid">
        <figure>
          <figcaption>실물 사진</figcaption>
          {image ? <img className="scan-photo" src={api.scanImageUrl(image.seq)} alt="스캔한 구조물 사진" /> : <div className="view3d-empty scan-empty">사진을 아직 못 받았어요</div>}
        </figure>
        {scan.cloud_path ? (
          <figure>
            <figcaption>점군</figcaption>
            {cloud ? <CloudView url={api.scanCloudUrl(cloud.seq)} frameKey={runId} dark={dark} /> : <div className="view3d-empty scan-empty">점군을 아직 못 받았어요</div>}
          </figure>
        ) : null}
        <figure>
          <figcaption>
            추론 설계 <span className="muted small">— 반투명 = 가려져서 추정한 블록</span>
          </figcaption>
          {rules ? <Preview3D items={items} blockMm={rules.block_size_mm} frameKey={runId} dark={dark} /> : <div className="view3d-empty scan-empty">블록 크기를 아직 못 받았어요</div>}
        </figure>
      </div>
      <p className={`result ${failed ? 'bad' : job?.state === 'saved' ? 'ok' : ''}`}>{status}</p>
      {error && <p className="bad-text">{error}</p>}
      {job?.state !== 'saved' && (
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
