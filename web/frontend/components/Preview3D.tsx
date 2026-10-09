'use client';
// 설계 3D 보기(E-40 — 미리보기는 three.js 하나). blocks/2.0 · blocks/1 의 x · y · z · ori 를 그대로 상자로 그린다(계산 · 판정 없음).
// 좌표: 설계(mm, x 오른쪽 · y 뒤 · z 위 = 아랫면) → three.js(y 가 위): (x, z, −y). 블록 크기는 backend 가 robot.yaml 에서 준 값(한 곳).
// 같은 부품을 후보 3개(W112) · 스캔 비교(W116)에도 쓴다 — 색 · 투명도는 부르는 쪽이 정한다.
import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export interface Item3D {
  x: number;
  y: number;
  z: number;
  ori: string;
  color: string;
  opacity?: number; // 1 보다 작으면 반투명(아직 안 놓은 블록 · 스캔 추정 블록)
}

type Mm3 = [number, number, number];

/** 방향 코드 → 설계 x · y · z 방향 길이(IRD 2장 ori 표 — d2_task recipe_to_blocks.ori_extents 와 같은 규칙). 모르는 코드는 null */
export function oriExtent(ori: string, [L, W, T]: Mm3): Mm3 | null {
  const table: Record<string, Mm3> = { x: [L, W, T], y: [W, L, T], xe: [L, T, W], ye: [T, L, W], zx: [T, W, L], zy: [W, T, L] };
  return table[ori] ?? null;
}

const GRID_MM = 300; // 바닥 격자 — 조립 작업공간(원점 ± 150 mm)과 같은 크기
const GRID_STEP_MM = 25; // 격자 한 칸 = 블록 폭(설계 격자)
const VIEW_DIR = new THREE.Vector3(0.8, 0.75, 1).normalize(); // 처음 보는 방향: 앞(−y) · 오른쪽 · 위에서

interface Scene3D {
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  blocks: THREE.Group;
  floor: THREE.Group;
}

/** 그룹 안 물체를 빼고 GPU 자원을 돌려준다(다시 그릴 때마다 — 안 하면 메모리가 계속 는다) */
function clear(group: THREE.Group) {
  for (const obj of [...group.children]) {
    group.remove(obj);
    obj.traverse((o) => {
      const m = o as THREE.Mesh;
      m.geometry?.dispose();
      const mat = m.material as THREE.Material | THREE.Material[] | undefined;
      (Array.isArray(mat) ? mat : mat ? [mat] : []).forEach((x) => {
        (x as THREE.SpriteMaterial).map?.dispose();
        x.dispose();
      });
    });
  }
}

/** 바닥 격자 + '앞' 글자(설계 −y 쪽). 색은 테마 변수(--grid-major · --grid-minor · --muted)에서 */
function buildFloor(floor: THREE.Group, css: CSSStyleDeclaration) {
  clear(floor);
  const major = css.getPropertyValue('--grid-major').trim() || '#888';
  const minor = css.getPropertyValue('--grid-minor').trim() || '#ccc';
  floor.add(new THREE.GridHelper(GRID_MM, GRID_MM / GRID_STEP_MM, major, minor));
  const canvas = document.createElement('canvas');
  canvas.width = 128;
  canvas.height = 64;
  const g = canvas.getContext('2d');
  if (g) {
    g.fillStyle = css.getPropertyValue('--muted').trim() || '#888';
    g.font = 'bold 40px system-ui, sans-serif';
    g.textAlign = 'center';
    g.textBaseline = 'middle';
    g.fillText('앞', 64, 32);
  }
  const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthWrite: false }));
  label.scale.set(40, 20, 1);
  label.position.set(0, 2, GRID_MM / 2 + 18);
  floor.add(label);
}

/** 카메라를 블록 전체가 보이게 맞춘다(설계가 바뀔 때만 — 진행도가 바뀔 때는 사람이 돌려 둔 방향을 그대로 둔다) */
function frame(s: Scene3D) {
  const box = new THREE.Box3().setFromObject(s.blocks);
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const radius = Math.max(box.getSize(new THREE.Vector3()).length() / 2, 60);
  const dist = (radius / Math.sin(THREE.MathUtils.degToRad(s.camera.fov / 2))) * 1.05;
  s.camera.position.copy(center).addScaledVector(VIEW_DIR, dist);
  s.camera.near = dist / 100;
  s.camera.far = dist * 20;
  s.camera.updateProjectionMatrix();
  s.controls.target.copy(center);
  s.controls.update();
}

export default function Preview3D({ items, blockMm, frameKey, dark }: { items: Item3D[]; blockMm: Mm3; frameKey: string; dark: boolean }) {
  const box = useRef<HTMLDivElement>(null);
  const s3 = useRef<Scene3D | null>(null);
  const framed = useRef('');
  const [noGl, setNoGl] = useState(false);

  // 장면 · 카메라 · 마우스 조작은 한 번만 만든다
  useEffect(() => {
    const el = box.current;
    if (!el) return;
    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true });
    } catch {
      setNoGl(true); // WebGL 이 없는 브라우저 · 원격 화면 — 3D 대신 안내 글
      return;
    }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    el.appendChild(renderer.domElement);
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(35, 1, 1, 5000);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.maxPolarAngle = Math.PI * 0.495; // 바닥 아래로는 못 내려가게(뒤집혀 헷갈리지 않게)
    scene.add(new THREE.HemisphereLight(0xffffff, 0x777777, 1.8));
    const sun = new THREE.DirectionalLight(0xffffff, 1.5);
    sun.position.set(250, 400, 300);
    scene.add(sun);
    const blocks = new THREE.Group();
    const floor = new THREE.Group();
    scene.add(blocks, floor);
    camera.position.set(240, 220, 300);
    const s: Scene3D = { renderer, scene, camera, controls, blocks, floor };
    s3.current = s;

    const resize = () => {
      const w = el.clientWidth || 1;
      const h = el.clientHeight || 1;
      renderer.setSize(w, h);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };
    const ro = new ResizeObserver(resize);
    ro.observe(el);
    resize();
    let raf = 0;
    const loop = () => {
      controls.update();
      renderer.render(scene, camera);
      raf = requestAnimationFrame(loop);
    };
    loop();
    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      controls.dispose();
      clear(blocks);
      clear(floor);
      renderer.dispose();
      el.removeChild(renderer.domElement);
      s3.current = null;
      framed.current = '';
    };
  }, []);

  // 테마가 바뀌면 바탕 · 격자 색만 다시
  useEffect(() => {
    const s = s3.current;
    if (!s) return;
    const css = getComputedStyle(document.documentElement);
    s.scene.background = new THREE.Color(css.getPropertyValue('--canvas-bg').trim() || '#eef1f5');
    buildFloor(s.floor, css);
  }, [dark, noGl]);

  // 블록을 다시 그린다(진행도 색이 바뀔 때마다 — 블록은 30개 안팎이라 통째로 다시 그려도 가볍다)
  useEffect(() => {
    const s = s3.current;
    if (!s) return;
    clear(s.blocks);
    const edge = new THREE.Color(dark ? '#0b0d10' : '#3a3f45');
    for (const it of items) {
      const ext = oriExtent(it.ori, blockMm);
      if (!ext) continue; // 모르는 방향 코드는 그리지 않는다(지어내지 않음)
      const [ex, ey, ez] = ext;
      const geo = new THREE.BoxGeometry(ex, ez, ey);
      const ghost = (it.opacity ?? 1) < 1;
      const mesh = new THREE.Mesh(
        geo,
        new THREE.MeshStandardMaterial({ color: it.color, roughness: 0.75, transparent: ghost, opacity: it.opacity ?? 1, depthWrite: !ghost }),
      );
      mesh.position.set(it.x, it.z + ez / 2, -it.y);
      const lines = new THREE.LineSegments(
        new THREE.EdgesGeometry(geo),
        new THREE.LineBasicMaterial({ color: edge, transparent: true, opacity: ghost ? 0.35 : 0.8 }),
      );
      mesh.add(lines);
      s.blocks.add(mesh);
    }
    if (items.length && framed.current !== frameKey) {
      frame(s);
      framed.current = frameKey;
    }
  }, [items, blockMm, frameKey, dark]);

  return (
    <div className="view3d" ref={box}>
      {noGl && <div className="view3d-empty">이 브라우저에서는 3D(WebGL)를 쓸 수 없어요</div>}
    </div>
  );
}
