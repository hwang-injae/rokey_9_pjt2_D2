'use client';
// 점군 3D 창(W112 — 스캔 비교 · E-82 스캔 중 미리보기가 같이 쓴다). PLY 를 받아 점으로만 그린다(계산 · 판정 없음).
// 좌표: 받은 그대로(로봇 PC 가 저장한 틀 — z 가 위)를 three.js(y 가 위)로 (x, z, −y) 만 바꾼다. 단위를 몰라도 되게 카메라를 점 전체에 맞춘다.
import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js';

const VIEW_DIR = new THREE.Vector3(0.8, 0.75, 1).normalize(); // 설계 3D 와 같은 처음 방향(앞 · 오른쪽 · 위에서)
const POINT_PX = 2; // 점 크기(화면 픽셀 — 단위를 몰라도 같은 크기로 보이게)

interface Cloud3D {
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  points: THREE.Points | null;
}

/** 카메라를 점 전체가 보이게 맞춘다(새 스캔일 때만 — 같은 스캔이면 사람이 돌려 둔 방향을 그대로 둔다) */
function frame(s: Cloud3D) {
  const geo = s.points?.geometry;
  if (!geo) return;
  geo.computeBoundingSphere();
  const sphere = geo.boundingSphere;
  if (!sphere || !Number.isFinite(sphere.radius) || sphere.radius <= 0) return;
  const dist = (sphere.radius / Math.sin(THREE.MathUtils.degToRad(s.camera.fov / 2))) * 1.05;
  s.camera.position.copy(sphere.center).addScaledVector(VIEW_DIR, dist);
  s.camera.near = dist / 100;
  s.camera.far = dist * 20;
  s.camera.updateProjectionMatrix();
  s.controls.target.copy(sphere.center);
  s.controls.update();
}

/**
 * 점군 하나를 그린다. url = PLY 주소(바뀌면 다시 받음), frameKey = 바뀔 때만 카메라를 맞춤(같은 스캔의 미리보기가 늘어날 때는 그대로).
 * 바깥 영향: url 을 GET 한다. 실패하면 창 안에 한 줄로 알린다(화면 나머지는 그대로).
 */
export default function CloudView({ url, frameKey, dark }: { url: string; frameKey: string; dark: boolean }) {
  const box = useRef<HTMLDivElement>(null);
  const s3 = useRef<Cloud3D | null>(null);
  const framed = useRef('');
  const [note, setNote] = useState('');

  // 장면 · 카메라 · 마우스 조작은 한 번만 만든다
  useEffect(() => {
    const el = box.current;
    if (!el) return;
    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true });
    } catch {
      setNote('이 브라우저에서는 3D(WebGL)를 쓸 수 없어요');
      return;
    }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    el.appendChild(renderer.domElement);
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(35, 1, 0.001, 5000);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    const s: Cloud3D = { renderer, scene, camera, controls, points: null };
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
      s.points?.geometry.dispose();
      (s.points?.material as THREE.Material | undefined)?.dispose();
      renderer.dispose();
      renderer.domElement.remove(); // 부모가 먼저 지워졌어도 예외 없이(화면 전체가 죽지 않게)
      s3.current = null;
      framed.current = '';
    };
  }, []);

  // 바탕색은 테마를 따른다(설계 3D 와 같은 변수)
  useEffect(() => {
    const s = s3.current;
    if (!s) return;
    const css = getComputedStyle(document.documentElement);
    s.scene.background = new THREE.Color(css.getPropertyValue('--canvas-bg').trim() || '#eef1f5');
  }, [dark]);

  // 점군을 받아 바꿔 그린다(누적 파일이라 더해 그리지 않고 통째로 바꿈 — E-82 10/11 PL)
  useEffect(() => {
    let gone = false;
    if (!s3.current?.points) setNote('점군 받는 중…'); // 이미 그린 점군이 있으면 안내로 가리지 않는다(실시간 미리보기는 1 ~ 2 Hz 로 바뀜)
    fetch(url)
      .then((res) => {
        if (!res.ok) throw new Error(`웹 서버 응답 ${res.status}`);
        return res.arrayBuffer();
      })
      .then((buf) => {
        const s = s3.current;
        if (gone || !s) return;
        const geo = new PLYLoader().parse(buf);
        geo.rotateX(-Math.PI / 2); // (x, y, z) → (x, z, −y): 받은 틀의 z(위)를 three.js 의 위로
        const colored = !!geo.getAttribute('color');
        const css = getComputedStyle(document.documentElement);
        const mat = new THREE.PointsMaterial({
          size: POINT_PX,
          sizeAttenuation: false,
          vertexColors: colored,
          color: colored ? 0xffffff : new THREE.Color(css.getPropertyValue('--primary').trim() || '#2563eb'),
        });
        if (s.points) {
          s.scene.remove(s.points);
          s.points.geometry.dispose();
          (s.points.material as THREE.Material).dispose();
        }
        s.points = new THREE.Points(geo, mat);
        s.scene.add(s.points);
        if (framed.current !== frameKey) {
          frame(s);
          framed.current = frameKey;
        }
        setNote(geo.getAttribute('position')?.count ? '' : '점이 없어요');
      })
      .catch((e: Error) => !gone && setNote(`점군을 못 받았어요 — ${e.message}`));
    return () => {
      gone = true;
    };
  }, [url, frameKey]);

  return (
    <div className="view3d" ref={box}>
      {/* note 가 '' 일 때 `note && …` 은 빈 글자를 그려 React 가 이 칸의 textContent 를 비운다 — 캔버스까지 지워져 점군이 안 보이고
          닫을 때 예외로 화면이 죽었다(10/13). 그래서 삼항으로 */}
      {note ? <div className="view3d-empty">{note}</div> : null}
    </div>
  );
}
