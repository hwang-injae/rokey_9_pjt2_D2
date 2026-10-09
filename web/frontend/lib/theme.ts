'use client';
// 다크 모드 한 곳 — <html data-theme="dark|light"> 를 바꾸고 이 브라우저에만 기억한다(localStorage, 막혀 있으면 기억만 안 함).
// 처음 값은 layout 의 THEME_BOOT(themeBoot.ts)가 그리기 전에 넣는다(기억한 값 → 없으면 운영체제 설정) — 밝은 화면이 번쩍이지 않게.
import { useEffect, useState } from 'react';
import { THEME_KEY } from './themeBoot';

export function useTheme(): [boolean, (dark: boolean) => void] {
  const [dark, setDarkState] = useState(false);

  useEffect(() => {
    setDarkState(document.documentElement.getAttribute('data-theme') === 'dark');
  }, []);

  const setDark = (d: boolean) => {
    document.documentElement.setAttribute('data-theme', d ? 'dark' : 'light');
    try {
      localStorage.setItem(THEME_KEY, d ? 'dark' : 'light');
    } catch {
      // 사생활 보호 창 등에서 막히면 이번 화면에서만 바뀐다
    }
    setDarkState(d);
  };
  return [dark, setDark];
}
