'use client';
// 다크 모드 토글 — 코드펜 토글 모양(숨긴 체크박스 + label, 해 · 달). 크기는 globals.css .toggle 의 font-size 하나로 바꾼다.
export default function ThemeToggle({ dark, onChange }: { dark: boolean; onChange: (dark: boolean) => void }) {
  return (
    <div className="theme" title={dark ? '밝은 화면으로' : '어두운 화면으로'}>
      <input type="checkbox" className="sr-only" id="darkmode-toggle" checked={dark} onChange={(e) => onChange(e.target.checked)} />
      <label htmlFor="darkmode-toggle" className="toggle">
        <span>다크 모드</span>
      </label>
    </div>
  );
}
