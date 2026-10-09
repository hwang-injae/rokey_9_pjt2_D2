// 다크 모드 처음 값 — layout(서버에서 만드는 HTML)의 <head> 에 넣는 작은 스크립트. React 가 뜨기 전에 data-theme 를 정한다
// (기억한 값 → 없으면 운영체제 설정). hook 이 있는 theme.ts 와 나눈 까닭: 서버 쪽 layout 은 React hook 파일을 import 할 수 없다.
export const THEME_KEY = 'd2-theme';

export const THEME_BOOT = `(function(){var t=null;try{t=localStorage.getItem('${THEME_KEY}')}catch(e){}
if(t!=='dark'&&t!=='light'){t=window.matchMedia&&matchMedia('(prefers-color-scheme: dark)').matches?'dark':'light'}
document.documentElement.setAttribute('data-theme',t)})()`;
