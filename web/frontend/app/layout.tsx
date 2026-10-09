import type { Metadata } from 'next';
import { THEME_BOOT } from '@/lib/themeBoot';
import './globals.css';

export const metadata: Metadata = {
  title: 'D2 젠가 가구',
  description: 'AI가 설계하고 로봇이 조립하는 젠가 가구 — 조립 화면',
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  // data-theme 는 그리기 전에 THEME_BOOT 가 넣는다 — 서버에서 만든 HTML 과 달라도 경고하지 않게 suppressHydrationWarning
  return (
    <html lang="ko" suppressHydrationWarning>
      <head>
        <script dangerouslySetInnerHTML={{ __html: THEME_BOOT }} />
      </head>
      <body>{children}</body>
    </html>
  );
}
