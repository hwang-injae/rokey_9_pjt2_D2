import type { Metadata } from 'next';
import './globals.css';

export const metadata: Metadata = {
  title: 'D2 젠가 가구',
  description: 'AI가 설계하고 로봇이 조립하는 젠가 가구 — 조립 화면',
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="ko">
      <body>{children}</body>
    </html>
  );
}
