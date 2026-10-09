/**
 * 정적 내보내기(out/) — 실행 때 Node 서버 없이 backend(FastAPI :8000)가 같은 주소에서 내려 준다(E-41, SDD 3.1.1).
 * trailingSlash: 페이지마다 폴더/index.html 로 만들어 backend StaticFiles(html=True)가 그대로 찾게 한다.
 */
const nextConfig = {
  output: 'export',
  trailingSlash: true,
  images: { unoptimized: true },
};

export default nextConfig;
