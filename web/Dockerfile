# 웹 이미지(W102) — 파이썬 의존성만 담는다. 코드 · 설정 · 레시피는 compose 가 읽기 전용으로 연결한다(복사 안 함).
FROM python:3.12-slim
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
COPY backend/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt
WORKDIR /repo/web/backend
