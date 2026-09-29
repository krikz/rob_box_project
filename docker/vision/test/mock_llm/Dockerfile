FROM python:3.11-slim

WORKDIR /app

RUN pip install --no-cache-dir fastapi uvicorn pyyaml

COPY server.py /app/server.py

RUN useradd --no-create-home --uid 1000 mockllm && \
    chown -R mockllm:mockllm /app
USER 1000:1000

EXPOSE 8765

HEALTHCHECK --interval=5s --timeout=3s --start-period=5s --retries=10 \
    CMD python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8765/health', timeout=2)" || exit 1

CMD ["python3", "/app/server.py"]
