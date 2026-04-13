FROM python:3.14-slim

WORKDIR /app

RUN pip install --no-cache-dir fastapi uvicorn pyyaml

COPY server.py /app/server.py

EXPOSE 8765

CMD ["python3", "/app/server.py"]
