# Dockerfile for bundled voice resources
# Contains heavy Renardo/FoxDot sample packs used at runtime by voice-assistant and supercollider.

FROM python:3.14-slim-bookworm

ARG BUILD_VERSION=2026-03-08-v1

LABEL org.opencontainers.image.title="Voice Resources for Vision Pi"
LABEL org.opencontainers.image.description="Bundled Renardo sample packs for one-shot volume initialization"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

ENV PYTHONUNBUFFERED=1

RUN pip install --no-cache-dir --break-system-packages renardo-lib

COPY voice_assistant/download_samples.py /tmp/download_samples.py
RUN python3 /tmp/download_samples.py || echo "WARNING: Renardo sample download failed — music will run in synth-only mode" \
    && rm -f /tmp/download_samples.py

RUN if [ -d /root/.config/renardo/samples ]; then \
        cp -rp /root/.config/renardo/samples /renardo_samples_bundle; \
    else \
        mkdir -p /renardo_samples_bundle; \
    fi

COPY voice_resources/init_resources.sh /usr/local/bin/init_voice_resources.sh
RUN chmod +x /usr/local/bin/init_voice_resources.sh

ENTRYPOINT ["/usr/local/bin/init_voice_resources.sh"]
