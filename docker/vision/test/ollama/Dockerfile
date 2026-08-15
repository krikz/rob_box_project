FROM ollama/ollama:0.32.13

# Копируем кастомный entrypoint (pull модели + warmup + serve)
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
