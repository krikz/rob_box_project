FROM ubuntu:22.04

# Build argument для APT кэша
ARG APT_PROXY=""

# Настройка APT прокси если указан и доступен
# Проверяем доступность прокси перед настройкой
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*:\([0-9]*\)$|\1|'); \
        echo "🔍 Checking proxy availability at $PROXY_HOST:$PROXY_PORT..."; \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::http::Timeout \"10\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::Retries \"3\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "✅ Using APT proxy: $APT_PROXY"; \
        else \
            echo "⚠️  APT proxy not accessible, will use direct connection"; \
        fi \
    fi

# Только основные пакеты которые точно есть
RUN apt-get update && \
    apt-get install -y \
        git \
        curl \
        && rm -rf /var/lib/apt/lists/*

# Удаляем APT прокси конфигурацию
RUN rm -f /etc/apt/apt.conf.d/02proxy

CMD ["echo", "Test successful!"]
