FROM ubuntu:22.04

# Build argument для APT кэша
ARG APT_PROXY=""

# Настройка APT прокси если указан
RUN if [ -n "$APT_PROXY" ]; then \
        echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
        echo "🔧 Using APT proxy: $APT_PROXY"; \
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
