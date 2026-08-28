"""Регресс-тесты Caddyfile для rob_box_quest (task t_0b926b23).

Зачем это существует
--------------------
`docker/vision/quest/Caddyfile` — единственная точка, где HTTPS-фронтенд
(порты 443/8443) склеивается с aiohttp-приложением `quest_node`. Приложение
одно (`server/ws_server.py::build_app` регистрирует `/healthz` и `/quest`),
поэтому оба маршрута ОБЯЗАНЫ смотреть в один и тот же upstream-порт.

В host-network рядом живёт `voice-action-server`, который слушает
`127.0.0.1:8765` и тоже отвечает `200 OK` на `/healthz` — но своим телом
(`{"ok": true, "active_goals": 0, "shutting_down": false}`).

Коммит f9bf3f68 («rebind WS to :8766 to avoid EADDRINUSE against
voice-action-server») перевёл `/quest` на 8766, но `/healthz` оставил на
8765. Итог, зафиксированный на живом Vision Pi (10.1.1.21):

    via Caddy  https://localhost:443/healthz  -> {"ok": true, "active_goals": 0, ...}
    quest_node http://localhost:8766/healthz  -> {"status": "ok", "sessions_active": 0, ...}

`local_test/quest_smoke_lib.py::check_healthz` проверяет `body["status"] == "ok"`.
Ответ voice-action-server такого поля не содержит, поэтому секция падала бы —
но 200 от чужого сервиса маскировал факт, что здоровье quest_node вообще НЕ
проверяется. Это ровно тот «красивый PASS», который запрещает ADR-0018.

Тесты ниже — чистый парсинг текста Caddyfile, без сети, Docker и ROS2.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

# local_test/ -> repo root
_REPO_ROOT = Path(__file__).resolve().parent.parent
_CADDYFILE = _REPO_ROOT / "docker" / "vision" / "quest" / "Caddyfile"
_COMPOSE = _REPO_ROOT / "docker" / "vision" / "docker-compose.yaml"

# Порт voice-action-server. Для quest-апстрима он ЗАПРЕЩЁН: сервис чужой,
# отвечает 200 на /healthz и тем самым делает smoke-проверку бессмысленной.
_VOICE_ACTION_SERVER_PORT = "8765"


def _caddyfile_text() -> str:
    assert _CADDYFILE.is_file(), f"Caddyfile не найден: {_CADDYFILE}"
    return _CADDYFILE.read_text(encoding="utf-8")


def _strip_comments(text: str) -> str:
    """Убрать строки-комментарии, чтобы пояснения про 8765 не ломали тесты."""
    return "\n".join(
        line for line in text.splitlines() if not line.lstrip().startswith("#")
    )


def _handle_upstreams(text: str) -> dict[str, list[str]]:
    """Собрать {путь handle: [порты reverse_proxy внутри него]}.

    Парсер намеренно простой: считает глубину фигурных скобок от строки
    `handle <path> {` и собирает все `reverse_proxy host:port` внутри блока.
    """
    body = _strip_comments(text)
    handles: dict[str, list[str]] = {}
    current = ""
    depth = 0

    for line in body.splitlines():
        stripped = line.strip()
        if not current:
            m = re.match(r"^handle\s+(/\S+)\s*\{", stripped)
            if m:
                current = m.group(1)
                depth = stripped.count("{") - stripped.count("}")
                handles.setdefault(current, [])
            continue

        depth += stripped.count("{") - stripped.count("}")
        m = re.search(r"reverse_proxy\s+\S*?:(\d+)", stripped)
        if m:
            handles[current].append(m.group(1))
        if depth <= 0:
            current = ""

    return handles


class TestHealthzUpstream:
    """/healthz обязан идти в quest_node, а не в соседний voice-action-server."""

    def test_healthz_handle_exists(self):
        handles = _handle_upstreams(_caddyfile_text())
        assert "/healthz" in handles, (
            "В Caddyfile нет `handle /healthz` — smoke-скрипт "
            "(quest_smoke_lib.check_healthz) не сможет проверить сервер."
        )

    def test_healthz_does_not_point_at_voice_action_server(self):
        """Главный регресс (t_0b926b23): 8765 — чужой сервис."""
        upstreams = _handle_upstreams(_caddyfile_text())["/healthz"]
        assert _VOICE_ACTION_SERVER_PORT not in upstreams, (
            f"/healthz проксируется на :{_VOICE_ACTION_SERVER_PORT} — это порт "
            "voice-action-server, а не quest_node. Он отвечает 200 своим телом "
            '({"ok": true, "active_goals": ...}), из-за чего smoke-проверка '
            "печатает PASS даже при мёртвом quest_node (ADR-0018: «Честный FAIL "
            "лучше красивого PASS»). Ожидается порт WS_PORT (8766)."
        )

    def test_healthz_and_quest_share_one_upstream(self):
        """build_app() отдаёт /healthz и /quest из одного aiohttp-приложения."""
        handles = _handle_upstreams(_caddyfile_text())
        healthz = handles.get("/healthz", [])
        quest = handles.get("/quest", [])

        assert healthz, "у /healthz нет reverse_proxy upstream"
        assert quest, "у /quest нет reverse_proxy upstream"
        assert set(healthz) == set(quest), (
            f"/healthz -> {healthz}, /quest -> {quest}. Оба маршрута обслуживает "
            "одно aiohttp-приложение quest_node (server/ws_server.py::build_app), "
            "поэтому порты обязаны совпадать. Расхождение означает, что один из "
            "них смотрит в чужой сервис."
        )


class TestComposeWsPortContract:
    """Порт в Caddyfile должен совпадать с WS_PORT из docker-compose."""

    def test_compose_declares_ws_port(self):
        assert _COMPOSE.is_file(), f"compose не найден: {_COMPOSE}"
        assert re.search(r"WS_PORT=\$\{QUEST_WS_PORT:-(\d+)\}", _COMPOSE.read_text(encoding="utf-8")), (
            "В docker/vision/docker-compose.yaml нет `WS_PORT=${QUEST_WS_PORT:-...}` "
            "для сервиса quest. Без него quest_node возьмёт default 8765 из "
            "quest_node.py и упадёт с EADDRINUSE против voice-action-server."
        )

    def test_caddyfile_upstream_matches_compose_ws_port(self):
        m = re.search(
            r"WS_PORT=\$\{QUEST_WS_PORT:-(\d+)\}", _COMPOSE.read_text(encoding="utf-8")
        )
        assert m, "WS_PORT не найден в compose (см. test_compose_declares_ws_port)"
        ws_port = m.group(1)

        handles = _handle_upstreams(_caddyfile_text())
        for path in ("/healthz", "/quest"):
            assert handles.get(path) == [ws_port], (
                f"handle {path} -> {handles.get(path)}, а compose задаёт "
                f"WS_PORT={ws_port}. Caddy запечён в образ (COPY в Dockerfile) и "
                "не читает env, поэтому расхождение чинится только правкой "
                "Caddyfile + пересборкой образа."
            )


class TestQuestWsTransport:
    """Защита ранее исправленных багов, чтобы их не откатили назад."""

    def test_quest_forces_http11_not_h2c(self):
        """450aa895: h2c ломает WebSocket-апгрейд (502 на каждый handshake)."""
        text = _strip_comments(_caddyfile_text())
        m = re.search(r"handle\s+/quest\s*\{(.+?)\n    \}", text, re.DOTALL)
        assert m, "не найден блок `handle /quest`"
        block = m.group(1)

        assert "flush_interval -1" in block, (
            "у /quest потерян `flush_interval -1` — Caddy начнёт буферизовать "
            "JPEG/LiDAR-фреймы и на Quest появится лаг."
        )
        versions = re.search(r"versions\s+([^\n}]+)", block)
        assert versions, "у /quest нет `transport http { versions ... }`"
        assert "h2c" not in versions.group(1), (
            "у /quest вернулся h2c: Caddy пытается WebSocket-апгрейд по HTTP/2 и "
            'падает с "http2: invalid Upgrade request header: [websocket]" → 502 '
            "на каждый handshake (регресс 450aa895). Оставляйте только 1.1."
        )


class TestListenPorts:
    """Quest-браузер идёт на 443; 8443 нужен smoke-скрипту."""

    @pytest.mark.parametrize("port", ["443", "8443"])
    def test_site_listens_on_port(self, port):
        text = _strip_comments(_caddyfile_text())
        m = re.search(r"^((?::\d+\s*,\s*)*:\d+)\s*\{", text, re.MULTILINE)
        assert m, "не найден site-блок вида `:443, :8443 {`"
        listen = [p.strip().lstrip(":") for p in m.group(1).split(",")]
        assert port in listen, (
            f"порт {port} потерян в site-блоке (сейчас: {listen}). "
            "443 — дефолт браузера Quest, 8443 — контракт quest_smoke.sh "
            "(QUEST_PORT по умолчанию)."
        )
