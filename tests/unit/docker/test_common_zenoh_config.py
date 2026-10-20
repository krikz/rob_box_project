"""Guards for docker/common — общие Zenoh-шаблоны и скрипты всех Pi (ADR-0159 В4, PR-2a).

До PR-2a router/session-конфиги, ``ros_with_namespace.sh`` и старт роутера
лежали двумя копиями (docker/main и docker/vision) и отличались только
адресами. Теперь шаблон один, адреса — ``ZENOH_LISTEN_IP`` /
``ZENOH_CONNECT_ENDPOINTS`` с дефолтами в compose узла.

Тесты исполняют НАСТОЯЩИЕ скрипты (``sh``/``bash``) над НАСТОЯЩИМИ
шаблонами с дефолтами, вычитанными из compose, и сверяют endpoint'ы с
адресами, на которых робот работал до PR-2a (develop @ d79692c15):

* Main Pi:   listen ``tcp/10.1.1.10:7447``, connect ``tcp/zenoh.robbox.online:7447``
* Vision Pi: listen ``tcp/10.1.1.11:7447``, connect ``tcp/10.1.1.10:7447#iface=eth0``
* сессии нод: ``tcp/<IP узла>:7447#so_sndbuf=16777216;so_rcvbuf=16777216``

Плюс гард issue #3669: у роутеров нет ``storage_manager`` на ключ ``robots/**`` —
иначе каждый роутер подписан на весь трафик робота, и соседи шлют ему сырые
кадры камер (~24 МБ/с на роутер) ради in-memory копии, у которой не нашлось потребителей.

Docker не нужен.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
COMMON = REPO_ROOT / "docker" / "common"
ROUTER_TEMPLATE = COMMON / "config" / "zenoh_router_config.json5"
SESSION_TEMPLATE = COMMON / "config" / "zenoh_session_config.json5"
ROUTER_SCRIPT = COMMON / "scripts" / "zenoh-router" / "start_zenoh_router.sh"
NS_SCRIPT = COMMON / "scripts" / "ros_with_namespace.sh"

EXPECTED = {
    "main": {
        "ip": "10.1.1.10",
        "connect": ["tcp/zenoh.robbox.online:7447"],
    },
    "vision": {
        "ip": "10.1.1.11",
        "connect": ["tcp/10.1.1.10:7447#iface=eth0"],
    },
    # ADR-0159 §2.2 вариант Б (В3): прямой линк на Vision + линк на Main.
    "sound": {
        "ip": "10.1.1.12",
        "connect": ["tcp/10.1.1.10:7447#iface=eth0", "tcp/10.1.1.11:7447#iface=eth0"],
    },
}

SH = shutil.which("sh")
BASH = shutil.which("bash")
needs_sh = pytest.mark.skipif(SH is None, reason="нет sh в PATH")
needs_bash = pytest.mark.skipif(BASH is None, reason="нет bash в PATH")


def _compose(pi: str) -> dict:
    with (REPO_ROOT / "docker" / pi / "docker-compose.yaml").open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _interpolate_defaults(value: str) -> str:
    """``${VAR:-default}`` → ``default`` — как compose при пустом окружении."""
    return re.sub(r"\$\{[A-Z_]+:-([^}]*)\}", r"\1", value)


def _env(service: dict) -> dict[str, str]:
    out = {}
    for item in service.get("environment") or []:
        key, _, val = item.partition("=")
        out[key] = _interpolate_defaults(val)
    return out


def _strip_comments(text: str) -> str:
    return "\n".join(re.sub(r"\s*//.*$", "", line) for line in text.splitlines())


def _endpoints(text: str, section: str) -> list[str]:
    """Строки массива ``<section>: { ... endpoints: [ ... ] }`` без комментариев."""
    body = _strip_comments(text)
    m = re.search(rf"\n  {section}: \{{.*?\n    endpoints: \[(.*?)\]", body, re.S)
    assert m, f"не нашёл {section}.endpoints"
    return re.findall(r'"([^"]*)"', m.group(1))


def _render_router(tmp_path: Path, env: dict[str, str]) -> subprocess.CompletedProcess:
    out = tmp_path / "router.json5"
    full_env = {
        "PATH": os.environ.get("PATH", ""),
        "ZENOH_ROUTER_TEMPLATE": ROUTER_TEMPLATE.as_posix(),
        "ZENOH_ROUTER_CONFIG_OUT": out.as_posix(),
        "ZENOH_RENDER_ONLY": "1",
        **env,
    }
    return subprocess.run(
        [SH, ROUTER_SCRIPT.as_posix()], env=full_env, capture_output=True, text=True,
        encoding="utf-8", check=False,
    )


@needs_sh
@pytest.mark.parametrize("pi", sorted(EXPECTED))
def test_router_renders_todays_addresses_from_compose_defaults(pi: str, tmp_path: Path) -> None:
    router_env = _env(_compose(pi)["services"]["zenoh-router"])
    env = {k: router_env[k] for k in ("ZENOH_LISTEN_IP", "ZENOH_CONNECT_ENDPOINTS")}
    res = _render_router(tmp_path, env)
    assert res.returncode == 0, res.stdout + res.stderr
    text = (tmp_path / "router.json5").read_text(encoding="utf-8")
    assert "${" not in text
    assert _endpoints(text, "listen") == [f"tcp/{EXPECTED[pi]['ip']}:7447"]
    assert _endpoints(text, "connect") == EXPECTED[pi]["connect"]


@needs_sh
def test_router_connect_list_with_two_endpoints(tmp_path: Path) -> None:
    """Sound Pi по ADR-0159 §3.2 (вариант Б): два connect-endpoint'а."""
    res = _render_router(tmp_path, {
        "ZENOH_LISTEN_IP": "10.1.1.12",
        "ZENOH_CONNECT_ENDPOINTS": "tcp/10.1.1.10:7447#iface=eth0,tcp/10.1.1.11:7447#iface=eth0",
    })
    assert res.returncode == 0, res.stdout + res.stderr
    text = (tmp_path / "router.json5").read_text(encoding="utf-8")
    assert _endpoints(text, "listen") == ["tcp/10.1.1.12:7447"]
    assert _endpoints(text, "connect") == [
        "tcp/10.1.1.10:7447#iface=eth0", "tcp/10.1.1.11:7447#iface=eth0",
    ]


@needs_sh
@pytest.mark.parametrize("env", [
    {"ZENOH_CONNECT_ENDPOINTS": "tcp/10.1.1.10:7447"},
    {"ZENOH_LISTEN_IP": "vision", "ZENOH_CONNECT_ENDPOINTS": "tcp/10.1.1.10:7447"},
    {"ZENOH_LISTEN_IP": "10.1.1.11"},
    {"ZENOH_LISTEN_IP": "10.1.1.11", "ZENOH_CONNECT_ENDPOINTS": "tcp/a:1,"},
    {"ZENOH_LISTEN_IP": "10.1.1.11", "ZENOH_CONNECT_ENDPOINTS": 'tcp/a:1"'},
    {"ROBOT_ID": "", "ZENOH_LISTEN_IP": "10.1.1.10", "ZENOH_CONNECT_ENDPOINTS": "tcp/a:1"},
], ids=["no-listen-ip", "listen-not-ipv4", "no-connect", "empty-item", "quote", "empty-robot-id"])
def test_router_fails_loudly_on_bad_env(env: dict[str, str], tmp_path: Path) -> None:
    res = _render_router(tmp_path, env)
    assert res.returncode != 0
    assert "ОШИБКА" in res.stderr


def _ros_services(pi: str) -> dict[str, dict]:
    return {
        name: svc for name, svc in _compose(pi)["services"].items()
        if (svc.get("command") or [None])[0] == "/ros_scripts/ros_with_namespace.sh"
    }


@pytest.mark.parametrize("pi", sorted(EXPECTED))
def test_every_wrapped_service_gets_common_mounts_and_listen_ip(pi: str) -> None:
    services = _ros_services(pi)
    assert services, f"{pi}: не нашёл ни одного сервиса с ros_with_namespace.sh"
    for name, svc in services.items():
        vols = svc.get("volumes") or []
        assert "../common/scripts:/ros_scripts:ro" in vols, f"{pi}/{name}: {vols}"
        assert "../common/config:/common_config:ro" in vols, f"{pi}/{name}: {vols}"
        assert _env(svc).get("ZENOH_LISTEN_IP") == EXPECTED[pi]["ip"], f"{pi}/{name}"


@needs_bash
@pytest.mark.parametrize("pi", sorted(EXPECTED))
def test_session_config_points_at_local_router(pi: str, tmp_path: Path) -> None:
    out = tmp_path / "session.json5"
    env = {
        "PATH": os.environ.get("PATH", ""),
        "ROBOT_ID": "RBXU100001",
        "ZENOH_LISTEN_IP": EXPECTED[pi]["ip"],
        "ZENOH_SESSION_TEMPLATE": SESSION_TEMPLATE.as_posix(),
        "ZENOH_SESSION_CONFIG_OUT": out.as_posix(),
    }
    res = subprocess.run([BASH, NS_SCRIPT.as_posix(), "true"], env=env, capture_output=True,
                         text=True, encoding="utf-8", check=False)
    assert res.returncode == 0, res.stdout + res.stderr
    text = out.read_text(encoding="utf-8")
    assert "${" not in text
    assert _endpoints(text, "connect") == [
        f"tcp/{EXPECTED[pi]['ip']}:7447#so_sndbuf=16777216;so_rcvbuf=16777216",
    ]
    assert re.search(r'^\s*namespace: "robots/RBXU100001",', text, re.M)


@needs_bash
def test_session_wrapper_refuses_to_start_without_listen_ip(tmp_path: Path) -> None:
    env = {
        "PATH": os.environ.get("PATH", ""),
        "ROBOT_ID": "RBXU100001",
        "ZENOH_SESSION_TEMPLATE": SESSION_TEMPLATE.as_posix(),
        "ZENOH_SESSION_CONFIG_OUT": (tmp_path / "s.json5").as_posix(),
    }
    res = subprocess.run([BASH, NS_SCRIPT.as_posix(), "echo", "NODE-STARTED"], env=env,
                         capture_output=True, text=True, encoding="utf-8", check=False)
    assert res.returncode != 0
    assert "NODE-STARTED" not in res.stdout


def _plugins_block(text: str) -> str:
    """Тело ``plugins: { ... }`` верхнего уровня (без комментариев), по счёту скобок."""
    body = _strip_comments(text)
    m = re.search(r"\n  plugins: \{", body)
    assert m, "не нашёл верхнеуровневый plugins"
    depth, start = 1, m.end()
    for i in range(start, len(body)):
        if body[i] == "{":
            depth += 1
        elif body[i] == "}":
            depth -= 1
            if depth == 0:
                return body[start:i]
    raise AssertionError("plugins: не закрыт")


def test_router_template_has_no_storage_manager() -> None:
    """issue #3669: хранилище ``robots/**`` на роутере = подписка роутера на весь трафик.

    Все топики ROS лежат под ``robots/<ROBOT_ID>/…``, поэтому storage с таким ключом
    заставляет каждого соседа слать роутеру ВСЁ (в т.ч. кадры oak-d ~24 МБ/с) ради
    копии последнего значения в памяти. transient_local хранилища роутера не требует
    (история берётся из кэша издателя по ``<топик>/@adv/**``, а ``**`` не матчит ``@adv``).
    """
    plugins = _plugins_block(ROUTER_TEMPLATE.read_text(encoding="utf-8"))
    assert "storage_manager" not in plugins, "storage_manager вернулся в шаблон роутера (#3669)"
    assert "storages" not in plugins
    assert "key_expr" not in plugins


def test_router_template_keeps_rest_admin_on_8000() -> None:
    """REST-админка нужна: healthcheck всех Pi и ожидание роутера в start_*.sh ходят на :8000."""
    plugins = _plugins_block(ROUTER_TEMPLATE.read_text(encoding="utf-8"))
    assert re.search(r"rest:\s*\{\s*http_port:\s*8000,?\s*\}", plugins), plugins


@pytest.mark.parametrize("pi", sorted(EXPECTED))
def test_router_healthcheck_uses_rest_admin(pi: str) -> None:
    """Связка: healthcheck compose ↔ плагин rest в шаблоне (его нельзя убрать вместе со storage)."""
    check = _compose(pi)["services"]["zenoh-router"]["healthcheck"]["test"]
    assert "http://localhost:8000/@/local/router" in " ".join(check)


def test_no_router_config_under_docker_declares_storage_manager() -> None:
    """Копированием чужого шаблона хранилище не должно вернуться ни в один конфиг docker/."""
    offenders = [
        path.relative_to(REPO_ROOT).as_posix()
        for path in sorted((REPO_ROOT / "docker").rglob("*.json5"))
        if "storage_manager" in _strip_comments(path.read_text(encoding="utf-8"))
    ]
    assert offenders == [], f"storage_manager в конфигах роутера (#3669): {offenders}"


@pytest.mark.parametrize("rel", [
    "main/scripts/ros_with_namespace.sh",
    "vision/scripts/ros_with_namespace.sh",
    "main/scripts/zenoh-router/start_zenoh_router.sh",
    "vision/scripts/zenoh-router/start_zenoh_router.sh",
    "main/config/zenoh_router_config.json5",
    "vision/config/zenoh_router_config.json5",
    "main/config/zenoh_session_config.json5",
    "vision/config/zenoh_session_config.json5",
    "main/zenoh-router/Dockerfile",
    "vision/zenoh-router/Dockerfile",
    "sound/scripts/ros_with_namespace.sh",
    "sound/scripts/zenoh-router/start_zenoh_router.sh",
    "sound/config/zenoh_router_config.json5",
    "sound/config/zenoh_session_config.json5",
    "sound/zenoh-router/Dockerfile",
])
def test_no_per_node_copies_left(rel: str) -> None:
    """Одна реализация в docker/common, не копия на узел (ADR-0159 В4)."""
    assert not (REPO_ROOT / "docker" / rel).exists()
