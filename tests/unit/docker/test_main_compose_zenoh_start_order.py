"""Гард #5039: ноды Main Pi стартуют только после healthy zenoh-router.

Контролируемый опыт 23.11.2026 (сырьё — issue #5039): одновременный старт роутера и
нод ломал nav2 в 3 опытах из 4 (паника zenoh `Invalid Key Expr` → Aborted, либо
молчащие сервисы controller_server); четвёртый одновременный старт — при
восстановлении стенда — прошёл. Гонка вероятностная, не гарантированная.
Двухфазный старт «роутер healthy, затем остальные» дал healthy nav2 2/2 за 60 с.
Сам `depends_on: zenoh-router: condition: service_healthy` — 4 успешных старта из
5 (22:18–23:10Z): 12 замеров до t=240s, nav2 healthy, роутер healthy,
12 контейнеров Up, `/controller_server` active, паник zenoh 0 (healthy-статус есть
только у nav2 и роутера — у imu-filter, lslidar, rtabmap,
robot-state-publisher healthcheck нет). Пятый старт (22:55Z) упал при
соблюдённом порядке — паника `Invalid Key Expr` с пустым/битым keyexpr уже
после healthy-роутера, поэтому порядок гонку не устраняет, а «снижает частоту»
статистически не доказано (4/5 против 1/4, точный тест Фишера p ≈ 0,17).
Порядок защищает от вероятностной гонки, а не от детерминированного отказа. Поэтому у
всех зависимых сервисов `depends_on` на zenoh-router — длинная форма с
`condition: service_healthy`, а не короткий список.

Проверяется структура compose, а не текст комментариев: короткая форма `- zenoh-router`
для любого сервиса — падение теста.
"""

from __future__ import annotations

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
COMPOSE = REPO_ROOT / "docker" / "main" / "docker-compose.yaml"

ROUTER = "zenoh-router"


def _services() -> dict:
    with open(COMPOSE, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data["services"]


def _router_dependents(services: dict) -> dict[str, dict]:
    out = {}
    for name, svc in services.items():
        deps = svc.get("depends_on") or {}
        if isinstance(deps, list) and ROUTER in deps:
            out[name] = svc  # короткая форма попадёт сюда и будет наказана ниже
        elif isinstance(deps, dict) and ROUTER in deps:
            out[name] = svc
    return out


def test_zenoh_router_has_healthcheck() -> None:
    router = _services()[ROUTER]
    hc = router.get("healthcheck")
    assert (
        hc
    ), "у zenoh-router должен быть healthcheck — без него service_healthy не работает"
    test = hc.get("test")
    flat = " ".join(test) if isinstance(test, list) else str(test)
    assert (
        "8000" in flat
    ), f"healthcheck должен опрашивать admin API :8000, а не {flat!r}"


def test_no_short_form_depends_on_router() -> None:
    services = _services()
    short = [
        name
        for name, svc in services.items()
        if isinstance(svc.get("depends_on"), list) and ROUTER in svc["depends_on"]
    ]
    assert not short, (
        f"короткая форма depends_on: - {ROUTER} у {short}: ноды поднимутся раньше роутера "
        "(гонка zenoh, #5039). Нужна длинная форма condition: service_healthy"
    )


def test_router_dependents_wait_for_healthy() -> None:
    dependents = _router_dependents(_services())
    assert (
        len(dependents) >= 11
    ), f"зависимых от роутера сервисов {len(dependents)}, ожидалось ≥11"
    bad = {
        name: svc["depends_on"][ROUTER]
        for name, svc in dependents.items()
        if (svc["depends_on"][ROUTER] or {}).get("condition") != "service_healthy"
    }
    assert (
        not bad
    ), f"у {sorted(bad)} условие depends_on на {ROUTER} не service_healthy: {bad}"


def test_other_dependents_keep_short_form_semantics() -> None:
    """Прочие зависимости в длинной форме должны остаться service_started (= старое поведение)."""
    services = _services()
    for name, svc in services.items():
        deps = svc.get("depends_on")
        if not isinstance(deps, dict):
            continue
        for dep, spec in deps.items():
            if dep == ROUTER:
                continue
            assert (spec or {}).get(
                "condition"
            ) == "service_started", (
                f"{name} -> {dep}: ожидалось service_started, чтобы порядок не менялся"
            )
