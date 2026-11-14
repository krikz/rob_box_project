#!/usr/bin/env python3
"""Смоук студии: пайплайн не встаёт, даже если товарищ Шифу проигнорировал черновик.

Задача #4700 (ADR-0170). Скрипт гоняет РЕАЛЬНЫЙ код студии (`studio_state`,
`channel_studio`, `studio_facts`) на временном состоянии — без Telegram, без
сети, без MiniMax. Ничего не отправляет и ничего не меняет в боевой студии.

Что проверяет (по acceptance из #4700):

1. Материал готовится каждый день: сбор фактов не стоит за 48-часовым фильтром.
2. Молчание — не отказ: черновик протухает, его факт возвращается в ожидание.
3. Явный отказ («Мимо») закрывает факт — обратно он не предлагается.
4. Материал не утекает: собранные, но не показанные факты остаются в ожидании.
5. Факт, потерянный вместе с черновиком, находится и восстанавливается.
6. Кнопка на протухшем черновике даёт понятный ответ и не уводит пост в очередь.
7. Публикация в канал — не чаще раза в 48 ч (`MIN_GAP` не тронут).

Запуск:

    python3 scripts/telegram/studio_smoke.py

Код возврата: `0` — все проверки прошли, `1` — есть провал (см. вывод).
"""

from __future__ import annotations

import sys
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_studio as studio  # noqa: E402
import studio_facts as facts_mod  # noqa: E402
import studio_state as st  # noqa: E402

FAILURES: list = []
CHECKS = 0


def check(name: str, condition: bool, detail: str = "") -> None:
    global CHECKS
    CHECKS += 1
    if condition:
        print(f"  ok   {name}")
    else:
        print(f"  FAIL {name}" + (f" — {detail}" if detail else ""))
        FAILURES.append(name)


def iso(moment: datetime) -> str:
    return moment.strftime("%Y-%m-%dT%H:%M:%SZ")


def fact(key: str) -> dict:
    number = key.split(":")[1]
    return {
        "key": key,
        "kind": "merged_pr",
        "claim": f"PR #{number} влит в develop: правка диалога",
        "source": f"PR #{number} (https://github.com/krikz/rob_box_project/pull/{number})",
        "visible": True,
    }


def section(title: str) -> None:
    print(f"\n{title}")


def main() -> int:
    tmp = Path(tempfile.mkdtemp(prefix="studio_smoke_"))
    state_path = tmp / "state.json"

    # --- 1. Сбор фактов не стоит за 48-часовым фильтром -----------------------
    section("1. Подготовка — каждый день (фильтр только на доставке)")
    script = (REPO / "scripts" / "telegram" / "studio_session.sh").read_text(encoding="utf-8")
    facts_at = script.index("собираю факты")
    delivery_at = script.index("Частота доставки")
    check("сбор фактов идёт ДО фильтра доставки", facts_at < delivery_at)
    check("фильтр 48 ч стоит на доставке, а не на подготовке", "TOO_SOON" in script)

    # --- 2. Собранный материал не утекает ------------------------------------
    section("2. Материал не утекает")
    facts = [fact("pr:4555"), fact("pr:4600")]
    with st.state_transaction(state_path) as state:
        st.keep_collected(state, facts)
    state = st.load_state(state_path)
    pending = {f["key"]: f for f in state.get("pending", [])}
    check("оба собранных факта в ожидании", set(pending) == {"pr:4555", "pr:4600"}, str(sorted(pending)))
    check("показов ещё не было", all(f["attempts"] == 0 for f in pending.values()))

    # --- 3. Показ черновика считает одну попытку ----------------------------
    section("3. Показ черновика")
    post = {
        "id": "p-smoke",
        "title": "черновик",
        "body_html": "<b>текст</b>",
        "tags": ["#роббокс"],
        "fact_keys": ["pr:4555"],
        "fact_dicts": [fact("pr:4555")],
    }
    studio.register_post(state_path, post)
    state = st.load_state(state_path)
    pending = {f["key"]: f for f in state.get("pending", [])}
    check("показанный факт учтён как одна попытка", pending["pr:4555"]["attempts"] == 1)
    check("непоказанный факт не тронут", pending["pr:4600"]["attempts"] == 0)

    # --- 4. Молчание — не отказ ---------------------------------------------
    section("4. Молчание — не отказ (TTL черновика)")
    state["posts"]["p-smoke"]["created_at"] = iso(datetime.now(timezone.utc) - timedelta(hours=49))
    st.save_state(state_path, state)
    state = st.load_state(state_path)
    expired = st.expire_stale_drafts(state)
    st.save_state(state_path, state)
    state = st.load_state(state_path)
    check("черновик протух", expired == ["p-smoke"], str(expired))
    check("статус expired", state["posts"]["p-smoke"]["status"] == "expired")
    pending = {f["key"]: f for f in state.get("pending", [])}
    check("факт вернулся в ожидание", "pr:4555" in pending)
    check("факт вернулся ЦЕЛИКОМ (claim на месте)", bool(pending.get("pr:4555", {}).get("claim")))
    check("второй факт не потерян", "pr:4600" in pending)

    # --- 5. Кнопка на протухшем черновике -----------------------------------
    section("5. Кнопка на протухшем черновике")
    answer = studio.apply_post_action(state, "p-smoke", studio.ACTION_PUBLISH)
    check("ответ понятный", "протух" in answer.lower(), answer[:80])
    check("пост не ушёл в очередь", "queue_file" not in state["posts"]["p-smoke"])

    # --- 6. Потерянный факт находится ---------------------------------------
    section("6. Потерянный факт восстанавливается")
    lost_state = {
        "posts": {"p-lost": {"id": "p-lost", "status": "proposed", "fact_keys": ["pr:4700"]}},
        "pending": [],
        "covered": {},
        "rejected": {},
    }
    check("факт, потерянный с черновиком, найден", facts_mod.lost_fact_keys(lost_state) == {"pr:4700"})
    check(
        "опубликованный пост не считается потерянным",
        facts_mod.lost_fact_keys(
            {"posts": {"p": {"status": "approved", "fact_keys": ["pr:1"]}}, "pending": [], "covered": {}, "rejected": {}}
        )
        == set(),
    )

    # --- 7. Явный отказ закрывает факт --------------------------------------
    section("7. Явный отказ («Мимо») закрывает факт")
    state = st.load_state(state_path)
    st.mark_rejected(state, {"pr:4600"}, iso(datetime.now(timezone.utc)))
    st.save_state(state_path, state)
    state = st.load_state(state_path)
    covered = st.covered_keys(state)
    reoffered = [f["key"] for f in st.pending_facts(state) if f["key"] not in covered]
    check("отклонённый факт закрыт", "pr:4600" in covered)
    check("отклонённый факт не предлагается снова", "pr:4600" not in reoffered)

    # --- 8. Лимит публикации в канал не тронут ------------------------------
    section("8. Публикация в канал — не чаще раза в 48 ч")
    import channel_lib as lib

    check("MIN_GAP = 48 ч", lib.MIN_GAP == timedelta(hours=48), str(lib.MIN_GAP))

    # --- 9. Разбор меток времени на любой версии Python ---------------------
    section("9. Метки времени с суффиксом Z (Python 3.10)")
    moment = st.parse_iso("2026-11-10T21:15:00Z")
    check("метка Z разбирается", moment is not None)
    check("разбор aware (есть tzinfo)", moment is not None and moment.tzinfo is not None)
    check("мусор не превращается в дату", st.parse_iso("не дата") is None)

    # --- Итог ----------------------------------------------------------------
    print(f"\n{'=' * 60}")
    if FAILURES:
        print(f"FAIL: {len(FAILURES)} из {CHECKS} проверок не прошли:")
        for name in FAILURES:
            print(f"  - {name}")
        return 1
    print(f"OK: все {CHECKS} проверок прошли")
    return 0


if __name__ == "__main__":
    sys.exit(main())
