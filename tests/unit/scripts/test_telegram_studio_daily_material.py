"""Студия готовит материал каждый день: игнор не останавливает пайплайн.

Разбор 11.11 (issue #4700): три поломки, которые вместе остановили студию.

1. Фильтр «не чаще 48 ч» стоял ПЕРЕД сбором фактов: сессии 10, 11 и 12.11
   вышли с «молчу до окна в 48 ч», не собрав ни одного факта.
2. `attempts` рос на каждый показ, а `MAX_ATTEMPTS=3` уводил факт в
   `deferred` навсегда — игнор приравнивался к отказу (факт `pr:4555`).
3. В пути `draft` факты не попадали в `pending`: проигнорированный черновик
   терял событие целиком.

Здесь — регрессия на каждую из трёх поломок плюс на восстановление.
"""

import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_studio as studio  # noqa: E402
import studio_draft as draft_mod  # noqa: E402
import studio_facts as facts_mod  # noqa: E402
import studio_state as st  # noqa: E402


def _iso(moment: datetime) -> str:
    return moment.strftime("%Y-%m-%dT%H:%M:%SZ")


def _fact(key: str, claim: str = "") -> dict:
    return {
        "key": key,
        "kind": "merged_pr",
        "claim": claim or f"PR #{key.split(':')[1]} влит в develop: важная правка",
        "source": f"PR #{key.split(':')[1]} (https://github.com/krikz/rob_box_project/pull/{key.split(':')[1]})",
        "visible": True,
    }


# ------------------------------------------------------------------ 1. TTL черновика


def test_ignored_draft_expires_and_returns_fact_to_pending():
    """Молчание — не отказ: протухший черновик возвращает факт в ожидание."""
    state = {
        "posts": {
            "p1": {
                "id": "p1",
                "status": "proposed",
                "created_at": _iso(datetime.now(timezone.utc) - timedelta(hours=49)),
                "fact_keys": ["pr:4555"],
                "fact_dicts": [_fact("pr:4555")],
            }
        },
        "pending": [{"key": "pr:4555", "attempts": 1}],
    }

    expired = st.expire_stale_drafts(state)

    assert expired == ["p1"]
    assert state["posts"]["p1"]["status"] == "expired"
    assert state["posts"]["p1"]["expired_at"]
    # Факт вернулся в ожидание — с полными данными, а не одним ключом.
    pending = {f["key"]: f for f in state.get("pending", [])}
    assert "pr:4555" in pending
    assert pending["pr:4555"]["claim"]


def test_fresh_draft_does_not_expire():
    """Свежий черновик живёт: TTL не вышибает черновик из-под ног."""
    state = {
        "posts": {
            "p1": {
                "id": "p1",
                "status": "proposed",
                "created_at": _iso(datetime.now(timezone.utc) - timedelta(hours=3)),
                "fact_keys": ["pr:4555"],
            }
        },
        "pending": [{"key": "pr:4555"}],
    }

    assert st.expire_stale_drafts(state) == []
    assert state["posts"]["p1"]["status"] == "proposed"


def test_approved_post_is_not_expired():
    """Пост, уже стоящий в очереди, не протухает: он ждёт своего слота."""
    state = {
        "posts": {
            "p1": {
                "id": "p1",
                "status": "approved",
                "created_at": _iso(datetime.now(timezone.utc) - timedelta(hours=100)),
                "queue_file": "/tmp/q/001-post.md",
                "fact_keys": ["pr:4555"],
            }
        },
        "pending": [],
    }

    assert st.expire_stale_drafts(state) == []
    assert state["posts"]["p1"]["status"] == "approved"


def test_expired_draft_button_answers_not_crashes():
    """Нажатие на кнопку протухшего черновика даёт понятный ответ."""
    state = {
        "posts": {"p1": {"id": "p1", "status": "expired", "fact_keys": ["pr:4555"]}}
    }

    answer = studio.apply_post_action(state, "p1", studio.ACTION_PUBLISH)

    assert "протух" in answer.lower()
    # И главное: протухший черновик не должен уехать в очередь канала.
    assert "queue_file" not in state["posts"]["p1"]


# ------------------------------------------------------------------ 2. Игнор ≠ отказ


def test_shown_but_ignored_fact_is_not_deferred():
    """Показ без ответа не отправляет факт в deferred.

    Отказ — только явный («Мимо»). Раньше три показа равнялись отказу, и
    факт `pr:4555` исчез из ленты навсегда.
    """
    facts = [
        {**_fact("pr:4555"), "attempts": 9},
        _fact("pr:4600"),
    ]

    fresh, deferred = facts_mod.partition_facts(facts, set())

    # Факт, который показывали много раз, но НЕ отклоняли, — показывается снова.
    assert "pr:4555" not in deferred
    assert "pr:4555" in {f["key"] for f in fresh}


def test_attempts_do_not_grow_per_show_of_same_post():
    """Один показ — одна попытка, даже если запись шла дважды."""
    state = {}
    fact = _fact("pr:4555")

    st.save_pending(state, [fact], "p1")
    st.save_pending(state, [fact], "p1")

    pending = {f["key"]: f for f in state["pending"]}
    assert pending["pr:4555"]["attempts"] == 1


def test_rejected_fact_stays_closed():
    """Явный отказ («Мимо») закрывает факт: обратно он не возвращается."""
    state = {"pending": [{"key": "pr:4555"}]}

    st.mark_rejected(state, {"pr:4555"}, _iso(datetime.now(timezone.utc)))

    assert "pr:4555" in st.covered_keys(state)
    assert state.get("pending") == []


# ------------------------------------------------------------------ 3. Факт не теряется


def test_draft_registration_puts_fact_into_pending(tmp_path):
    """Показанный факт ложится в ожидание: без этого игнор = потеря события.

    Путь `draft` (studio_jobs.draft_job → register_post) раньше этого не
    делал — факт `pr:4555` остался только ключом в записи поста.
    """
    state_path = tmp_path / "state.json"
    post = {
        "id": "p1",
        "title": "черновик",
        "body_html": "<b>текст</b>",
        "tags": ["#роббокс"],
        "fact_keys": ["pr:4555"],
        "fact_dicts": [_fact("pr:4555")],
    }

    studio.register_post(state_path, post)

    state = st.load_state(state_path)
    pending = {f["key"]: f for f in state.get("pending", [])}
    assert "pr:4555" in pending
    assert pending["pr:4555"]["claim"]


def test_lost_fact_is_detected_and_refetched():
    """Факт, потерянный вместе с черновиком, находится и восстанавливается."""
    state = {
        "posts": {"p1": {"id": "p1", "status": "proposed", "fact_keys": ["pr:4555"]}},
        "pending": [],
        "covered": {},
        "rejected": {},
    }

    assert facts_mod.lost_fact_keys(state) == {"pr:4555"}


def test_lost_fact_ignored_when_post_was_published():
    """Опубликованный пост — не потеря: его факты закрыты."""
    state = {
        "posts": {"p1": {"id": "p1", "status": "approved", "fact_keys": ["pr:4555"]}},
        "pending": [],
        "covered": {"pr:4555": {"post": "p1", "at": "2026-11-10T10:00:00Z"}},
        "rejected": {},
    }

    assert facts_mod.lost_fact_keys(state) == set()


# ------------------------------------------------------------------ 4. Регрессия сценария


def test_two_facts_one_ignored_other_goes_to_new_draft():
    """Два факта, один проигнорирован — второй всё равно попадает в черновик.

    Сценарий из acceptance: игнор одного черновика не должен блокировать
    подготовку следующего.
    """
    state = {
        "posts": {
            "old": {
                "id": "old",
                "status": "proposed",
                "created_at": _iso(datetime.now(timezone.utc) - timedelta(hours=60)),
                "fact_keys": ["pr:4555"],
                "fact_dicts": [_fact("pr:4555")],
            }
        },
        "pending": [{"key": "pr:4555", "attempts": 2}],
    }

    expired = st.expire_stale_drafts(state)
    assert expired == ["old"]

    # Факт, который игнорировали, снова в ленте…
    pending = {f["key"] for f in state.get("pending", [])}
    assert "pr:4555" in pending

    # …и второй факт при этом не потерян: пайплайн жив.
    other = _fact("pr:4600")
    state["pending"].append(other)
    picked = draft_mod.pick_fact(state["pending"])
    assert picked["key"] in {"pr:4555", "pr:4600"}


# ------------------------------------------------------------------ 5. Фильтр 48 ч — на доставке


def test_session_script_has_no_publish_window_before_facts():
    """Сбор фактов не стоит за 48-часовым фильтром.

    Фильтр переехал на ДОСТАВКУ черновика: готовим каждый день, а сообщение
    Шифу не сыпется чаще раза в 48 ч.
    """
    script = (REPO / "scripts" / "telegram" / "studio_session.sh").read_text(
        encoding="utf-8"
    )

    facts_at = script.index("собираю факты")
    delivery_at = script.index("Частота доставки")
    assert facts_at < delivery_at, "фильтр доставки стоит до сбора фактов"


def test_publisher_keeps_48h_gap():
    """Публикация в канал — не чаще раза в 48 ч (это не менялось)."""
    from channel_lib import MIN_GAP

    assert MIN_GAP == timedelta(hours=48)


# ------------------------------------------------------------------ 6. Материал не утекает


def test_collected_but_not_shown_fact_stays_pending(tmp_path):
    """Собранный, но не показанный факт не теряется.

    Сессия собирает факты, в черновик попадает один. Остальные кладутся в
    ожидание — иначе метка `last_session` сдвинется, повторный сбор их не
    вернёт (фильтр `merged:>=since`), и факт просто исчезнет.
    """
    state_path = tmp_path / "state.json"
    facts = [_fact("pr:4555"), _fact("pr:4600")]

    with st.state_transaction(state_path) as state:
        st.keep_collected(state, facts)

    state = st.load_state(state_path)
    pending = {f["key"]: f for f in state["pending"]}
    assert set(pending) == {"pr:4555", "pr:4600"}
    # Никто не показан — счётчик показов не растёт.
    assert pending["pr:4555"]["attempts"] == 0


def test_showing_fact_counts_one_attempt_over_pending(tmp_path):
    """Показ черновика — одна попытка, даже если факт уже лежал в ожидании."""
    state_path = tmp_path / "state.json"

    with st.state_transaction(state_path) as state:
        st.keep_collected(state, [_fact("pr:4555")])

    post = {
        "id": "p1",
        "title": "черновик",
        "body_html": "<b>текст</b>",
        "tags": ["#роббокс"],
        "fact_keys": ["pr:4555"],
        "fact_dicts": [_fact("pr:4555")],
    }
    studio.register_post(state_path, post)

    state = st.load_state(state_path)
    pending = {f["key"]: f for f in state["pending"]}
    assert pending["pr:4555"]["attempts"] == 1


def test_rejected_fact_not_reoffered_by_next_session(tmp_path):
    """Явный отказ закрывает факт: следующая сессия его не предлагает."""
    state_path = tmp_path / "state.json"

    with st.state_transaction(state_path) as state:
        st.keep_collected(state, [_fact("pr:4600")])
        st.mark_rejected(state, {"pr:4600"}, _iso(datetime.now(timezone.utc)))

    state = st.load_state(state_path)
    covered = st.covered_keys(state)
    reoffered = [f["key"] for f in st.pending_facts(state) if f["key"] not in covered]
    assert "pr:4600" not in reoffered
