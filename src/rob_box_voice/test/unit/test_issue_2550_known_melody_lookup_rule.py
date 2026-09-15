"""Static regression guard for issue #2550 — known-melody lookup enforcement.

Root cause (live DJ-set 2026-09-15): юзер 8 раз подряд просил «в пещере
горного короля», а модель НИ РАЗУ не вызвала ``lookup_melody`` для проверки
нот. Вместо этого ``execute_music_code`` получил отсебятину:

    p1 >> strangerarp(midinote=[52,52,55,52,59 ...]
    # Grieg mountain king melody (pe8le1f in E minor, classic ascending ostinato)

«pe8le1f» — выдуманный идентификатор (не MIDI-нота), «heartbeat Hawkins» —
выдуманный сэмпл. Тема НЕ сыграла, юзер жаловался «не слышу тему короля».

Fix:
  * ``prompts/skills/composer.txt`` — новый ``RULE #KNOWN-MELODY`` с явным
    списком композиторов и жёстким требованием: «если юзер назвал композитора
    или конкретное произведение — ОБЯЗАТЕЛЬНО первым делом lookup_melody,
    только ПОТОМ compose_music» (issue #2550 fix).
  * ``prompts/master_prompt_compact.txt`` — короткое ``RULE #KNOWN-MELODY``
    рядом с ``RULE #MUSIC-STATE``, чтобы enforcement читался при
    ``skills_enabled=true``, когда большая часть музыкальных правил уезжает
    в скилл composer.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/test_issue_2550_known_melody_lookup_rule.py
"""

from __future__ import annotations

from pathlib import Path
import re

_PROMPTS = Path(__file__).resolve().parents[2] / "prompts"
COMPOSER = _PROMPTS / "skills" / "composer.txt"
MASTER = _PROMPTS / "master_prompt_compact.txt"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _known_melody_rule_block_in_composer() -> str:
    """Body of ``RULE #KNOWN-MELODY`` в ``composer.txt``.

    Anchored на ``🎼 RULE #KNOWN-MELODY`` чтобы не перепутать с другими
    вхождениями «known». Заканчиваем на следующем ``🎼`` или
    ``⚠️ `` (новый блок) — в текущей структуре файла после правила идёт
    «Имя ищи на АНГЛИЙСКОМ...».
    """
    content = _read(COMPOSER)
    match = re.search(
        r"🎼 RULE #KNOWN-MELODY — .*?(?=\nИмя ищи на АНГЛИЙСКОМ|\n🚫 HONESTY RULE|\n🚨 RULE #NOTES|\n└─ 9\. A MOTIF|\n🎵 IMPERIAL|\n🎵 Stranger|\n🎵 RUSSIAN|\n🎵 HAPPY BIRTHDAY)",
        content,
        re.DOTALL,
    )
    assert match, (
        "RULE #KNOWN-MELODY block not found in composer.txt — expected a "
        "line starting with '🎼 RULE #KNOWN-MELODY — ' followed by the next "
        "section anchor ('Имя ищи на АНГЛИЙСКОМ' or one of the reference "
        "recipes). Issue #2550 fix depends on this enforcement block."
    )
    return match.group(0)


def _known_melody_rule_block_in_master() -> str:
    """Body of ``RULE #KNOWN-MELODY`` в ``master_prompt_compact.txt``.

    Anchored на ``🚨 **RULE #KNOWN-MELODY``. Заканчиваем на следующем
    ``🚨 **RULE #``.
    """
    content = _read(MASTER)
    match = re.search(
        r"🚨 \*\*RULE #KNOWN-MELODY — .*?(?=🚨 \*\*RULE #)",
        content,
        re.DOTALL,
    )
    assert match, (
        "RULE #KNOWN-MELODY block not found in master_prompt_compact.txt — "
        "expected a line starting with '🚨 **RULE #KNOWN-MELODY — ' followed "
        "by another '🚨 **RULE #'. Issue #2550 fix depends on this short "
        "enforcement (skills_enabled=true mode must still see it)."
    )
    return match.group(0)


# ── composer.txt: RULE #KNOWN-MELODY ──────────────────────────────────────


def test_composer_prompt_contains_known_melody_rule() -> None:
    """Анкер RULE #KNOWN-MELODY в composer.txt — regression guard."""
    content = _read(COMPOSER)
    assert "🎼 RULE #KNOWN-MELODY" in content, (
        "composer.txt lost RULE #KNOWN-MELODY — LLM will improvise known "
        "melodies from memory again (issue #2550). Restore the block — see "
        "test_issue_2550_known_melody_lookup_rule.py for context."
    )


def test_known_melody_rule_lists_composers() -> None:
    """Правило содержит список известных композиторов как lexical trigger.

    Без явного списка («Григ», «Бетховен», «Моцарт», ...) модель читает
    правило как абстракцию и не имеет лексического триггера для своих
    решений. DJ-сет 2026-09-15 — модель пропустила «Григ» мимо ушей.
    """
    block = _known_melody_rule_block_in_composer()
    composers = (
        "Григ", "Бетховен", "Моцарт", "Шопен", "Чайковский", "Бах",
        "Вивальди", "Рахманинов", "Шостакович", "Лист", "Стравинский",
        "Прокофьев", "Шуберт", "Дебюсси", "Равель", "Брамс", "Вагнер",
        "Паганини", "Бородин", "Мусоргский", "Римский-Корсаков",
    )
    missing = [name for name in composers if name not in block]
    assert not missing, (
        f"RULE #KNOWN-MELODY composer.txt missing composer names: "
        f"{', '.join(missing)}. Without these lexical triggers, LLM has no "
        "anchor to associate the rule with a user request (issue #2550)."
    )


def test_known_melody_rule_lists_specific_pieces() -> None:
    """Правило содержит конкретные произведения как lexical trigger.

    «Лунная соната», «к Элизе», «В пещере горного короля» — триггеры,
    которые юзер реально говорил. Без них модель не отличит «сыграй
    Бетховена» (общий класс) от «сыграй Лунную сонату» (конкретный трек).
    """
    block = _known_melody_rule_block_in_composer()
    pieces = (
        "к Элизе", "Лунная соната", "В пещере горного короля",
        "Кармен", "Щелкунчик", "Лебединое озеро", "Токката и фуга",
    )
    missing = [name for name in pieces if name not in block]
    assert not missing, (
        f"RULE #KNOWN-MELODY composer.txt missing piece names: "
        f"{', '.join(missing)}. Concrete titles are the strongest signal "
        "for LLM that this is a known-melody request (issue #2550)."
    )


def test_known_melody_rule_requires_lookup_first() -> None:
    """Шаг 1 правила — ОБЯЗАТЕЛЬНО lookup_melody.

    Без этого требования модель зовёт compose_music(name=...) сразу,
    и arranger сам подбирает ноты — что и есть баг (issue #1810 #2550).
    """
    block = _known_melody_rule_block_in_composer()
    assert "lookup_melody" in block, (
        "RULE #KNOWN-MELODY must require lookup_melody — that's the only "
        "way to anchor LLM on real RTTTL notes instead of improvising "
        "(issue #2550, regression of #1810)"
    )
    # Шаг 1 должен явно говорить «первым делом» / «СНАЧАЛА».
    assert "СНАЧАЛА" in block or "первым делом" in block or "**СНАЧАЛА**" in block, (
        "RULE #KNOWN-MELODY must explicitly require FIRST lookup_melody "
        "call — otherwise LLM treats it as an optional step and skips it "
        "(live DJ-set 2026-09-15 reproduced 8 times)"
    )


def test_known_melody_rule_orders_lookup_before_compose() -> None:
    """В правиле lookup_melody упоминается раньше compose_music.

    Если compose_music в тексте правила стоит первым — модель сразу
    зовёт его и забивает на lookup. Тут нужен именно порядок
    «lookup → compose», а не наоборот.
    """
    block = _known_melody_rule_block_in_composer()
    lookup_pos = block.find("lookup_melody")
    compose_pos = block.find("compose_music")
    assert lookup_pos >= 0 and compose_pos >= 0, (
        f"RULE #KNOWN-MELODY must reference both lookup_melody and "
        f"compose_music; got lookup_pos={lookup_pos}, compose_pos={compose_pos}"
    )
    assert lookup_pos < compose_pos, (
        "RULE #KNOWN-MELODY mentions compose_music BEFORE lookup_melody — "
        "model will skip the lookup step. Place lookup_melody first "
        "(issue #2550)."
    )


def test_known_melody_rule_bans_improvised_substitute() -> None:
    """Правило запрещает «Григ-подобный ostinato» / «Бетховен-стайл мотив».

    Это и есть ровно тот babble, который воспроизводился 8 раз подряд.
    Без явного BANNED модель генерирует такую отсебятину при первом же
    удобном случае (LLM hallucination).
    """
    block = _known_melody_rule_block_in_composer()
    # Хотя бы один из двух явных запретов должен быть в тексте.
    assert "Григ-подобный" in block or "Бетховен-стайл" in block, (
        "RULE #KNOWN-MELODY must ban 'Григ-подобный ostinato' / "
        "'Бетховен-стайл мотив' — that's the literal hallucination pattern "
        "from the DJ-set 2026-09-15 bug (issue #2550)"
    )
    # И явный BANNED/ЗАПРЕЩЕНО.
    assert "НЕ выдумывай" in block or "ЗАПРЕЩЕНО" in block or "BANNED" in block, (
        "RULE #KNOWN-MELODY must contain a 'НЕ выдумывай' / ЗАПРЕЩЕНО "
        "marker — without it the rule is advisory, not enforced"
    )


def test_known_melody_rule_handles_empty_lookup_result() -> None:
    """Правило описывает поведение при пустом lookup_melody.

    Если lookup вернул пусто (мелодии нет в RTTTL-библиотеке) — модель
    должна сказать «не нашёл» и НЕ придумывать MIDI. Без явного шага
    модель галлюцинирует дальше.
    """
    block = _known_melody_rule_block_in_composer()
    assert "пустой" in block or "пусто" in block, (
        "RULE #KNOWN-MELODY must describe what to do on empty "
        "lookup_melody result — without this, LLM hallucinates notes "
        "anyway (issue #2550 / #1810)"
    )
    # И должен быть «честный ответ» / «не нашёл».
    assert "не нашёл" in block or "не знаю" in block, (
        "RULE #KNOWN-MELODY must require honest 'not found' answer on "
        "empty lookup — that's the user's only signal that the library "
        "doesn't have this melody (issue #2550)"
    )


def test_known_melody_rule_references_issue_2550() -> None:
    """Правило ссылается на issue #2550 для трассировки.

    Будущие cleanup-агенты увидят «orphaned rule» и удалят. Issue-ссылка
    делает правило принадлежностью bug-report, а не «находкой».
    """
    block = _known_melody_rule_block_in_composer()
    assert "#2550" in block, (
        "RULE #KNOWN-MELODY must reference issue #2550 — otherwise future "
        "cleanup agents may treat it as orphaned and remove it"
    )


# ── master_prompt_compact.txt: компактный RULE #KNOWN-MELODY ──────────────


def test_master_prompt_contains_known_melody_rule() -> None:
    """RULE #KNOWN-MELODY присутствует в мастер-промпте."""
    content = _read(MASTER)
    assert "🚨 **RULE #KNOWN-MELODY" in content, (
        "master_prompt_compact.txt lost RULE #KNOWN-MELODY — at "
        "skills_enabled=true the long composer.txt moves out, so the short "
        "reminder is the only anchor LLM sees (issue #2550)"
    )


def test_master_known_melody_rule_mentions_lookup_melody() -> None:
    """Краткое правило в мастер-промпте явно требует lookup_melody."""
    block = _known_melody_rule_block_in_master()
    assert "lookup_melody" in block, (
        "master_prompt RULE #KNOWN-MELODY must mention lookup_melody — "
        "without the tool name LLM has nothing to call"
    )


def test_master_known_melody_rule_placed_near_music_state() -> None:
    """Краткое правило стоит рядом с ``RULE #MUSIC-STATE``.

    Это гарантирует, что правило попадает в один read-сегмент с уже
    известной LLM группой музыкальных правил (RULE #MUSIC / #MUSIC-STATE).
    """
    content = _read(MASTER)
    music_state = re.search(r"🚨 \*\*RULE #MUSIC-STATE — ", content)
    known_melody = re.search(r"🚨 \*\*RULE #KNOWN-MELODY — ", content)
    assert music_state, "RULE #MUSIC-STATE anchor missing — cannot check ordering"
    assert known_melody, "RULE #KNOWN-MELODY anchor missing"
    assert known_melody.start() > music_state.start(), (
        "RULE #KNOWN-MELODY must come AFTER RULE #MUSIC-STATE — group all "
        "music-related enforcement in one place for LLM attention "
        "(issue #2550)"
    )


def test_master_known_melody_rule_stays_invariant() -> None:
    """Краткое правило в мастер-промпте НЕ уезжает в скиллы.

    §1/§2 — инварианты мастер-промпта; ``RULE #KNOWN-MELODY`` про музыку,
    но это ENFORCEMENT (а не «раздел скилла»), и должен жить в мастере
    даже при ``skills_enabled=true``. Тестовая проверка: при
    рендере с флагом правило не должно пропасть.
    """
    from rob_box_core.prompt_sections import render_prompt

    content = _read(MASTER)
    rendered_enabled = render_prompt(content, skills_enabled=True).system_prompt
    rendered_disabled = render_prompt(content, skills_enabled=False).system_prompt
    assert "🚨 **RULE #KNOWN-MELODY" in rendered_enabled, (
        "RULE #KNOWN-MELODY disappeared from master_prompt when "
        "skills_enabled=true — it's an enforcement rule and must stay in "
        "the master prompt at every flag (issue #2550)"
    )
    assert "🚨 **RULE #KNOWN-MELODY" in rendered_disabled, (
        "RULE #KNOWN-MELODY missing in master_prompt at skills_enabled=false"
    )


# ── композитор не выдаёт новые правила за инварианты ──────────────────────


def test_known_melody_rule_does_not_redefine_lang_or_anti_dup() -> None:
    """RULE #KNOWN-MELODY — это enforcement, не переопределение инвариантов.

    Правило может ССЫЛАТЬСЯ на анти-дупликат (RULE #0 etc.), но не должно
    их копировать — иначе появится второй источник правды.
    """
    block = _known_melody_rule_block_in_composer()
    for marker in ("RULE #LANG —", "RULE #UNICODE-SPEECH", "RULE #SYSCTX"):
        assert marker not in block, (
            f"RULE #KNOWN-MELODY composer.txt must not redefine {marker} — "
            "this would create a second source of truth and break the "
            "single-invariant contract (see test_skill_prompt_contract.py)"
        )