"""Короткое описание играющего трека из его фактов (#4060, ADR-0148).

Шифу 27.10: «хочу услышать, что именно сейчас играет, а не номер трека. У каждого трека есть название, стиль и т. д.».
Раньше ответ собирался из того, что было под рукой: ``клубный трек · 136 BPM`` в ``<music_state>``, «трек 3 из 10» от
диджея, ключ ``gen48211:00:A`` в данных тула. Теперь описание строит одна функция :func:`track_brief`; её зовут все, кто
говорит о звучащем треке: ответ «что играет» (``dj_line.now_playing_text``, ``media_phrases.now_playing_track_text``),
снимок плеера (поле ``brief`` — его читает ``<music_state>`` хода: ``track``) и ``get_music_state``.

Описание — только факты: название (генерации, каталога, кавера или мелодия сета), автор, стиль, движок, темп, инструмент,
у кавера — оригинал. Ключей кода (``track_id``, ``library_id``, ``set_id``) и номера трека в нём нет. Чего в фактах нет,
того в описании нет: темп сгенерированного файла неизвестен (``bpm`` снимка — темп деки, а не файла), его не называем.
"""

from __future__ import annotations

from typing import Any, List, Mapping

from . import knowledge as kn

#: Факты снимка плеера (поле ``dj``), из которых складывается описание. Ключи кода в него не входят.
BRIEF_FACTS = ("name", "melody", "composer", "cover", "style", "bpm", "engine", "source", "instrument_played")
#: Источники, у которых ``bpm`` снимка — темп деки (60 или темп сета), а не темп звучащего файла: темп не называем.
TEMPO_UNKNOWN_SOURCES = frozenset({"generated"})
#: Источники без настоящего названия (#4060): свой синт-трек собран кодом по теме заказа, у него нет названия из каталога,
#: генерации или кавера. Слова заказа («какую нибудь спокойную музыку») названием не служат: живьём 28.10 06:13 они попали
#: в кавычки как «название» трека. Такой трек описывает стиль, движок и темп; у трека сета название - мелодия (``melody``).
UNTITLED_SOURCES = frozenset({"synth"})
#: Движок трека словами (ключи ``knowledge.MUSIC_ENGINES``) и движок, которым играет источник.
ENGINE_WORDS = {"synth": "синтез Ренардо"}
ENGINE_OF_SOURCE = {"synth": "synth"}
#: Как названа безымянная часть: «трек в стиле синтвейв» (названия нет — описание начинается с неё).
UNNAMED_TRACK = "трек"


def plural(n: int, one: str, few: str, many: str) -> str:
    """Склонение существительного к числу ``n``: 1 удар, 2 удара, 5 ударов."""
    if n % 10 == 1 and n % 100 != 11:
        return one
    return few if 2 <= n % 10 <= 4 and not 12 <= n % 100 <= 14 else many


def tempo_text(facts: Mapping[str, Any]) -> str:
    """«130 ударов в минуту»; темпа нет или он не темп звучащего файла — ``""``."""
    if str(facts.get("source") or "") in TEMPO_UNKNOWN_SOURCES:
        return ""
    try:
        bpm = int(round(float(facts.get("bpm") or 0)))
    except (TypeError, ValueError):
        return ""
    return f"{bpm} {plural(bpm, 'удар', 'удара', 'ударов')} в минуту" if bpm > 0 else ""


def style_name(facts: Mapping[str, Any]) -> str:
    """Имя стиля словами (``knowledge.GEN_STYLES``: «синтвейв», «клуб»); стиля нет или он не из таблицы — ``""``."""
    style = kn.GEN_STYLES.get(str(facts.get("style") or ""))
    return style.name if style is not None else ""


def engine_word(facts: Mapping[str, Any]) -> str:
    """Чем играет: движок назван в фактах или его даёт источник («синтез Ренардо»)."""
    key = str(facts.get("engine") or ENGINE_OF_SOURCE.get(str(facts.get("source") or ""), ""))
    return ENGINE_WORDS.get(key, "")


def instrument_text(facts: Mapping[str, Any]) -> str:
    """«на органе» — инструмент, на котором сыграно (``knowledge.INSTRUMENTS``); нет — ``""``."""
    lead = kn.INSTRUMENTS.get(str(facts.get("instrument_played") or ""))
    return f"на {lead.prepositional}" if lead is not None else ""


def is_untitled(facts: Mapping[str, Any]) -> bool:
    """Источник трека названия не имеет (:data:`UNTITLED_SOURCES`): ни ``name``, ни ``title`` снимка названием не служат."""
    return str(facts.get("source") or "") in UNTITLED_SOURCES


def track_name(facts: Mapping[str, Any]) -> str:
    """Настоящее название трека: каталога, генерации или кавера, иначе мелодия трека сета; нет — ``""``. У источника
    без названия (:data:`UNTITLED_SOURCES`) слова заказа в ``name`` в счёт не идут - только мелодия сета."""
    melody = str(facts.get("melody") or "").strip()
    return melody if is_untitled(facts) else str(facts.get("name") or melody).strip()


def named(facts: Mapping[str, Any]) -> str:
    """«Название» в кавычках, автор — за кавычками, как во фразе запуска (``media_phrases.piece_text``); названия нет —
    ``""``."""
    name = track_name(facts)
    if not name:
        return ""
    head = name if name.startswith("«") else f"«{name}»"
    composer = str(facts.get("composer") or "").strip()
    return f"{head}, {composer}" if composer and composer.lower() not in name.lower() else head


def cover_text(facts: Mapping[str, Any]) -> str:
    """У кавера — оригинал: «кавер на «Группа крови»»; название кавера само называет оригинал — просто «кавер»."""
    original = str(facts.get("cover") or "").strip()
    if not original:
        return ""
    return "кавер" if original.lower() in track_name(facts).lower() else f"кавер на «{original}»"


def track_traits(facts: Mapping[str, Any]) -> List[str]:
    """Черты трека по порядку: стиль, движок, темп, инструмент — те, что в фактах есть."""
    return [t for t in (style_name(facts), engine_word(facts), tempo_text(facts), instrument_text(facts)) if t]


def track_brief(facts: Mapping[str, Any]) -> str:
    """Описание трека из фактов: «Название», автор, оригинал кавера, стиль, движок, темп, инструмент; без названия —
    «трек в стиле синтвейв, 130 ударов в минуту». Описывать нечем (нет ни названия, ни оригинала, ни стиля: темп, движок
    и инструмент одни трека не описывают) — ``""``: говорить нечего, звать не с чем."""
    head = [p for p in (named(facts), cover_text(facts)) if p]
    traits = track_traits(facts)
    if not head:
        style = style_name(facts)  # стиль, если он есть, — первая черта
        if not style:
            return ""
        traits[0] = f"{UNNAMED_TRACK} в стиле {style}"
    return ", ".join([*head, *traits])


__all__ = ["BRIEF_FACTS", "ENGINE_WORDS", "TEMPO_UNKNOWN_SOURCES", "UNTITLED_SOURCES", "engine_word", "instrument_text",
           "is_untitled", "named", "plural", "style_name", "tempo_text", "track_brief", "track_name", "track_traits"]
