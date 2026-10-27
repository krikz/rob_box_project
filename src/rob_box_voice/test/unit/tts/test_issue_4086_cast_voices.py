"""#4086: голоса каста марафона в каталоге робота - «говори голосом Весельчак» работает честно.

Было (живая проверка 27.10, dev-0d94634): «Робот, говори голосом English_Jovialman от MiniMax» - модель вызвала
``list_tts_voices``, в каталоге MiniMax было 10 голосов, без каста, и честно ответила «такого голоса нет». Голоса
каста (``English_Jovialman`` и др.) знал только харнесс e2e.

Проверяют ПОВЕДЕНИЕ реестра и синтеза, а не текст промпта (ADR-0148):

* голоса каста есть в каталоге MiniMax с русскими именами, полом и нативным языком;
* голос называют id или именем - ``voice_id_by_name`` / ``resolve_voice`` дают id каталога;
* имя без голоса и чужого провайдера - отказ кодом (``fell_back``), не «угадывание»;
* синтез MiniMax получает id каталога, а в лог идёт ``voice_used=<id>``.

Сверка «каждый голос каста есть в каталоге» (источник каста - ``map_tts_voice`` харнесса) лежит рядом с тестами
харнесса: ``tests/unit/e2e_scripts/test_issue_4086_cast_in_catalog.py``.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice import tts_voice_registry as registry  # noqa: E402
from rob_box_voice.tts_voice_registry import (  # noqa: E402
    PROVIDER_VOICES,
    VOICE_METADATA,
    format_tts_context,
    language_of,
    resolve_voice,
    role_voices_for,
    sing_preset_for,
    voice_id_by_name,
    voice_info_for,
    voices_info_for,
)
from test.unit.tts.test_voice_selection import (  # noqa: E402
    _playback_node,
    _run_with_voice,
)

#: (id, русское имя, пол) - как в каталоге и в карточке #4086.
CAST = (
    ("English_Jovialman", "Весельчак", "male"),
    ("English_WiseScholar", "Мудрый учёный", "male"),
    ("Chinese (Mandarin)_Gentle_Senior", "Добрая бабушка", "female"),
    ("English_Whispering_girl", "Шёпот", "female"),
    ("Portuguese_StrictBoss", "Строгая начальница", "female"),
)


# ── каталог ──────────────────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("voice_id,name,gender", CAST)
def test_cast_voice_is_in_the_minimax_catalogue_with_a_human_name(
    voice_id: str, name: str, gender: str
) -> None:
    assert voice_id in PROVIDER_VOICES["minimax"]
    info = voice_info_for("minimax", voice_id)
    assert info is not None
    assert (info["display_name"], info["gender"]) == (name, gender)


def test_cast_voices_are_visible_to_the_model_in_the_tts_context_line() -> None:
    """Модель видит голоса в строке ``[TTS] ... voices: ...`` и не идёт в ``list_tts_voices`` за проверкой."""
    line = format_tts_context("minimax")
    assert all(voice_id in line for voice_id, _, _ in CAST), line


def test_cast_voices_keep_their_native_language_and_stay_out_of_tale_roles() -> None:
    """Русский текст они читают с акцентом: в роли сказки (#3998) не идут, язык в каталоге - нативный."""
    assert [language_of("minimax", v) for v, _, _ in CAST] == [
        "en",
        "en",
        "zh",
        "en",
        "pt",
    ]
    roles = role_voices_for("minimax")
    assert roles and not {v for v, _, _ in CAST} & set(roles)


def test_wire_voice_info_keeps_its_five_fields() -> None:
    """Quest-пикер читает ``VoiceInfo``: новых полей в проводе нет (имена-алиасы живут отдельно)."""
    for info in voices_info_for("minimax"):
        assert set(info) == {
            "voice_id",
            "display_name",
            "language",
            "gender",
            "presets",
        }, info


@pytest.mark.parametrize("voice_id,_name,_gender", CAST)
def test_cast_voice_can_be_sung(voice_id: str, _name: str, _gender: str) -> None:
    """Пресет пения строится кодом из каталога: голос каста поёт без правок в двух местах (ADR-0158)."""
    assert sing_preset_for("minimax", voice_id).vocal_gender in ("m", "f")


# ── имя -> id ────────────────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "spoken",
    [
        "Весельчак",
        "весельчак",
        "  Весельчак ",
        "Jovialman",
        "jovialman",
        "English_Jovialman",
        "english jovialman",
        "Джовиалмен",
        "джовиалмэн",
    ],
)
def test_jovialman_is_found_by_every_name_a_human_or_stt_gives(spoken: str) -> None:
    assert voice_id_by_name("minimax", spoken) == "English_Jovialman"
    assert resolve_voice("minimax", spoken) == ("English_Jovialman", False)


@pytest.mark.parametrize(
    "spoken,voice_id",
    [
        ("Мудрый ученый", "English_WiseScholar"),
        ("Wise Scholar", "English_WiseScholar"),
        ("добрая бабушка", "Chinese (Mandarin)_Gentle_Senior"),
        ("Gentle_Senior", "Chinese (Mandarin)_Gentle_Senior"),
        ("шепот", "English_Whispering_girl"),
        ("Строгая начальница", "Portuguese_StrictBoss"),
        ("Strict Boss", "Portuguese_StrictBoss"),
    ],
)
def test_the_rest_of_the_cast_is_found_by_name(spoken: str, voice_id: str) -> None:
    assert resolve_voice("minimax", spoken) == (voice_id, False)


@pytest.mark.parametrize("provider", sorted(PROVIDER_VOICES))
def test_every_catalogue_voice_is_found_by_its_id_and_by_its_name(
    provider: str,
) -> None:
    """Имя каталога однозначно: у двух голосов одного провайдера общего имени нет (иначе имя ничего не выберет)."""
    for voice_id in PROVIDER_VOICES[provider]:
        name = VOICE_METADATA[f"{provider}:{voice_id}"]["display_name"]
        assert voice_id_by_name(provider, voice_id) == voice_id, (provider, voice_id)
        assert voice_id_by_name(provider, name) == voice_id, (provider, voice_id, name)


def test_a_name_of_two_voices_selects_none(monkeypatch: pytest.MonkeyPatch) -> None:
    """Двусмысленное имя - не «первый попавшийся», а отказ: решает код, что голоса нет."""
    monkeypatch.setitem(
        registry._NAME_INDEX["minimax"],
        "двойник",
        {"Russian_ReliableMan", "Russian_AttractiveGuy"},
    )
    assert voice_id_by_name("minimax", "Двойник") == ""
    assert resolve_voice("minimax", "Двойник") == ("male-qn-qingse", True)


# ── отказ ────────────────────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "spoken", ["Терминатор", "Jovial", "English", "", None, "alena"]
)
def test_unknown_name_is_refused_to_the_provider_default(spoken) -> None:
    """Нет такого голоса (или это голос другого провайдера) - ``fell_back``, как и раньше (Q6)."""
    assert voice_id_by_name("minimax", spoken) == ""
    assert resolve_voice("minimax", spoken) == ("male-qn-qingse", True)


def test_name_of_another_provider_voice_is_not_found_here() -> None:
    assert voice_id_by_name("minimax", "Алёна") == ""
    assert voice_id_by_name("yandex", "Алёна") == "alena"
    assert resolve_voice("yandex", "алена") == ("alena", False)


def test_unknown_provider_has_no_names() -> None:
    assert voice_id_by_name("elevenlabs", "Весельчак") == ""


# ── синтез: «говори голосом Весельчак» -> MiniMax voice_used=English_Jovialman ───────────────────────────────


def test_minimax_synthesis_gets_the_catalogue_id_when_asked_by_name() -> None:
    node = _playback_node()
    _run_with_voice(node, voice="Весельчак")
    node._synthesize_minimax.assert_called_once()
    _, kwargs = node._synthesize_minimax.call_args
    assert kwargs.get("voice") == "English_Jovialman"
    logged = " ".join(str(call) for call in node.logger.info.call_args_list)
    assert "voice_used=English_Jovialman" in logged
    assert not node.logger.warn.called  # подмены на дефолт нет


def test_minimax_synthesis_with_the_id_of_a_cast_voice() -> None:
    node = _playback_node()
    _run_with_voice(node, voice="English_Jovialman")
    _, kwargs = node._synthesize_minimax.call_args
    assert kwargs.get("voice") == "English_Jovialman"


def test_minimax_synthesis_of_unknown_voice_still_falls_back_loudly() -> None:
    """Голоса нет в каталоге: дефолт провайдера и WARN в логе (не «молча приняли»)."""
    node = _playback_node()
    _run_with_voice(node, voice="Терминатор")
    _, kwargs = node._synthesize_minimax.call_args
    assert kwargs.get("voice") == "male-qn-qingse"
    assert node.logger.warn.called
