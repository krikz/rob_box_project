"""#3780: эксперимент чойсера вайба. Выбор из списка применяется, ответ вне списка — нет.

Сети и LLM нет: выбирающая функция подставная. Боевой путь (шар) этот модуль не заменяет.
"""

from rob_box_dialog import knowledge as kn
from rob_box_dialog.vibe_chooser import NEUTRAL, accept, choose, prompt, slot_names

#: Имена, которые уже есть на develop: фоны и импульсы полотна, без ``none``.
LIVE = ("rainbow", "fire", "matrix", "plasma", "rain", "slash", "glitch", "boot", "power_down")


def test_slot_names_are_the_live_bg_and_fx_values():
    """Список берётся из таблицы знания, не из выдуманных эффектов."""
    assert slot_names() == LIVE
    assert set(LIVE) <= set(kn.FACE_SCENE_SLOTS["bg"]) | set(kn.FACE_SCENE_SLOTS["fx"])
    assert "none" not in slot_names()
    assert "disco" not in slot_names()


def test_answer_from_the_list_is_applied():
    seen = []

    def picker(text):
        seen.append(text)
        return "fire"

    assert choose("рок и азарт", LIVE, picker) == "fire"
    assert "рок и азарт" in seen[0]
    assert "fire" in seen[0] and "rainbow" in seen[0] and NEUTRAL in seen[0]


def test_disco_is_not_applied():
    assert choose("дискотека", LIVE, lambda _text: "disco") == NEUTRAL
    assert accept("disco", LIVE) == NEUTRAL
    assert accept("  Disco  ", LIVE) == NEUTRAL


def test_empty_context_does_not_raise():
    assert choose("", LIVE, lambda text: "rain") == "rain"
    assert "Контекст: \n" in prompt("", LIVE)
    assert choose("", LIVE, lambda _text: "") == NEUTRAL
    assert choose(None, LIVE, lambda _text: "matrix") == "matrix"  # type: ignore[arg-type]


def test_picker_failure_and_junk_stay_neutral():
    def boom(_text):
        raise RuntimeError("нет сети в эксперименте")

    assert choose("контекст", LIVE, boom) == NEUTRAL
    assert choose("контекст", LIVE, lambda _text: None) == NEUTRAL
    assert choose("контекст", LIVE, lambda _text: ["fire"]) == NEUTRAL
    assert choose("контекст", (), lambda _text: "fire") == NEUTRAL
