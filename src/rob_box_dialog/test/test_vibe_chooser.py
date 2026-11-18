"""#3780: эксперимент чойсера вайба. Выбор из списка применяется, ответ вне списка — нет.

Сети и LLM нет: выбирающая функция подставная. Боевой путь (шар) этот модуль не заменяет.
"""

from rob_box_dialog import knowledge as kn
from rob_box_dialog.music_state import MusicStateMemory
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.shift_log import ShiftLog
from rob_box_dialog.turn_log import TurnLog
from rob_box_dialog.vibe import VibeBall
from rob_box_dialog.vibe_chooser import (
    CRITERIA,
    NEUTRAL,
    NO_DATA,
    SILENCE,
    accept,
    choose,
    prompt,
    slot_names,
    vibe_context,
)
from rob_box_dialog_host.music_player_state import MusicPlayerState

#: Имена, которые уже есть на develop: фоны и импульсы полотна, без ``none``.
LIVE = (
    "rainbow",
    "fire",
    "matrix",
    "plasma",
    "rain",
    "slash",
    "glitch",
    "boot",
    "power_down",
)


class _Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


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
    assert "rainbow — вайб" in seen[0] and "fire — разогрев" in seen[0]
    assert "matrix — думаю, ищу, считаю" in seen[0]
    assert "plasma — медленный фон" in seen[0] and "rain — редкие капли" in seen[0]
    assert "рок" not in seen[0].split("Контекст:")[0]


def test_disco_is_not_applied():
    assert choose("дискотека", LIVE, lambda _text: "disco") == NEUTRAL
    assert accept("disco", LIVE) == NEUTRAL
    assert accept("  Disco  ", LIVE) == NEUTRAL


def test_empty_context_does_not_raise():
    assert choose("", LIVE, lambda text: "rain") == "rain"
    assert "Контекст: \n" in prompt("", LIVE)
    assert choose("", LIVE, lambda _text: "") == NEUTRAL
    assert choose(None, LIVE, lambda _text: "matrix") == "matrix"  # type: ignore[arg-type]


class _Facts:
    """Снимок сессии плюс журнал смены: поля, которые уже есть, без выдуманного «сейчас в кадре»."""

    def __init__(self, snap, shift):
        self.snapshot = snap
        self.shift = shift


def _playing(title: str) -> MusicStateMemory:
    """Имя — как у живого плеера: снимок, затем ``/voice/music/form`` (``observe_track_name``)."""
    memory = MusicStateMemory()
    memory.observe_state(MusicPlayerState("playing", track_id="t-1", ts=10.0))
    memory.observe_track_name(title)
    return memory


def test_playing_title_lands_in_the_context():
    text = vibe_context(_playing("X"))
    assert "Музыка: играет X" in text
    assert "X" in text
    seen = []
    assert (
        choose(text, LIVE, lambda prompt_text: seen.append(prompt_text) or "matrix")
        == "matrix"
    )
    assert "играет X" in seen[0]


def test_silence_is_the_word_silence():
    memory = MusicStateMemory()
    memory.observe_state(MusicPlayerState("idle", ts=10.0))
    text = vibe_context(memory)
    assert f"Музыка: {SILENCE}" in text
    assert "играет" not in text.split("Реплики:")[0]


def test_missing_facts_are_named_not_invented():
    text = vibe_context()
    assert text.count(NO_DATA) == 6
    assert "люди: пусто" not in text
    assert "Предыдущий вайб: нет данных" in text
    assert "Текущий вайб: нет данных" in text
    assert "Текущий включён: нет данных" in text
    bare = vibe_context(state=Session("personality").snapshot)
    assert "люди: нет данных" in bare and "люди: пусто" not in bare


def test_ball_snapshot_names_previous_current_and_since():
    """push fire, затем matrix: предыдущий fire, текущий matrix, время второй смены."""
    clock = _Clock()
    ball = VibeBall(clock)
    clock.t = 10.0
    ball.push("fire")
    clock.t = 25.0
    snap = ball.push("matrix")
    text = vibe_context(ball=snap)
    assert "Предыдущий вайб: fire" in text
    assert "Текущий вайб: matrix" in text
    assert "Текущий включён: 25.0" in text
    since_value = text.split("Текущий включён:")[1].split("\n")[0].strip()
    assert since_value == "25.0"
    assert "10.0" not in since_value


def test_without_ball_vibe_lines_are_no_data_not_invented_style():
    text = vibe_context()
    assert "Предыдущий вайб: нет данных" in text
    assert "Текущий вайб: нет данных" in text
    assert "Текущий включён: нет данных" in text
    assert "Текущий вайб: fire" not in text
    assert "Текущий вайб: matrix" not in text
    assert "Текущий вайб: rainbow" not in text


def test_prompt_keeps_shifu_criteria_and_has_no_rock():
    text = prompt("что угодно", LIVE)
    assert all(name in text for name, _when in CRITERIA)
    assert "разогрев" in text and "думаю, ищу, считаю" in text
    assert "рок" not in text.split("Контекст:")[0]


def test_log_fragment_and_people_come_from_existing_facts():
    log = TurnLog()
    log.begin("t1", 0, "давай ещё", "Саша")
    log.said("t1", "s1", "уже грею", delivered=True)
    session = Session("personality", clock=lambda: 100.0)
    session.set_music(MusicStateRef("playing", track_id="t-1", title="X"))
    shift = ShiftLog(clock=lambda: 100.0)
    shift.seen("Саша", "face")
    text = vibe_context(_playing("X"), log, _Facts(session.snapshot, shift), now=100.0)
    assert "давай ещё" in text and "робот: уже грею" in text
    assert (
        "кадров нет" in text
        and "речь молчит" in text
        and "музыка сессии: играет X" in text
    )
    assert "люди: Саша" in text
    assert all(name in prompt("x", LIVE) for name, _when in CRITERIA)


def test_picker_failure_and_junk_stay_neutral():
    def boom(_text):
        raise RuntimeError("нет сети в эксперименте")

    assert choose("контекст", LIVE, boom) == NEUTRAL
    assert choose("контекст", LIVE, lambda _text: None) == NEUTRAL
    assert choose("контекст", LIVE, lambda _text: ["fire"]) == NEUTRAL
    assert choose("контекст", (), lambda _text: "fire") == NEUTRAL
