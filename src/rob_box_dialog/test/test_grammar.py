"""Tier-1 грамматика (ADR-0150 §4.2, §6.3, В19; карточка #3696): корпус фраз ≥ 200, SILENCED, каналы громкости."""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Ask
from rob_box_dialog.grammar import Tier1Command, match, volume_channel
from rob_box_dialog.session import MusicStateRef, SceneFrame, SessionSnapshot


def _snap(**kw):
    return SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0, **kw)


IDLE = _snap()
SILENCED = _snap(scenes=(SceneFrame("Silence", "user"),))
MUSIC = _snap(music=MusicStateRef("playing", title="В пещере горного короля"))
SPEAKING = _snap(speaking=True)
MUSIC_AND_SPEECH = _snap(music=MusicStateRef("playing"), speaking=True)
LAST_MUSIC = _snap(volume_channel="music")
ASKED = _snap(pending_ask=Ask("yes_no", "Выключить и зачитать?"))
VOICES = [
    ("Russian_ReliableMan", "Надёжный мужчина"), ("Russian_BrightHeroine", "Яркая героиня"),
    ("Russian_AttractiveGuy", "Привлекательный парень"), ("Russian_PessimisticGirl", "Пессимистичная девушка"),
    ("Russian_CrazyQueen", "Безумная королева"), ("male-qn-qingse", "Qingse (legacy)"),
]
YANDEX = [("anton", "Антон"), ("alena", "Алёна"), ("zahar", "Захар"), ("ermil", "Ермил"), ("jane", "Джейн")]


def _kind(text, snap=IDLE, voices=VOICES):
    command = match(text, snap, voices)
    return None if command is None else command.kind


# --------------------------------------------------------------------------------------------------------------
# Стоп и тишина: ясные (код, 0 LLM)
# --------------------------------------------------------------------------------------------------------------

STOP_ALL = [
    "Робот, стоп", "робот стоп", "РОББОКС СТОП", "Робот, стой", "робот стойте", "Робби, отставить",
    "Роббокс, стоп-стоп", "эй робот стоп", "робот ну стоп", "Робот, всё, стоп", "стоп", "стой", "Стоп!",
    "Робот, стоп.", "ну стоп уже", "да стой", "робот, стоп, стоп", "Робот, ладно, стоп", "слушай, стоп",
    "а стой",
]
STOP_ALL += [  # «хватит» — остановить звуки, не тишина (Шифу 22.10, #3700)
    "Робот, хватит", "робот хватит", "Робот, всё, хватит", "робот ну хватит уже", "робот да хватит", "хватит",
    "Хватит!", "хватит робот",
]
SILENCE = [
    "Робот, замолчи", "робот замолчите", "Робот, помолчи", "помолчите", "замолчи уже", "эй, помолчи",
    "Робот, всё, хватит, замолчи", "робот хватит, помолчи", "Робот, замолчи, хватит",  # «замолчи» в серии — тишина
]
STOP_WITH_REST = [
    ("Робот, всё, хватит, спой другую", "спой другую"),
    ("Робот, стоп. Включи музыку", "Включи музыку"),
    ("хватит, расскажи анекдот", "расскажи анекдот"),
    ("стой, повернись налево", "повернись налево"),
    ("Робот, замолчи и включи свет", "и включи свет"),
    ("стоп стоп, давай другую сказку", "давай другую сказку"),
    ("ну хватит уже, лучше спой", "лучше спой"),
    ("Робот, стоп, забудь всё", "забудь всё"),
]


@pytest.mark.parametrize("text", STOP_ALL)
def test_clear_stop_is_stop_all(text):
    command = match(text, IDLE)
    assert command == Tier1Command("stop_all"), text


@pytest.mark.parametrize("text", SILENCE)
def test_clear_silence_word_alone_is_silence(text):
    command = match(text, IDLE)
    assert command == Tier1Command("silence"), text


@pytest.mark.parametrize("snap", [IDLE, SILENCED, MUSIC, SPEAKING, MUSIC_AND_SPEECH])
def test_enough_is_stop_all_in_every_state_and_never_silence(snap):
    """«Хватит» (Шифу 22.10, #3700) — остановить звуки: вид один при любом состоянии, режима тишины он не даёт."""
    assert match("Робот, хватит", snap) == Tier1Command("stop_all")
    assert _kind("Робот, замолчи", snap) == "silence"


@pytest.mark.parametrize("text, rest", STOP_WITH_REST)
def test_clear_stop_with_new_command_keeps_the_rest(text, rest):
    command = match(text, IDLE)
    assert command is not None and command.kind == "stop_all" and not command.disputed
    assert command.rest == rest


# --- #3994: охват стопа — «хватит читать» гасит речь, плеер музыки остаётся --------------------------------------

STOP_SPEECH = [  # (фраза, keep_music)
    ("Робот, хватит читать, а музыку оставь", True),  # n811, акт 8
    ("робот хватит читать а музыку оставь", True),  # то же из STT, без запятых
    ("Робот, хватит, музыку оставь", True),
    ("Робот, стоп, а бит не трогай", True),
    ("Робот, хватит читать", False),
    ("робот хватит говорить", False),
    ("Робот, хватит рассказывать", False),
    ("Робот, хватит петь", False),
    ("робот ну хватит уже болтать", False),
]


@pytest.mark.parametrize("text, keep", STOP_SPEECH)
@pytest.mark.parametrize("snap", [IDLE, MUSIC, MUSIC_AND_SPEECH, SILENCED])
def test_3994_stop_speech_names_speech_and_leaves_no_rest(text, keep, snap):
    """Речь (и «музыку оставь») покрыта целиком: вид ``stop_speech``, остатка для LLM нет; в тишине тоже работает."""
    assert match(text, snap) == Tier1Command("stop_speech", {"keep_music": keep}), text


@pytest.mark.parametrize("text, rest", [
    ("Робот, хватит, спой другую", "спой другую"),  # решение по умолчанию (Шифу 26.10): стоп всего + остаток
    ("Робот, хватит, сыграй другую", "сыграй другую"),
    ("Робот, хватит музыки", ""),
    ("Робот, стоп музыка", ""),
    ("Робот, хватит играть", ""),
    ("Робот, хватит читать и играть", ""),
    ("Робот, останови", ""),  # #4020 n709
    ("Робот, остановись", ""),
    ("робот остановите", ""),
    ("Робот, останови, Борис уже уснул под неё", "Борис уже уснул под неё"),
])
def test_3994_music_named_nothing_named_or_new_request_stays_stop_all(text, rest):
    command = match(text, MUSIC)
    assert command is not None and (command.kind, command.rest, command.disputed) == ("stop_all", rest, False), text


def test_3994_stop_speech_with_a_new_request_keeps_only_the_uncovered_rest():
    command = match("Робот, хватит читать, а музыку сделай потише", MUSIC)
    assert command == Tier1Command("stop_speech", {"keep_music": False}, rest="музыку сделай потише")


def test_3994_shut_up_keeping_music_is_still_silence():
    """«Замолчи, а музыку оставь» — тишина (она музыку и так не трогает), а не стоп речи."""
    assert match("Робот, замолчи, а музыку оставь", MUSIC) == Tier1Command("silence")


@pytest.mark.parametrize("text", ["Робот, где остановка автобуса", "робот останова двигателя"])
def test_4020_stop_lookalikes_are_not_tier1_stops(text):
    command = match(text, MUSIC)
    assert command is None or command.kind not in ("stop_all", "stop_speech", "silence"), text


# --------------------------------------------------------------------------------------------------------------
# В19: корпус пар «срабатывает / не срабатывает» (требование Шифу 20.10; те же фразы — scripts/dialog/reference_set.yaml r65–r83)
# --------------------------------------------------------------------------------------------------------------

V19_PAIRS = [
    ("Робот, стоп", "Робот, покажи анимацию стоп"),
    ("Робот, хватит", "Робот, спой песню про хватит"),
    ("Робот, хватит", "Робот, расскажи сказку про то, как ёж сказал хватит"),
    ("Робот, замолчи", "Робот, найди песню замолчи"),
    ("Робот, стой", "Робот, включи песню стой"),
    ("Робот, всё, хватит, спой другую", "Робот, спой другую и хватит"),
    ("Робот, стоп", "Робот, спой песню про стоп"),
    ("Робот, отставить", "Робот, как будет по-английски хватит"),
    ("Робот, помолчи", "Робот, напиши стих со словом помолчи"),
    ("Робот, стойте", "Робот, покажи анимацию «стой»"),
    ("Робот, ну хватит", "Робот, сыграй трек стоп"),
    ("Робот, стоп-стоп", "Робот, расскажи анекдот про стоп-стоп"),
    ("Робот, да стой", "Робот, спой песенку стой где стоишь"),
    ("Робот, замолчите", "Робот, переведи замолчите на немецкий"),
    ("Робот, хватит", "Робот, хватит ли нам сил"),
    ("Робот, стоп. Включи музыку", "Робот, включи музыку и стоп"),
    ("хватит робот", "спой песню про стоп робот"),
    ("Робот, эй, стоп", "Робот, поставь песню хватит это терпеть"),
]


@pytest.mark.parametrize("clear, inside", V19_PAIRS)
def test_v19_stop_word_inside_another_command_is_disputed(clear, inside):
    ours = match(clear, IDLE)
    other = match(inside, IDLE)
    assert ours is not None and not ours.disputed, clear
    assert other is not None and other.disputed, inside
    assert other.kind in ("stop_all", "silence")


def test_v19_corpus_size():
    assert len(V19_PAIRS) >= 15


def test_v19_phrases_are_in_the_reference_set():
    """Корпус В19 и эталонный набор — одни и те же фразы (требование Шифу: «те же фразы — в reference_set»)."""
    import pathlib

    import yaml

    ref = pathlib.Path(__file__).resolve().parents[3] / "scripts" / "dialog" / "reference_set.yaml"
    if not ref.exists():
        pytest.skip("reference_set.yaml вне пакета (сборка пакета без репозитория)")
    texts = {p["text"] for p in yaml.safe_load(ref.read_text(encoding="utf-8"))["phrases"]}
    inside = {other for _, other in V19_PAIRS if other.startswith("Робот,")}
    assert inside <= texts, sorted(inside - texts)


NOT_STOP = [
    "Робот, у меня стопка книг", "робот захватит флаг", "Робот, это стоит дорого", "робот работает",
    "Робот, найди песню Stop", "Робот, включи стоп-кран", "нехватка денег", "Робот, постоянно что-то",
]


@pytest.mark.parametrize("text", NOT_STOP)
def test_not_a_stop_at_all(text):
    command = match(text, IDLE)
    assert command is None or command.kind not in ("stop_all", "silence"), text


# --------------------------------------------------------------------------------------------------------------
# Громкость: намерение + канал по снимку (§4.2: музыка → музыка; речь → голос; иначе последний)
# --------------------------------------------------------------------------------------------------------------

VOLUME = [
    # (фраза, снимок, action, channel)
    ("громче", MUSIC, "louder", "music"),
    ("громче", SPEAKING, "louder", "tts"),
    ("громче", MUSIC_AND_SPEECH, "louder", "music"),
    ("громче", IDLE, "louder", "tts"),
    ("громче", LAST_MUSIC, "louder", "music"),
    ("погромче", MUSIC, "louder", "music"),
    ("погромче", SPEAKING, "louder", "tts"),
    ("тише", MUSIC, "quieter", "music"),
    ("тише", SPEAKING, "quieter", "tts"),
    ("потише", LAST_MUSIC, "quieter", "music"),
    ("потише пожалуйста", IDLE, "quieter", "tts"),
    ("играй громче", IDLE, "louder", "music"),
    ("играй громче", SPEAKING, "louder", "music"),
    ("играй трей кромче а не говори громче", SPEAKING, "louder", "music"),
    ("сделай музыку потише", SPEAKING, "quieter", "music"),
    ("сделай музыку громче", IDLE, "louder", "music"),
    ("тише музыку", IDLE, "quieter", "music"),
    ("убавь музыку", MUSIC, "quieter", "music"),
    ("прибавь звук", MUSIC, "louder", "music"),
    ("прибавь звук", SPEAKING, "louder", "tts"),
    ("на максимум", MUSIC, "max", "music"),
    ("громкость на максимум", MUSIC, "max", "music"),
    ("выкрути музыку на полную", IDLE, "max", "music"),
    ("говори громче", MUSIC, "louder", "tts"),
    ("говори потише", MUSIC, "quieter", "tts"),
    ("говори тише", IDLE, "quieter", "tts"),
    ("сделай голос громче", MUSIC, "louder", "tts"),
    ("нормальная громкость", MUSIC, "normal", "music"),
    ("обычная громкость голоса", MUSIC, "normal", "tts"),
    ("стандартная громкость", SPEAKING, "normal", "tts"),
    ("максимальная громкость", IDLE, "max", "tts"),
    ("поставь музыку погромче", IDLE, "louder", "music"),
    ("включи погромче", MUSIC, "louder", "music"),
    ("горного короля погромче", MUSIC, "louder", "music"),
    ("ну чуть-чуть потише", MUSIC, "quieter", "music"),
    ("бит погромче", IDLE, "louder", "music"),
    ("трек потише", SPEAKING, "quieter", "music"),
    ("ещё громче", MUSIC, "louder", "music"),
    ("а можно потише", SPEAKING, "quieter", "tts"),
    ("ГРОМЧЕ", MUSIC, "louder", "music"),
]


@pytest.mark.parametrize("text, snap, action, channel", VOLUME)
def test_volume_channel_by_snapshot(text, snap, action, channel):
    command = match(text, snap)
    assert command is not None and command.kind == "volume", text
    assert dict(command.args) == {"action": action, "channel": channel}, text


NOT_VOLUME = [
    ("сыграй в пещере горного короля погромче", MUSIC),
    ("расскажи анекдот погромче", IDLE),
    ("горный король погромче", IDLE),  # трек не играет — название не узнать
    ("горный король погромче", SPEAKING),
    ("не громче", MUSIC),
    ("громче или тише", MUSIC),
    ("говори нормально", IDLE),
    ("обычный голос", IDLE),
    ("сделай громко", MUSIC),
    ("громкость", MUSIC),
    ("включи бетховена погромче", MUSIC),
    ("музыку громче а голос тише", MUSIC),
    ("тишина в библиотеке", IDLE),
    ("почему ты говоришь громче обычного", SPEAKING),
    ("", IDLE),
]


@pytest.mark.parametrize("text, snap", NOT_VOLUME)
def test_not_volume(text, snap):
    assert _kind(text, snap) != "volume", text


def test_volume_channel_order():
    assert volume_channel(MUSIC_AND_SPEECH) == "music"
    assert volume_channel(SPEAKING) == "tts"
    assert volume_channel(LAST_MUSIC) == "music"
    assert volume_channel(_snap(volume_channel="bogus")) == "tts"
    assert volume_channel(MUSIC, "tts") == "tts"
    assert {ch.tool for ch in kn.VOLUME_CHANNELS.values() if ch.tool} == {"set_volume", "set_music_volume"}


# --------------------------------------------------------------------------------------------------------------
# Голос: «включи голос надёжного мужика» → set_voice по реестру (#3744: голос меняет только явная команда)
# --------------------------------------------------------------------------------------------------------------

VOICE = [
    ("Робот, включи голос надёжного мужика", VOICES, "Russian_ReliableMan"),
    ("включи голос надежного мужика", VOICES, "Russian_ReliableMan"),
    ("смени голос на мужской", VOICES, "Russian_ReliableMan"),
    ("говори голосом надёжного мужчины", VOICES, "Russian_ReliableMan"),
    ("поставь голос яркой героини", VOICES, "Russian_BrightHeroine"),
    ("давай голосом безумной королевы", VOICES, "Russian_CrazyQueen"),
    ("переключи голос на пессимистичную девушку", VOICES, "Russian_PessimisticGirl"),
    ("голос привлекательного парня", VOICES, "Russian_AttractiveGuy"),
    ("смени голос на Алёну", YANDEX, "alena"),
    ("включи голос Захара", YANDEX, "zahar"),
    ("говори голосом Антона", YANDEX, "anton"),
    ("поменяй голос на ермила", YANDEX, "ermil"),
    ("голос reliable man", VOICES, "Russian_ReliableMan"),
]


# «Верни обычный голос» (#3803): значение default разворачивает код, провайдера и голос модель не выбирает
DEFAULT_VOICE_PHRASES = [
    "Робот, верни свой обычный голос", "верни обычный голос", "верни мне мой обычный голос",
    "говори своим обычным голосом", "поставь голос по умолчанию", "вернись к обычному голосу",
    "верни свой стандартный голос",
]


@pytest.mark.parametrize("text", DEFAULT_VOICE_PHRASES)
def test_default_voice_is_resolved_by_code(text):
    assert match(text, IDLE, VOICES) == Tier1Command("set_voice", {"voice": "default"}), text


@pytest.mark.parametrize("text", ["верни обычный голос надёжного мужика", "какой у тебя обычный голос",
                                  "верни голос обратно"])
def test_default_voice_needs_nothing_else(text):
    command = match(text, IDLE, VOICES)
    assert command is None or command.args.get("voice") != "default", text


@pytest.mark.parametrize("text, voices, voice", VOICE)
def test_voice_by_registry(text, voices, voice):
    command = match(text, IDLE, voices)
    assert command == Tier1Command("set_voice", {"voice": voice}), text


NOT_VOICE = [
    ("какой у тебя голос", VOICES),
    ("смени голос на какой-нибудь весёлый", VOICES),
    ("смени голос", VOICES),
    ("включи голос Алёны", VOICES),  # у активного провайдера такого нет — решает LLM
    ("расскажи сказку разными голосами", VOICES),
    ("голос", VOICES),
    ("включи надёжного мужика", VOICES),  # без слова «голос» — не про голос
    ("смени голос на детский", VOICES),
]


@pytest.mark.parametrize("text, voices", NOT_VOICE)
def test_not_a_voice_command(text, voices):
    assert _kind(text, IDLE, voices) != "set_voice", text


# --------------------------------------------------------------------------------------------------------------
# Новая сессия (U8), снятие тишины, да/нет (U9)
# --------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("phrase", kn.NEW_SESSION_PHRASES)
def test_new_session_phrases(phrase):
    assert _kind(f"Робот, {phrase}") == "new_session"


@pytest.mark.parametrize("text", ["расскажи про новый год", "что такое сессия", "забудь", "начни песню"])
def test_not_new_session(text):
    assert _kind(text) != "new_session"


@pytest.mark.parametrize("text", ["говори", "Робот, отвечай", "включайся", "можешь говорить", "разговаривай"])
def test_unsilence_only_when_silenced(text):
    assert _kind(text, SILENCED) == "unsilence"
    assert _kind(text, IDLE) != "unsilence"


@pytest.mark.parametrize("text, value", [("да", "yes"), ("Да.", "yes"), ("ага", "yes"), ("угу", "yes"),
                                         ("ладно", "yes"), ("нет", "no"), ("не", "no")])
def test_yes_no_on_pending_ask(text, value):
    assert match(text, ASKED) == Tier1Command("answer", {"value": value})
    assert _kind(text, IDLE) != "answer"


def test_yes_no_words_are_short_answers():
    assert set(kn.YES_WORDS) | set(kn.NO_WORDS) <= set(kn.SHORT_ANSWERS)


def test_long_answer_is_not_tier1():
    assert _kind("да, но потом", ASKED) != "answer"
    assert _kind("да", _snap(pending_ask=Ask("name", "Как тебя зовут?"))) != "answer"


# --------------------------------------------------------------------------------------------------------------
# SILENCED: работает всё незвучащее (стоп, тишина, громкость, новая сессия), голос — нет (§6.3)
# --------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("text", STOP_ALL + SILENCE)
def test_stop_and_silence_work_when_silenced(text):
    assert _kind(text, SILENCED) in ("stop_all", "silence")


@pytest.mark.parametrize("text", ["громче", "тише", "сделай музыку потише", "на максимум", "нормальная громкость"])
def test_volume_works_when_silenced(text):
    assert _kind(text, SILENCED) == "volume"


@pytest.mark.parametrize("text", ["Робот, забудь всё", "новая сессия", "начни заново"])
def test_new_session_works_when_silenced(text):
    assert _kind(text, SILENCED) == "new_session"


@pytest.mark.parametrize("text", ["смени голос на мужской", "поставь голос яркой героини"])
def test_voice_change_is_not_tier1_when_silenced(text):
    assert _kind(text, SILENCED) is None


def test_silenced_kinds_are_the_quiet_ones():
    assert {kind for kind, sounds in kn.TIER1_KINDS.items() if not sounds} == {
        "stop_all", "stop_speech", "silence", "unsilence", "volume", "new_session", "answer"}


ORDINARY = [
    "расскажи анекдот", "какая погода", "спой песню", "включи музыку", "покажи анимацию сон", "который час",
    "привет", "что ты умеешь", "поставь клубный трек", "ты диджей Дайв",
    "запомни, что Саша любит зелёный чай", "поезжай на кухню", "найди в интернете рецепт борща",
    "сколько у тебя заряда", "спой песенку про стрижей и ежей", "включи что-нибудь из библиотеки",
    "сыграй что нибудь для души", "расскажи страшную историю про полицию", "как тебя зовут",
    "покажи анимацию ускорения", "сгенерируй песню про кота", "что сейчас играет",
]


@pytest.mark.parametrize("text", ORDINARY)
def test_ordinary_requests_are_not_tier1(text):
    assert match(text, IDLE, VOICES) is None, text


def test_corpus_is_at_least_200_phrases():
    corpus = (STOP_ALL + SILENCE + [t for t, _ in STOP_WITH_REST] + [t for pair in V19_PAIRS for t in pair]
              + NOT_STOP + [t for t, *_ in VOLUME] + [t for t, _ in NOT_VOLUME] + [t for t, *_ in VOICE]
              + [t for t, _ in NOT_VOICE] + list(kn.NEW_SESSION_PHRASES) + ORDINARY)
    assert len(corpus) >= 200, len(corpus)
