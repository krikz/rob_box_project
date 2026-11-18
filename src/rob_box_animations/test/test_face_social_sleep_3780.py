"""Сон, когда робота никто не теребит (#3780 C).

Решает код по фактам: кадр пуст, реплики нет, музыка не играет, прошло SOCIAL_SLEEP_AFTER_S (10 мин).
Лицо в кадре или голос — короткое пробуждение, потом обычное лицо. Проснувшись, молчит: директор речи не шлёт.
"""

from rob_box_animations.face.states import SCENES
from rob_box_animations.face_director import FaceDirector
from rob_box_dialog.knowledge import SOCIAL_SLEEP_AFTER_S

T0 = 1_800_000_000.0


def awake() -> FaceDirector:
    """Директор после стартового пробуждения, голосовой стек на месте."""
    d = FaceDirector(T0)
    d.voice_seen(T0 + 7.0)
    assert d.choose(T0 + 7.0)[0] == 'idle_stare'
    return d


PLAYING = '{"state": "playing", "track_id": "t1", "dj": {"bpm": 118, "started_at": %s}}' % (T0 + 8)


def test_ten_minutes_is_the_card_default():
    assert SOCIAL_SLEEP_AFTER_S == 10 * 60


def _quiet(now: float) -> FaceDirector:
    """Стек жив (его сон — другой, секунды), кадр пуст, реплик нет.

    ``voice_timeout_s`` больше окна карточки: иначе за 10 минут стек «умрёт» сам, и тест проверит не тот сон.
    """
    d = FaceDirector(T0, voice_timeout_s=3 * SOCIAL_SLEEP_AFTER_S)
    d.voice_seen(now)
    d._wake_until = now
    assert d.choose(now)[0] == 'idle_stare'
    d.note_someone(now)          # последнее обращение — сейчас, не старт директора
    d.note_frame(True, now)
    return d


def test_ten_minutes_of_silence_and_an_empty_frame_sleeps():
    now = T0 + 8
    d = _quiet(now)
    assert d.choose(now + SOCIAL_SLEEP_AFTER_S - 1)[0] == 'idle_stare'
    assert d.choose(now + SOCIAL_SLEEP_AFTER_S)[0] == 'sleep'


def test_no_empty_frame_is_not_nobody():
    """Камера молчит — это не «никого нет». Сна по карточке нет (стек при этом жив)."""
    now = T0 + 8
    d = FaceDirector(T0, voice_timeout_s=3 * SOCIAL_SLEEP_AFTER_S)
    d.voice_seen(now)
    d._wake_until = now
    assert d.choose(now + SOCIAL_SLEEP_AFTER_S + 1)[0] == 'idle_stare'


def test_a_face_or_a_voice_wakes_then_the_ordinary_face():
    now = T0 + 8
    d = _quiet(now)
    assert d.choose(now + SOCIAL_SLEEP_AFTER_S)[0] == 'sleep'
    d.note_frame(False, now + SOCIAL_SLEEP_AFTER_S + 1)
    assert d.choose(now + SOCIAL_SLEEP_AFTER_S + 1)[0] == 'wakeup'
    later = now + SOCIAL_SLEEP_AFTER_S + 1 + SCENES['wakeup'].period
    assert d.choose(later)[0] == 'idle_stare'

    d.note_frame(True, later)
    assert d.choose(later + SOCIAL_SLEEP_AFTER_S)[0] == 'sleep'
    d.note_someone(later + SOCIAL_SLEEP_AFTER_S + 1)
    assert d.choose(later + SOCIAL_SLEEP_AFTER_S + 1)[0] == 'wakeup'


def test_music_keeps_the_face_awake():
    now = T0 + 8
    d = _quiet(now)
    assert d.on_music_state(PLAYING, now)
    assert d.choose(now + SOCIAL_SLEEP_AFTER_S + 5)[0].startswith('music_')
