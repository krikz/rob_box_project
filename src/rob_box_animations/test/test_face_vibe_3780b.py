"""Вайб лица — длительное состояние, не сцена на 30 с (#3780 B).

Решает код по фактам. Явная просьба про аксессуар толкает шар сильно: очки видны далеко за
``FACE_SCENE_MAX_S``. «Сними» — толчок домой, аксессуар уходит сразу. Без событий шар у нейтрали,
лицо обычное. Сон и тестовый кадр выше вайба.
"""

from rob_box_animations.face.face import SHADES_FRAME
from rob_box_animations.face_director import FaceDirector
from rob_box_animations.face_vibe import STYLE_SLOTS

from rob_box_dialog.knowledge import FACE_SCENE_MAX_S, FACE_SCENE_SLOTS, VIBE_DECAY_S

T0 = 1_800_000_000.0
AWAKE = T0 + 8.0


def awake(**kw) -> FaceDirector:
    """Директор после стартового пробуждения, голосовой стек на месте."""
    d = FaceDirector(T0, voice_timeout_s=3 * VIBE_DECAY_S, **kw)
    d.voice_seen(AWAKE)
    d._wake_until = AWAKE
    assert d.choose(AWAKE)[0] == "idle_stare"
    return d


def shades_on(frame) -> bool:
    """Очки на глазу: оправа спрайта, не просто «глаз что-то нарисовал»."""
    return any((frame.eye_l[y, x] == SHADES_FRAME).all() for y in range(8) for x in range(8))


def landed(d: FaceDirector, start: float):
    """Кадр после доезда очков: первый tick начинает сцену с t=0, спрайт виден ~через 1.2 с.

    Сцена DSL крутится по своему периоду, поэтому «через 1.2 с от старта» — не то же, что «через 1.2 с от now».
    """
    d.tick(start)
    return d.tick(start + 1.2)


def worn_slots(d: FaceDirector, now: float):
    key, ctx = d.choose(now)
    return key, (dict(ctx.face) if ctx.face else {})


def test_style_table_uses_only_existing_slots():
    """Одна таблица, значения — из слотов лица. Фона и новых пикселей нет."""
    assert set(STYLE_SLOTS) <= set(FACE_SCENE_SLOTS["eyes"]) | set(FACE_SCENE_SLOTS["bg"]) | set(
        FACE_SCENE_SLOTS["fx"]
    ) | {"shades"}
    for slots in STYLE_SLOTS.values():
        assert "bg" not in slots and "fx" not in slots
        for slot, value in slots.items():
            assert value in FACE_SCENE_SLOTS[slot]


def test_ask_for_shades_holds_past_the_scene_cap():
    """«Надень очки»: аксессуар виден сразу (очки доезжают за кадр) и ещё на +120 с — далеко за 30 с сцены."""
    d = awake()
    d.apply_face({"eyes": "shades"}, 3.0, AWAKE)
    assert worn_slots(d, AWAKE) == ("face_dsl", {"eyes": "shades"})
    # Слот стоит сразу. Спрайт очков доезжает за свой кадр (~1.2 с) — это рисунок, не сцена на 30 с.
    assert shades_on(landed(d, AWAKE))
    later = AWAKE + 120.0
    assert later - AWAKE > FACE_SCENE_MAX_S
    assert worn_slots(d, later) == ("face_dsl", {"eyes": "shades"})
    # Не первый кадр петли DSL (очки в ней падают заново): слот держится, спрайт доезжает.
    assert shades_on(landed(d, later))


def test_without_events_the_vibe_fades_to_the_ordinary_face():
    """Без событий на +60 с вайб ещё есть, к времени затухания — нейтраль и обычное лицо."""
    d = awake()
    d.apply_face({"eyes": "shades"}, 3.0, AWAKE)
    assert worn_slots(d, AWAKE + 60.0)[1] == {"eyes": "shades"}
    key, slots = worn_slots(d, AWAKE + VIBE_DECAY_S)
    assert slots == {}
    assert key == "idle_stare"
    assert not shades_on(landed(d, AWAKE + VIBE_DECAY_S))
    assert d.vibe.at(AWAKE + VIBE_DECAY_S).neutral()


def test_take_off_drops_the_accessory_within_a_couple_of_frames():
    """«Сними»: толчок домой, очки уходят за пару кадров, не через десять минут."""
    d = awake()
    d.apply_face({"eyes": "shades"}, 3.0, AWAKE)
    assert shades_on(landed(d, AWAKE))
    off = AWAKE + 3.0
    d.apply_face({"eyes": "open"}, 3.0, off)
    assert worn_slots(d, off)[1] == {}
    # Смена сцены кроссфейдится ~0.12 с: через пару кадров очки уже не сцена, пиксели старой сцены догасают.
    assert d.tick(off).eye_l is not None
    faded = d.tick(off + 0.2)
    assert not shades_on(faded)
    assert worn_slots(d, off + 120.0)[1] == {}


def test_no_events_means_the_ball_stays_neutral():
    """Никаких событий — шар у нейтрали всё время, вайб не выдумывается."""
    d = awake()
    for dt in (0.0, 1.0, 30.0, 120.0, VIBE_DECAY_S):
        assert d.vibe.at(AWAKE + dt).neutral()
        assert worn_slots(d, AWAKE + dt)[1] == {}
        assert d.choose(AWAKE + dt)[0] == "idle_stare"


def test_sleep_beats_the_vibe():
    """В окне сна шар не перебивает sleep, хотя очки ещё «на нём»."""
    d = FaceDirector(T0, voice_timeout_s=10.0)
    d.voice_seen(AWAKE)
    d._wake_until = AWAKE
    d.apply_face({"eyes": "shades"}, 3.0, AWAKE)
    assert worn_slots(d, AWAKE + 1.0)[1] == {"eyes": "shades"}
    assert d.choose(AWAKE + 11.0)[0] == "sleep"
    assert not d.vibe.at(AWAKE + 11.0).neutral()


def test_test_pattern_beats_the_vibe():
    d = awake(test_pattern=True)
    d.apply_face({"eyes": "shades"}, 3.0, AWAKE)
    frame = d.tick(AWAKE + 1.0)
    assert d.key == "test_pattern"
    assert not shades_on(frame)


def test_music_and_praise_push_once_and_hold_one_style():
    """Музыка и похвала толкают шар, но повтор кадра тот же стиль не дёргает."""
    d = awake()
    snap = '{"state": "playing", "track_id": "t1", "dj": {"bpm": 90, "started_at": %s}}' % AWAKE
    assert d.on_music_state(snap, AWAKE)
    # Музыка остаётся своей сценой: вайб её не перебивает, но шар уже толкнут и стиль один.
    assert d.choose(AWAKE + 1.0)[0].startswith("music_")
    style = d.vibe.at(AWAKE + 1.0).style
    assert style != "neutral"
    d.on_music_state(snap, AWAKE + 1.04)  # тот же снимок — не новое событие
    assert d.vibe.at(AWAKE + 1.04).style == style
    d.note_praise(AWAKE + 2.0)
    d.note_praise(AWAKE + 2.04)
    assert d.vibe.at(AWAKE + 3.0).style == d.vibe.at(AWAKE + 3.5).style
    # Палитра не подменяет сцену музыки и не мигает от кадра к кадру: оттенок один и тот же.
    _, ctx_a = d.choose(AWAKE + 3.0)
    _, ctx_b = d.choose(AWAKE + 3.04)
    assert d.choose(AWAKE + 3.0)[0].startswith("music_")
    assert ctx_a.tint == ctx_b.tint and ctx_a.tint is not None
    # #5143: слот оттенка едет в контексте (лог берёт его, не пересчитывает).
    # Похвала после музыки — fire, слот warm; кадр к кадру слот тот же.
    assert ctx_a.tint_slot == ctx_b.tint_slot == "warm"


def test_same_events_give_the_same_frames():
    """Одна последовательность событий — одинаковые кадры."""

    def run():
        d = awake()
        d.apply_face({"eyes": "shades"}, 3.0, AWAKE)
        d.note_praise(AWAKE + 1.0)
        frames = [d.tick(AWAKE + i / 25.0) for i in range(0, 50, 10)]
        return [(f.eye_l.tobytes(), f.eye_r.tobytes(), f.mouth.tobytes()) for f in frames]

    assert run() == run()
