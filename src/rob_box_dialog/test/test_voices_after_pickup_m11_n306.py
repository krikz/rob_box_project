"""М11 n306/n307 (#4236, 05.11 01:45:56): «Трое было… ты и Борис… третий без имени», а говорили четверо - Саша
(собеседник), Борис, безымянный «Тихий» и курьер (новый профиль 01:45:02).

Причина: в n305b голое «робот» подхватило фразу курьера (``Backlog.pickup``), а подхват удалял её из бэклога -
сводка голосов окна (``Backlog.background_summary``) курьера уже не видела. Решение (ADR-0148): подхваченное из
бэклога уходит (второй раз не подхватывается, в ``background_since_reply`` не идёт), но голос его - голос окна; число и
состав голосов строит код и отдаёт модели готовыми (``background_summary``), сколько фраз окна ушло подхватом -
``picked_up``.
"""

from rob_box_dialog.address import BACKLOG_PICKUP, decide
from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.knowledge import BACKGROUND_WINDOW_S, BACKLOG_TTL_S, DEFAULT_WAKE_WORDS
from rob_box_dialog.speaker_adapter import voice_tag

SASHA = {"is_known": True, "speaker_id": "spk-sasha", "name": "Саша", "confidence": 0.9}
BORIS = {"is_known": True, "speaker_id": "spk-boris", "name": "Борис", "confidence": 0.9}
QUIET = {"is_known": True, "speaker_id": "spk-quiet", "epithet": "Тихий", "confidence": 0.8}
COURIER = {"is_known": True, "speaker_id": "spk-courier", "epithet": "Сосед", "confidence": 0.8}
WAKE = list(DEFAULT_WAKE_WORDS)


def _say(backlog, uid, t, text, payload):
    backlog.add(Heard(text=text, t=t, voice_s=2.0, utterance_id=uid))
    if payload is not None:
        backlog.note_voice(uid, voice_tag(payload))


def _workshop():
    """Акт n305: Саша, Борис и «Тихий» говорят на фоне, последним - курьер."""
    backlog = Backlog()
    _say(backlog, "u-sasha", 2.0, "Борис, подай отвёртку", SASHA)
    _say(backlog, "u-boris", 5.0, "держи", BORIS)
    _say(backlog, "u-quiet", 8.0, "я тут посижу тихонько", QUIET)
    _say(backlog, "u-courier", 30.0, "посылку куда поставить", COURIER)
    return backlog


def _ask_who(backlog, t=60.0):
    return decide(Heard(text="Робот, кто сейчас говорил?", t=t, voice_s=2.0, person_ref="spk-sasha"), backlog, WAKE)


def test_n306_courier_picked_up_is_still_a_voice_of_the_window():
    """Живой n305b→n306: голое «робот» подхватило фразу курьера, потом Саша спросил «кто говорил». Голосов четверо:
    Саша (собеседник), Борис и двое безымянных («Тихий» и курьер); одна фраза окна ушла подхватом."""
    backlog = _workshop()
    picked = decide(Heard(text="робот", t=33.0), backlog, WAKE)
    assert picked.source == BACKLOG_PICKUP and picked.text == "посылку куда поставить"
    asked = _ask_who(backlog)
    assert asked.background_summary == {"voices": 4, "caller_included": True, "known": ["Борис"], "unknown": 2,
                                        "picked_up": 1}
    assert [p["text"] for p in asked.background] == [
        "Борис, подай отвёртку", "держи", "я тут посижу тихонько"], "подхваченное в фон второй раз не идёт"


def test_n307_count_of_voices_is_the_same_with_and_without_the_pickup():
    """Число голосов окна - код, и подхват его не меняет: тот же фон без подхвата даёт те же 4 голоса."""
    without = _ask_who(_workshop()).background_summary
    backlog = _workshop()
    decide(Heard(text="робот", t=33.0), backlog, WAKE)
    with_pickup = _ask_who(backlog).background_summary
    assert without["voices"] == with_pickup["voices"] == 4
    assert {k: v for k, v in with_pickup.items() if k != "picked_up"} == without


def test_voice_result_after_the_pickup_still_labels_the_picked_phrase():
    """Биометрия фразы курьера пришла уже после подхвата (новый профиль заводится позже STT): голос дописывается и
    подхваченной фразе - иначе она шла бы в сводку как ``unattributed``, а не как голос."""
    backlog = Backlog()
    _say(backlog, "u-boris", 5.0, "держи", BORIS)
    backlog.add(Heard(text="посылку куда поставить", t=30.0, voice_s=2.0, utterance_id="u-courier"))
    assert [e.text for e in backlog.pickup(Heard(text="робот", t=33.0))] == ["посылку куда поставить"]
    backlog.note_voice("u-courier", voice_tag(COURIER))
    assert _ask_who(backlog).background_summary == {"voices": 2, "caller_included": False, "known": ["Борис"],
                                                    "unknown": 1, "picked_up": 1}


def test_picked_phrase_is_not_picked_twice_and_leaves_the_summary_with_the_window():
    backlog = _workshop()
    decide(Heard(text="робот", t=33.0), backlog, WAKE)
    assert backlog.pickup(Heard(text="робот", t=34.0)) == [], "подхваченное второй раз не подхватывается"
    late = backlog.background_summary(Heard(text="робот кто говорил", t=30.0 + BACKGROUND_WINDOW_S + 1.0))
    assert "picked_up" not in late, "старше окна - не голос окна"
    backlog.background_summary(Heard(text="робот", t=30.0 + BACKLOG_TTL_S + 1.0))
    assert backlog._picked == [], "по TTL уходит, как и бэклог"


def test_clear_forgets_picked_phrases():
    backlog = _workshop()
    decide(Heard(text="робот", t=33.0), backlog, WAKE)
    backlog.clear()
    assert _ask_who(backlog).background_summary == {}
