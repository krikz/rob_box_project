"""М12 (05.11, деплой 41967820283, develop b0acaa6de; Refs #4236): красные шаги, где ошибся код.

* n311 («Робот, а кто именно просил включить музыку?», run 41972397405, 05:17:39 UTC): модель ответила по делу -
  «Сет запустил женский голос… А Борис минуту назад просил хоть какую-то музыку, раз тишина давит на уши.», - а
  проверка ложной «тишины» под играющей музыкой (``turn.stop_claim_false``, #4402) приняла пересказ «раз тишина
  давит» за утверждение о плеере: два повтора, затем фраза кода сводкой сета, ход ``status=done``. Теперь «тишина»
  утверждает, только начиная часть предложения («Тишина.», «Всё, тишина.»).
* n401 («Робот, какими голосами ты умеешь говорить? Перечисли, что у тебя есть.», run 41973431869, 05:22:14 UTC):
  модель перечислила голоса по памяти («надёжный мужчина… русоволосый друг детства. Есть ещё английские и пара
  служебных») без ``list_tts_voices``. Тот же класс, что звуки М11 n109: вопрос о каталоге голосов без данных
  каталога - нарушение ``voice_catalog``; повтора нет - каталог читает код сам и говорит по его данным.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import catalogs_asked
from rob_box_dialog.turn import TurnEngine, _Turn, catalog_owed, owed_reason, stop_claim_false
from rob_box_dialog.words import music_stop_claimed

from test_music_stop_honest_m11 import PLAYING, Ports as MusicPorts, _run as _run_music, _say
from test_sound_answer_after_query_n109 import Ports, _answer

# --- n311: пересказ «раз тишина давит» - не утверждение о плеере ----------------------------------------------------

N311_STT = "робот а кто именно просил включить музыку"  # STT М12 n311, 05:17:36
#: ответы модели М12 n311 (LLM RESPONSE 05:17:39 и повтор 05:17:40), музыка играла (сет «Deep Space Nine»)
N311_LLM = ("Сет запустил женский голос, я его узнал, но имени не знаю. А Борис минуту назад просил хоть какую-то "
            "музыку, раз тишина давит на уши.")
N311_LLM_RETRY = ("Сет заказал женский голос, я его узнал, но имени не знаю. А Борис перед этим ворчал, что тишина "
                  "давит на уши.")


@pytest.mark.parametrize("text", [N311_LLM, N311_LLM_RETRY, "Борис ворчал, мол, в мастерской тишина давит."])
def test_n311_silence_retold_mid_sentence_is_not_a_stop_claim(text):
    assert not music_stop_claimed(text), text


@pytest.mark.parametrize("text", ["Тишина.", "Всё, тишина.", "Ну вот и тишина.", "Тишина, Борису больше не мешаем.",
                                  "Готово - тишина.", "Музыку выключил, теперь тишина."])
def test_silence_heading_a_clause_is_still_a_stop_claim(text):
    assert music_stop_claimed(text), text


def test_n311_retold_silence_under_music_is_not_a_violation():
    turn = _Turn("t1", 0, N311_STT, VOICE, deadline=None)
    assert not stop_claim_false(PLAYING, turn, Command(speech=N311_LLM))
    assert owed_reason(PLAYING, turn, Command(speech=N311_LLM)) == ""


def test_n311_answer_about_who_asked_is_spoken_as_is_under_music():
    """Живой n311: ответ модели звучит с первого раза - без повтора, без сводки сета вместо ответа."""
    ports = MusicPorts([_say(N311_LLM)])
    agg = _run_music(ports, Addressed(N311_STT, VOICE))
    assert ports.said == [N311_LLM]
    assert ports.asked == 1 and ports.executed == []
    assert (agg.stage, agg.status) == ("done", "finished")


# --- n401: голоса по памяти - нарушение voice_catalog ---------------------------------------------------------------

N401 = "робот какими голосами ты умеешь говорить перечисли что у тебя есть"  # STT М12 n401, 05:22:14
#: ответ модели М12 n401 без list_tts_voices (LLM RESPONSE 05:22:16)
N401_LLM = ("Из голосов у меня сейчас русские: надёжный мужчина, обаятельный парень, дерзкий парень, яркая героиня, "
            "амбициозная женщина, безумная королева, пессимистка и русоволосый друг детства. Есть ещё английские и "
            "пара служебных.")
VOICES = ToolCall(kn.VOICE_CATALOG_TOOL, {})
VOICES_DATA = {"providers": [{"provider": "minimax", "label": "MiniMax", "voices_count": 15},
                             {"provider": "yandex", "label": "Яндекс", "voices_count": 13}]}


def _run(ports, text=N401):
    return asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))


def _reads(ports):
    return [call for batch in ports.executed for call in batch if call[0] == kn.VOICE_CATALOG_TOOL]


@pytest.mark.parametrize("text, expected", [
    (N401, ("voices",)),
    ("Робот, какие голоса ты умеешь говорить?", ("voices",)),
    ("Робот, сколько голосов у тебя, какими можешь разговаривать?", ("voices",)),
    # не каталог голосов: текущий голос (n402), люди по голосу (n211, n307), смена голоса (n403)
    ("робот а каким голосом ты говоришь прямо сейчас", ()),
    ("робот перечисли всех кого ты сегодня запомнил по голосу", ()),
    ("робот а сколько всего человек сейчас было в комнате посчитай по голосам", ()),
    ("робот говори голосом алены", ()),
    ("Робот, какие звуки ты умеешь?", ("sounds",)),
])
def test_catalogs_asked(text, expected):
    assert catalogs_asked(text) == expected, text


def test_n401_voices_from_memory_are_a_voice_catalog_violation():
    snap = Session("personality").snapshot
    turn = _Turn("t1", 0, N401, VOICE, deadline=None)
    assert catalog_owed(snap, turn, Command(speech=N401_LLM)) == kn.VIOLATION_VOICE_CATALOG
    assert owed_reason(snap, turn, Command(speech=N401_LLM)) == kn.VIOLATION_VOICE_CATALOG
    assert catalog_owed(snap, turn, Command(actions=(VOICES,))) == ""


def test_n401_list_from_memory_is_returned_and_the_answer_comes_from_the_catalog():
    """Живой n401: голоса словами без запроса. Список по памяти не звучит - модели нарушение с подсказкой «положи
    list_tts_voices»; она читает каталог и отвечает по данным."""
    ports = Ports([_answer(speech=N401_LLM), _answer(actions=(VOICES,)),
                   _answer(speech="Голосов у меня двадцать восемь: пятнадцать у MiniMax и тринадцать у Яндекса.")],
                  {kn.VOICE_CATALOG_TOOL: ("done", VOICES_DATA)})
    _run(ports)
    assert ports.said == ["Голосов у меня двадцать восемь: пятнадцать у MiniMax и тринадцать у Яндекса."]
    hint = ports.asked[1][-1]["content"]
    assert kn.VIOLATION_VOICE_CATALOG in hint and kn.VOICE_CATALOG_HINT in hint
    assert _reads(ports) == [(kn.VOICE_CATALOG_TOOL, {})]


def test_n401_model_insists_on_memory_the_code_reads_the_catalog_and_speaks():
    """Модель и после повторов перечисляет по памяти: её список не звучит - код сам читает ``list_tts_voices`` и
    говорит сводку по данным каталога."""
    ports = Ports([_answer(speech=N401_LLM)] * (kn.COMMAND_RETRIES_MAX + 1),
                  {kn.VOICE_CATALOG_TOOL: ("done", VOICES_DATA)})
    _run(ports)
    assert N401_LLM not in ports.said
    assert ports.said == [phrases.text("query.voices_summary", summary="MiniMax — 15, Яндекс — 13")]
    assert _reads(ports) == [(kn.VOICE_CATALOG_TOOL, {})]
