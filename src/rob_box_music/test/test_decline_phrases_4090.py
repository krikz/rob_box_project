"""#4090 G: две фразы кода пути музыки (ADR-0148: фразу об исходе строит код из фактов пути, а не из заготовки).

* Отказ при запасном голосе (``song_block=silero_voice``): «…запасным голосом петь пока не умею. Могу спеть сам вживую
  под синт.» - два предложения противоречили друг другу. Факт такой: запасной голос (Silero) песню для записи
  (генерации) не поёт, а вживую робот читает под синт и им же (``perform`` идёт через TTS любого провайдера). Фраза
  говорит ровно это.
* ``LIVE_TOOL_TEXT`` - просьба спеть вживую дошла до тула в агенте без формы пения (чат Telegram, оператор, v1): человек
  просил спеть, а робот ничего не пел и ничего не предлагал. Теперь фраза называет, где робот поёт (вслух, у него) и что
  сказать.
"""

import pytest

from rob_box_music.choice import DeterministicChooser
from rob_box_music.source import (DECLINE, LIVE, LIVE_OFFER, LIVE_OFFER_BY_REASON, LIVE_TOOL_TEXT, REASON_WORDS,
                                  SourceContext, choose_route, decline_text, fallback_note, source_ask)

DET = DeterministicChooser()
SILERO = SourceContext(gen_ready=True, song_block="silero_voice")


def test_silero_decline_does_not_say_cannot_sing_and_can_sing():
    """Просьба спеть записанную песню при запасном голосе: причина - запись, предложение - то же вживую, этим голосом."""
    route = choose_route(source_ask("Робот, сгенерируй песню про енота своим голосом"), SILERO, DET)
    assert route.kind == DECLINE and route.gen_reason == "silero_voice"
    text = decline_text(route)
    assert text == ("Сгенерировать песню своим голосом сейчас не могу: запасной голос для записи песен пока не годится. "
                    "Могу спеть сам вживую под синт — этим же голосом, без записи.")
    assert "петь пока не умею" not in text
    assert text.count("не могу") == 1 and text.endswith(LIVE_OFFER_BY_REASON["silero_voice"])


def test_other_reasons_keep_the_general_live_offer():
    for ctx in (SourceContext(), SourceContext(gen_ready=True, has_key=False), SourceContext(gen_ready=True, balance=0.5)):
        route = choose_route(source_ask("сгенерируй песню про енота"), ctx, DET)
        assert route.kind == DECLINE and route.gen_reason not in LIVE_OFFER_BY_REASON
        assert decline_text(route).endswith(f" {LIVE_OFFER}"), route.gen_reason


def test_instrumental_decline_offers_own_styles_not_singing():
    """Просили трек без пения - предложение не про пение (вилка Шифу 27.10: другая клетка - свои стили синтеза)."""
    route = choose_route(source_ask("сгенерируй метал"), SourceContext(), DET)
    assert LIVE_OFFER not in decline_text(route) and "Могу сыграть своё" in decline_text(route)


def test_reason_offer_table_names_reasons_that_have_words():
    """Предложение по причине - только для причин, у которых есть слова отказа; запись таблицы не висит в воздухе."""
    assert LIVE_OFFER_BY_REASON and set(LIVE_OFFER_BY_REASON) <= set(REASON_WORDS)
    for reason, offer in LIVE_OFFER_BY_REASON.items():
        assert offer.startswith("Могу спеть сам вживую под синт"), reason


def test_silero_reason_words_read_in_every_phrase_that_uses_them():
    """Те же слова причины стоят в фразе-запасе («играю своё») и в отказе: ни «петь не умею», ни обрыва мысли."""
    words = REASON_WORDS["silero_voice"]
    assert "петь" not in words and "песен" in words
    assert fallback_note("silero_voice") == f"Сгенерировать сейчас не могу: {words}, играю своё."


# -- LIVE_TOOL_TEXT ------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("heard", ["спой под синт про енота", "Робот, спой вживую", "спой на Ренардо"])
def test_live_request_in_an_agent_without_sing_form_says_where_the_robot_sings(heard):
    """Тул получил «спой вживую»: путь ``live`` кодом, фраза - честно, где поёт робот и что сказать; генерации нет."""
    route = choose_route(source_ask(heard), SourceContext(gen_ready=True, balance=900.0, day_left=120.0), DET)
    assert route.kind == LIVE
    text = decline_text(route)
    assert text == LIVE_TOOL_TEXT
    assert "отсюда не выйдет" in text, "здесь он не поёт - и не обещает"
    assert "вслух" in text and "у робота" in text and "«Робот, спой под синт»" in text, "где и что сказать"
    assert "Генерацию для этого не запускаю" in text, "платная генерация не стартовала"
