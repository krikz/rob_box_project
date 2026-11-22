"""#5092: тема — не готовый текст, отказ называет условие, строка успеха — из результата.

Акт 7 марафона №3: ``generate_music`` сказал «выполнен успешно», через 14 мс студия отвергла тему
«енота-механика» как короткий текст, песни нет. Фразы здесь синтетические, не чьи-то тексты.
"""

from rob_box_mcp_tools.engine import song_order
from rob_box_mcp_tools.engine.gen import song as sg

THEME = "енота-механика"
GIVEN = "ветер тихо дует над рекою\n" * 6  # 6 строк, длиннее MIN_CHARS


def test_theme_without_verbatim_phrase_is_not_given_lyrics():
    """Тема без «этими словами» — тема, не готовый текст: LLM пишет слова, check_song тему не судит."""
    order = song_order.song_request(
        ("track", THEME, "calm", "rock", "t1", "сгенерируй песню своим голосом про енота-механика"), "rock"
    )
    assert order.theme == THEME and order.lyrics == ""
    assert not sg.is_given_lyrics(THEME)


def test_words_after_verbatim_phrase_stay_given_lyrics():
    """Регресс #4973: слова после «этими словами» поются как есть, даже если text — сама просьба."""
    heard = "сгенерируй трек с этими словами " + GIVEN
    order = song_order.song_request(("track", "сгенерируй трек с этими словами", "calm", "rock", "t2", heard), "rock")
    assert order.lyrics == GIVEN and order.theme == GIVEN


def test_text_long_enough_is_given_lyrics_and_short_heard_text_is_not():
    """Явный text длиннее порога — готовые слова (ход 17:23); короткий heard_text целиком — тема, не слова."""
    long = song_order.song_request(("track", GIVEN, None, "rock", "t3", "а где слова"), "rock")
    assert long.lyrics == GIVEN
    short = song_order.song_request(("track", "", None, "rock", "t4", "про батарейку"), "rock")
    assert short.theme == "про батарейку" and short.lyrics == ""


def test_check_song_names_which_bound_failed():
    """Два отказа — два сообщения: строк мало и символов мало не сваливаются в одну фразу."""
    long_line = "ветер тихо дует над рекою " * 8  # одна строка, символов хватает
    assert "строк мало" in sg.check_song("Песня", long_line, "ru")
    assert "символов мало" not in sg.check_song("Песня", long_line, "ru")
    six_short = "\n".join(["а"] * 6)  # строк хватает, символов нет
    assert "символов мало" in sg.check_song("Песня", six_short, "ru")
    assert "строк мало" not in sg.check_song("Песня", six_short, "ru")


def test_pending_song_order_is_not_reported_done():
    """Фоновый заказ песни не даёт «выполнен успешно»: строку строит код из результата."""
    line = song_order.tool_outcome_line("generate_music", True, {"ok": True, "pending": "song"}, None)
    assert "принят, готовится: song" in line and "выполнен успешно" not in line
    done = song_order.tool_outcome_line("stop_music", True, {"ok": True}, None)
    assert done == "✅ Инструмент stop_music выполнен успешно"
    failed = song_order.tool_outcome_line("generate_music", False, None, "нет ключа")
    assert "завершился с ошибкой" in failed and "нет ключа" in failed
    # уровень — из результата, не из текста строки: провал идёт WARNING, как раньше
    assert song_order.tool_outcome("generate_music", False, None, "нет ключа")[0] == "warning"
    assert song_order.tool_outcome("generate_music", True, {"pending": "song"}, None)[0] == "info"


def test_rejected_llm_lyrics_are_logged_before_the_raise(caplog):
    """Отказ писателя тоже оставляет текст в WARN: ``request.lyrics`` на этом пути пуст (#5092)."""
    import logging

    def ask(_system, _user, _schema):
        return {"title": "Енот", "lyrics": THEME}

    with caplog.at_level(logging.WARNING, logger="rob_box_mcp_tools.engine.gen.song"):
        try:
            sg.write_lyrics(ask, sg.SongRequest(THEME, "rock", "", ""), "ru", "")
        except sg.SongFailed as exc:
            assert exc.reason == "lyrics" and "строк мало" in exc.detail
        else:
            raise AssertionError("короткий ответ LLM должен быть отвергнут")
    assert any("отклонённый текст" in r.message and THEME in r.message for r in caplog.records)


def test_rejected_lyrics_are_logged_at_warning(caplog, monkeypatch):
    """Отклонённый текст песни виден в WARN-логе: причина следующего отказа не гадается.

    ``_run`` — тот же путь, что у фонового потока ``launch``: он ловит ``SongFailed`` и пишет лог.
    Пресет подменён: без него отказ случился бы раньше проверки текста."""
    import logging

    import rob_box_dialog_host.tts_voice_registry as reg

    monkeypatch.setattr(reg, "sing_preset_for", lambda *_a, **_k: type("P", (), {"language": "ru"})())
    studio = sg.SongStudio(
        provider=object(),
        library=lambda: object(),
        lyrics=lambda *_: {},
        voice=lambda: ("minimax", "v"),
        play=lambda _t: None,
        speak=lambda _s: None,
        log=lambda: logging.getLogger("song5092"),
    )
    with caplog.at_level(logging.WARNING, logger="song5092"):
        studio._run(sg.SongRequest(THEME, "rock", "", THEME), lambda why: why, lambda _t: None)
    assert any("отклонённый текст" in r.message and THEME in r.message for r in caplog.records)
