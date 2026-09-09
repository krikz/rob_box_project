"""test_tts_text_guard.py — Unicode-script guard for TTS (issue #1709).

Юзер сообщил: «робот что-то бормотал на хинди». В логе
``voice-assistant`` не было ни индийских, ни китайских строк — только 2
failed TTS чанка БЕЗ текста. Гипотеза (подтверждена архитектором):
LLM периодически вставляет в ``speak_text`` символы неподдерживаемых
письменностей, TTS-провайдер читает их как попало, а текст чанка
терялся в логах.

Этот файл покрывает pure-Python guard
(``rob_box_voice/tts_text_guard.py``), который решает вторую половину
acceptance issue #1709: «если чанк содержит >10% non-cyrillic+non-latin —
лог warning + пропустить».

Модуль без ROS-зависимостей, поэтому тест запускается напрямую::

    cd src/rob_box_voice && PYTHONPATH=. python3 -m pytest \\
        test/unit/core/test_tts_text_guard.py -v
"""

from __future__ import annotations

from rob_box_voice.tts_text_guard import (
    DEFAULT_FOREIGN_SCRIPT_THRESHOLD,
    analyze,
    describe,
    is_allowed_letter,
    script_of,
    should_skip,
)


# ── Разрешённые письменности не блокируются ───────────────────────────


class TestAllowedText:
    def test_pure_russian_is_clean(self) -> None:
        report = analyze("Привет, я РОББОКС! Как дела?")
        assert report.foreign_letters == 0
        assert report.ratio == 0.0
        assert report.scripts == ()
        assert should_skip("Привет, я РОББОКС! Как дела?") is False

    def test_russian_with_english_loanwords_is_clean(self) -> None:
        text = "Зачитаю рэп, брат — это настоящий hip-hop flow!"
        assert analyze(text).foreign_letters == 0
        assert should_skip(text) is False

    def test_digits_punctuation_emoji_do_not_count(self) -> None:
        # Не-буквы вообще не участвуют в расчёте: эмодзи/цифры/пунктуация
        # не должны выглядеть как «чужая письменность».
        report = analyze("Погнали 2024!!! 🎶🦝 128 bpm — ага?")
        assert report.foreign_letters == 0
        assert report.ratio == 0.0
        assert should_skip("Погнали 2024!!! 🎶🦝 128 bpm") is False

    def test_empty_and_non_string_are_clean(self) -> None:
        assert analyze("").total_letters == 0
        assert should_skip("") is False
        # Не-строка не должна ронять guard (контракт strip_markdown).
        assert should_skip(None) is False  # type: ignore[arg-type]
        assert analyze(12345).total_letters == 0  # type: ignore[arg-type]

    def test_latin_diacritics_allowed(self) -> None:
        # Транслит/имена с диакритикой — латиница, читается нормально.
        text = "Café Zürich naïve"
        assert analyze(text).foreign_letters == 0
        assert should_skip(text) is False


# ── Чужие письменности детектируются и блокируются ────────────────────


class TestForeignScripts:
    def test_pure_chinese_is_skipped(self) -> None:
        text = "你好世界"
        report = analyze(text)
        assert report.foreign_letters == 4
        assert report.ratio == 1.0
        assert "cjk" in report.scripts
        assert should_skip(text) is True

    def test_pure_devanagari_is_skipped(self) -> None:
        # Именно этот кейс юзер услышал как «бормотание на хинди».
        text = "नमस्ते दुनिया"
        report = analyze(text)
        assert report.foreign_letters > 0
        assert "devanagari" in report.scripts
        assert should_skip(text) is True

    def test_arabic_is_skipped(self) -> None:
        text = "مرحبا بالعالم"
        assert "arabic" in analyze(text).scripts
        assert should_skip(text) is True

    def test_hangul_and_kana_detected(self) -> None:
        assert "hangul" in analyze("안녕하세요").scripts
        assert "hiragana" in analyze("こんにちは").scripts
        assert "katakana" in analyze("ロボット").scripts

    def test_mixed_russian_and_hieroglyphs_over_threshold_skipped(self) -> None:
        # Это реальная форма бага: короткая русская обвязка + большой
        # хвост иероглифов, который юзер и слышал как бормотание.
        text = "Слушай: 加油加油加油加油加油加油"
        report = analyze(text)
        assert report.ratio > DEFAULT_FOREIGN_SCRIPT_THRESHOLD
        assert should_skip(text) is True

    def test_single_hieroglyph_in_long_russian_phrase_is_spoken(self) -> None:
        # Обратная сторона: одно-два вкрапления в длинной русской фразе
        # НЕ должны глушить всю реплику — иначе робот замолчит на любой
        # мелочи. Порог 10% как раз про это.
        text = (
            "Китайская идиома звучит как вай го, а пишется она "
            "вот таким единственным символом 加 в самом конце фразы"
        )
        report = analyze(text)
        assert report.foreign_letters == 1
        assert report.ratio < DEFAULT_FOREIGN_SCRIPT_THRESHOLD
        assert should_skip(text) is False

    def test_greek_letters_detected(self) -> None:
        # §7 TEXT RULES требует «α→альфа»; сплошная греческая строка —
        # чужая письменность.
        assert "greek" in analyze("αβγδεζηθ").scripts
        assert should_skip("αβγδεζηθ") is True


# ── Порог ─────────────────────────────────────────────────────────────


class TestThreshold:
    def test_default_threshold_is_ten_percent(self) -> None:
        assert DEFAULT_FOREIGN_SCRIPT_THRESHOLD == 0.10

    def test_exactly_at_threshold_is_not_skipped(self) -> None:
        # 1 чужая буква из 10 = ровно 10% → сравнение строгое (>), значит
        # НЕ блокируем. Явно фиксируем границу.
        text = "абвгдежзи" + "加"  # 9 кириллических + 1 CJK = 10 букв
        report = analyze(text)
        assert report.total_letters == 10
        assert report.foreign_letters == 1
        assert report.ratio == 0.1
        assert should_skip(text) is False

    def test_just_over_threshold_is_skipped(self) -> None:
        text = "абвгдежз" + "加"  # 8 + 1 = 9 букв → 11.1%
        assert analyze(text).ratio > 0.10
        assert should_skip(text) is True

    def test_custom_threshold_respected(self) -> None:
        text = "абвгдежз" + "加"  # 11.1%
        assert should_skip(text, threshold=0.5) is False
        assert should_skip(text, threshold=0.0) is True


# ── Хелперы ───────────────────────────────────────────────────────────


class TestHelpers:
    def test_is_allowed_letter(self) -> None:
        assert is_allowed_letter("я") is True
        assert is_allowed_letter("Z") is True
        assert is_allowed_letter("加") is False
        # Не-буквы никогда не «разрешённые буквы».
        assert is_allowed_letter(" ") is False
        assert is_allowed_letter("7") is False
        assert is_allowed_letter("🦝") is False

    def test_script_of_returns_empty_for_allowed_and_non_letters(self) -> None:
        assert script_of("ж") == ""
        assert script_of("q") == ""
        assert script_of("!") == ""
        assert script_of("5") == ""

    def test_script_of_unknown_alpha_is_other(self) -> None:
        # Буква вне известных диапазонов всё равно считается чужой —
        # guard не должен «пропускать по незнанию».
        assert script_of("ᚠ") == "other"  # Runic
        assert should_skip("ᚠᚢᚦᚨᚱᚲ") is True

    def test_describe_contains_ratio_scripts_and_sample(self) -> None:
        report = analyze("Слушай 加油加油加油加油")
        line = describe(report)
        assert "foreign_ratio=" in line
        assert "scripts=cjk" in line
        assert "sample=" in line
        assert "加" in line

    def test_sample_is_bounded(self) -> None:
        # Лог не должен пухнуть от длинного мусорного чанка.
        report = analyze("加" * 500)
        assert len(report.sample) <= 12
        assert report.foreign_letters == 500
