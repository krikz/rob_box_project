"""Unit tests for :mod:`rob_box_voice.core.dialogue_text`."""

from __future__ import annotations

import pytest

from rob_box_voice.core.dialogue_text import (
    DEFAULT_SILENCE_COMMANDS,
    DEFAULT_UNSILENCE_COMMANDS,
    DEFAULT_WAKE_WORDS,
    has_wake_word,
    is_silence_command,
    is_unsilence_command,
    strip_wake_word,
)


# ---------------------------------------------------------------------------
# has_wake_word
# ---------------------------------------------------------------------------


class TestHasWakeWord:
    def test_default_wake_word_detected(self) -> None:
        assert has_wake_word("привет робок", DEFAULT_WAKE_WORDS) is True

    def test_default_wake_word_case_insensitive(self) -> None:
        # Lowercase both sides — that's the contract.
        text = "Роббокс расскажи анекдот".lower()
        assert has_wake_word(text, DEFAULT_WAKE_WORDS) is True

    def test_missing_wake_word(self) -> None:
        assert has_wake_word("просто текст без триггера", DEFAULT_WAKE_WORDS) is False

    def test_empty_wake_words_bypass(self) -> None:
        # Bypass mode — accept every input
        assert has_wake_word("anything goes here", []) is True

    def test_custom_wake_words(self) -> None:
        assert has_wake_word("джарвис выключи свет", ["джарвис"]) is True
        assert has_wake_word("выключи свет", ["джарвис"]) is False

    def test_substring_does_not_trigger_bot(self) -> None:
        """Issue #1292: «бот» ∈ «работает» — ложный wake word на фоновую речь."""
        assert has_wake_word("он потом работает", ["бот"]) is False
        assert has_wake_word("работает", ["бот"]) is False
        assert has_wake_word("он потом работает", DEFAULT_WAKE_WORDS) is False

    def test_substring_words_do_not_trigger(self) -> None:
        """«работник», «заработок», «работа» содержат «бот», но не как слово."""
        assert has_wake_word("работник", ["бот"]) is False
        assert has_wake_word("заработок", ["бот"]) is False
        assert has_wake_word("работа", ["бот"]) is False
        assert has_wake_word("собака", ["бот"]) is False

    def test_word_boundary_still_matches_real_bot(self) -> None:
        """Отдельное слово «бот» — валидный wake word."""
        assert has_wake_word("бот привет", ["бот"]) is True
        assert has_wake_word("привет бот", ["бот"]) is True

    def test_deployed_config_wake_words(self) -> None:
        """Реальный список из dialogue_node.yaml (включая «бот», «робо», «роб»)."""
        deployed = [
            "робок", "робот", "роббокс", "робокос", "роббос", "робокс",
            "роберт", "рыбок", "рома", "бот", "робо", "роб",
        ]
        # Фоновая речь с «работает»/«работник» — НЕ wake word (баг #1292)
        assert has_wake_word("он потом работает", deployed) is False
        assert has_wake_word("работник пришёл", deployed) is False
        assert has_wake_word("заработок", deployed) is False
        # Прямое обращение — wake word
        assert has_wake_word("робот расскажи анекдот", deployed) is True
        assert has_wake_word("бот привет", deployed) is True
        assert has_wake_word("робо вруби музыку", deployed) is True
        assert has_wake_word("роб", deployed) is True

    def test_wake_word_in_middle_of_phrase(self) -> None:
        """Регресс-тест фикса 10.08: «робот» в середине фразы — wake word."""
        text = "денчик ой фу робот меня зовут саша".lower()
        assert has_wake_word(text, DEFAULT_WAKE_WORDS) is True

    def test_issue_1292_reported_phrase(self) -> None:
        """Точная фраза из docker logs 15.08 — фоновый шум без обращения."""
        deployed = [
            "робок", "робот", "роббокс", "робокос", "роббос", "робокс",
            "роберт", "рыбок", "рома", "бот", "робо", "роб",
        ]
        phrase = (
            "в кустах каких нибудь полегало ну так и что что он потом работает"
        )
        assert has_wake_word(phrase, deployed) is False

    def test_case_insensitive_pattern(self) -> None:
        """Паттерн с re.IGNORECASE — переживает не-нижний регистр."""
        assert has_wake_word("РОБОТ расскажи", DEFAULT_WAKE_WORDS) is True
        assert has_wake_word("Он потом РАБОТАЕТ", ["бот"]) is False


# ---------------------------------------------------------------------------
# strip_wake_word
# ---------------------------------------------------------------------------


class TestStripWakeWord:
    def test_strips_default_robok(self) -> None:
        assert strip_wake_word("робок привет как дела") == "привет как дела"

    def test_strips_default_robot_case_insensitive(self) -> None:
        assert strip_wake_word("РОБОТ расскажи анекдот") == "расскажи анекдот"

    def test_strips_with_trailing_punctuation(self) -> None:
        assert strip_wake_word("роббокс, погода какая?") == "погода какая?"

    def test_strips_with_space_in_wake_word(self) -> None:
        # "роб бокс" is one of the supported spellings
        assert strip_wake_word("роб бокс расскажи анекдот") == "расскажи анекдот"

    def test_no_wake_word_returns_input(self) -> None:
        assert strip_wake_word("просто текст") == "просто текст"

    def test_only_wake_word(self) -> None:
        # Single word "роббокс" → empty string after strip
        assert strip_wake_word("роббокс") == ""

    def test_empty_wake_words_returns_stripped_input(self) -> None:
        assert strip_wake_word("  hello  ", []) == "hello"


# ---------------------------------------------------------------------------
# is_silence_command
# ---------------------------------------------------------------------------


class TestIsSilenceCommand:
    @pytest.mark.parametrize(
        "text",
        [
            "помолчи",
            "робок помолчи пожалуйста",
            "замолчи уже",
            "хватит разговаривать",
        ],
    )
    def test_default_silence_detected(self, text: str) -> None:
        assert is_silence_command(text, DEFAULT_SILENCE_COMMANDS) is True

    def test_no_silence_command(self) -> None:
        assert is_silence_command("расскажи анекдот", DEFAULT_SILENCE_COMMANDS) is False

    def test_custom_commands(self) -> None:
        assert is_silence_command("тишина!", ["тиш"]) is True
        assert is_silence_command("помолчи", ["тиш"]) is False


# ---------------------------------------------------------------------------
# is_unsilence_command
# ---------------------------------------------------------------------------


class TestIsUnsilenceCommand:
    @pytest.mark.parametrize(
        "text",
        [
            "говори уже",
            "включись",
            "давай работай",
            "давай отвечай мне",
            "разговаривай со мной",
        ],
    )
    def test_default_unsilence_detected(self, text: str) -> None:
        # Defaults are prefixes — "отвечай" contains "отвеч".
        assert is_unsilence_command(text, DEFAULT_UNSILENCE_COMMANDS) is True

    def test_no_unsilence_command(self) -> None:
        assert is_unsilence_command("расскажи анекдот", DEFAULT_UNSILENCE_COMMANDS) is False

    def test_custom_unsilence(self) -> None:
        assert is_unsilence_command("voice on", ["voice on"]) is True
        assert is_unsilence_command("говори", ["voice on"]) is False


# ---------------------------------------------------------------------------
# Smoke test — guard against regressions when DEFAULT_* tuples change
# ---------------------------------------------------------------------------


def test_default_tuples_are_non_empty() -> None:
    """Triggers / wake words must not be empty (silent break)."""
    assert len(DEFAULT_WAKE_WORDS) > 0
    assert len(DEFAULT_SILENCE_COMMANDS) > 0
    assert len(DEFAULT_UNSILENCE_COMMANDS) > 0

# ---------------------------------------------------------------------------
# «Робота» — винительный падеж от STT (прогон 35734532425, шаг n209)
# ---------------------------------------------------------------------------


class TestAccusativeRobota:
    """Регрессия: «Робота, про меня что помнишь?» обязана будить робота.

    Живой прогон 35734532425 (акт 2 «Знакомство»): vosk трижды подряд
    распознал обращение «Робот,» как «Робота,», has_wake_word вернул False
    (``\b``-матчинг не видит «робот» внутри «робота»), робот промолчал, шаг
    упал с ``FAIL no_accept``. Диктор при этом был опознан верно — потерян
    был именно вейк, а не голос.
    """

    def test_accusative_wakes(self) -> None:
        assert has_wake_word("робота, про меня что помнишь?", DEFAULT_WAKE_WORDS) is True

    def test_accusative_stripped_cleanly(self) -> None:
        assert (
            strip_wake_word("Робота, про меня что помнишь?", DEFAULT_WAKE_WORDS)
            == "про меня что помнишь?"
        )

    def test_nominative_still_wakes(self) -> None:
        assert has_wake_word("робот, привет", DEFAULT_WAKE_WORDS) is True

    def test_rabotaet_still_not_a_wake_word(self) -> None:
        """Гард #1292 не должен пострадать: «работает» — не вейк."""
        assert has_wake_word("он работает уже час", DEFAULT_WAKE_WORDS) is False


# ---------------------------------------------------------------------------
# Синхронность кодового списка и docker/vision/config/wake_words.yaml
# ---------------------------------------------------------------------------


def test_wake_words_yaml_matches_code_list() -> None:
    """YAML на роботе и DEFAULT_WAKE_WORDS — один список, а не два.

    Инвариант заявлен в шапке ``wake_words.yaml`` («байт-в-байт копия»), но
    до сих пор держался только комментарием: автотеста не было, и правка в
    одном месте молча расходилась со вторым. На роботе читается ИМЕННО
    YAML (bind-mount), кодовый список — фолбек для dev-env и тестов, так
    что расхождение означает «в тестах зелено, на роботе глухо».
    """
    from pathlib import Path

    import yaml

    repo_root = Path(__file__).resolve().parents[5]
    cfg = repo_root / "docker" / "vision" / "config" / "wake_words.yaml"
    assert cfg.exists(), f"конфиг вейк-слов не найден: {cfg}"

    personality = yaml.safe_load(cfg.read_text(encoding="utf-8"))["personality"]
    assert list(personality) == list(DEFAULT_WAKE_WORDS), (
        "personality из wake_words.yaml разошёлся с DEFAULT_WAKE_WORDS — "
        "правь ОБА места в одном коммите: на роботе действует YAML, "
        "в юнит-тестах кодовый список"
    )
