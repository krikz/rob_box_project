"""test_quest_llm_formalize.py — Unit-тесты режима quest_llm_formalize (AV-28).

Закрывает дочернюю карточку декомпозиции AV-28 (issue #1920 / PR #1931 →
PR #XXXX): ``voice_input_mode=quest_llm_formalize`` в ``_on_quest_stt``
должен НЕ звать дословный TTS, а планировать ``_formalize_with_llm`` через
``asyncio.run_coroutine_threadsafe(self._formalize_with_llm(...),
self._loop)``. Если планирование упало — fallback на ``_speak_direct``
(как в P7-simple / quest_ttts). Также покрывает резолверы
``_resolve_voice_preset`` / ``_resolve_voice_language`` и
``_get_formalize_timeout``.

Не требует ROS2 — rclpy/rcl_interfaces замоканы в conftest.py. Тесты
изолируют формализатор от LLM: подсовываем фейковый yaml в /tmp и
мокаем ``asyncio.run_coroutine_threadsafe``, чтобы убедиться, что код
формирует корректные ``LLMMessage`` (system+user) и использует tools=[].
"""

from __future__ import annotations

import asyncio
import os
import sys
import tempfile
from pathlib import Path
from unittest.mock import MagicMock

import pytest
import yaml

# rob_box_voice.dialogue_node → rob_box_harness.core.agent_core →
# rob_box_core.tool_catalog.ToolCatalogEntry: тот использует
# ``MappingProxyType({})`` как dataclass default — в Python 3.11+ это
# raise'ит ``ValueError: mutable default <class 'mappingproxy'>``,
# потому что 3.11 строже проверяет defaults на frozen dataclass.
# CI гоняет Python 3.10, где dataclass warnings soft. Локально на 3.11
# тесты скипаются целиком (см. test_issue_1777_time_format.py — это
# устоявшийся паттерн в репо).
_IS_PY_311_PLUS = sys.version_info >= (3, 11)
_SKIP_REASON = (
    "Pre-existing dataclass mutable-default bug in rob_box_core.tool_catalog; "
    "CI runs Python 3.10 where this is a soft warning, not a raise. "
    "Skip locally; the CI will validate these tests."
)

try:
    from rob_box_voice.dialogue_node import DialogueNode

    _DIALOGUE_NODE_IMPORT_OK = True
except ImportError:
    DialogueNode = None  # type: ignore[assignment]
    _DIALOGUE_NODE_IMPORT_OK = False
except Exception:  # noqa: BLE001 — import может падать по-разному
    DialogueNode = None  # type: ignore[assignment]
    _DIALOGUE_NODE_IMPORT_OK = False


# ─────────────────────────────────────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────────────────────────────────────


class _Param:
    def __init__(self, value):
        self.value = value


def _make_node(*, voice_preset: str = "",
               voice_output_language: str = "",
               voice_presets_file: str = "",
               voice_formalize_timeout_sec: float = 0.0) -> DialogueNode:
    """DialogueNode через object.__new__ + ручные атрибуты.

    Минимум, чтобы пройти _on_quest_stt + _resolve_voice_* + _get_formalize_timeout.
    """
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger

    # _resolve_voice_preset/language читают три параметра; всё через .value.
    n._voice_preset = voice_preset
    n._voice_output_language = voice_output_language
    n._voice_presets_file = voice_presets_file
    n._voice_formalize_timeout_sec = voice_formalize_timeout_sec
    n._llm_timeout_sec = 12.0  # дефолт, чтобы _get_formalize_timeout вернул 12s

    n._voice_presets_cache = None
    n._voice_presets_cache_path = None

    # _on_quest_stt в formalize-режиме использует asyncio.run_coroutine_threadsafe
    # — замокаем и _loop, и саму функцию (тесты явно проверяют вызов).
    n._loop = MagicMock(name="event_loop")

    # Дефолтные зависимости на тот случай, если _formalize_with_llm стартует
    # (внутри try/except всё ловится, но get_logger уже есть).
    n._speak_direct = MagicMock()
    n._llm = MagicMock()
    return n


def _install_param_getter(n: DialogueNode) -> None:
    """self.get_parameter(name) → _Param(value) с маршрутизацией по имени.

    Читаем атрибуты ноды ЛЕНИВО, на каждый вызов. Снимок, снятый здесь,
    был бы неверен: тесты присваивают ``n._voice_presets_file`` уже
    после установки геттера, и ``_resolve_voice_presets_path`` получал
    пустую строку → уходил на репозиторный config/voice_presets.yaml
    вместо временного файла теста.
    """
    attr_of = {
        "voice_preset": "_voice_preset",
        "voice_output_language": "_voice_output_language",
        "voice_presets_file": "_voice_presets_file",
        "voice_formalize_timeout_sec": "_voice_formalize_timeout_sec",
        "llm_timeout_sec": "_llm_timeout_sec",
    }
    defaults = {"llm_timeout_sec": 12.0}

    def _get(name):
        attr = attr_of.get(name)
        if attr is None:
            return _Param(None)
        return _Param(getattr(n, attr, defaults.get(name)))

    n.get_parameter = _get


def _make_stt_msg(data: str):
    msg = MagicMock()
    msg.data = data
    return msg


def _write_minimal_yaml(tmpdir: Path, *, with_prompts: bool = True) -> str:
    """Записать минимальный, но валидный voice_presets.yaml + 1 prompt_file.

    Используем двуязычный (RU+EN) пресет, чтобы покрыть контракт:
    один prompt_text, а не два отдельных файла. _load_presets читает
    ОДИН файл (prompt_file) целиком — а RU/EN-секции разруливаются уже
    внутри файла. Тестам этого достаточно.
    """
    prompt_dir = tmpdir / "presets"
    prompt_dir.mkdir(parents=True, exist_ok=True)
    if with_prompts:
        # Файлы для ОБОИХ пресетов из yaml ниже. Двуязычный формат как в
        # реальных presets/*.txt: RU-часть сверху, EN-часть с маркера
        # "EN version" (test_loads_yaml_and_prompts проверяет обе секции,
        # а _formalize_with_llm разруливает их по этому маркеру).
        for _key in ("technical", "lenin"):
            (prompt_dir / f"{_key}.txt").write_text(
                "RU prompt\n"
                "================================================================================\n"
                "EN version (same rules, English output)\n"
                "================================================================================\n"
                "EN prompt\n",
                encoding="utf-8",
            )
    yaml_path = tmpdir / "voice_presets.yaml"
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "presets": {
                    "technical": {
                        "name": "Технический",
                        "prompt_file": "presets/technical.txt",
                    },
                    "lenin": {
                        "name": "Ленин",
                        "prompt_file": "presets/lenin.txt",
                    },
                },
                "languages": ["ru", "en"],
                "default_preset": "technical",
                "default_language": "ru",
            },
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return str(yaml_path)


# ─────────────────────────────────────────────────────────────────────────────
#  _resolve_voice_preset / _resolve_voice_language
# ─────────────────────────────────────────────────────────────────────────────


class TestResolveVoicePreset:
    """Резолвер пресета: параметр → default → unknown-fallback."""

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_default_when_param_empty(self, tmp_path):
        n = _make_node()
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)
        # Не задан voice_preset → берём default_preset из yaml.
        assert n._resolve_voice_preset() == "technical"

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_explicit_preset_resolved(self, tmp_path):
        n = _make_node(voice_preset="lenin")
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)
        assert n._resolve_voice_preset() == "lenin"

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_unknown_preset_falls_back_to_default(self, tmp_path):
        n = _make_node(voice_preset="nonexistent_zzz")
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)
        # Неизвестный ключ → default_preset (technical), а не KeyError.
        assert n._resolve_voice_preset() == "technical"


class TestResolveVoiceLanguage:
    """Резолвер языка: параметр → default → invalid-fallback."""

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_default_when_param_empty(self, tmp_path):
        n = _make_node()
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)
        assert n._resolve_voice_language() == "ru"

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_explicit_language_resolved(self, tmp_path):
        n = _make_node(voice_output_language="en")
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)
        assert n._resolve_voice_language() == "en"

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_unknown_language_falls_back(self, tmp_path):
        n = _make_node(voice_output_language="fr")
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)
        assert n._resolve_voice_language() == "ru"

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_language_normalized_to_lower(self, tmp_path):
        n = _make_node(voice_output_language="EN")
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)
        assert n._resolve_voice_language() == "en"


class TestFormalizeTimeout:
    """Таймаут формализатора: параметр > 0 → он; иначе llm_timeout_sec."""

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_zero_falls_back_to_llm_timeout_sec(self):
        n = _make_node(voice_formalize_timeout_sec=0.0)
        n._llm_timeout_sec = 25.0
        _install_param_getter(n)
        assert n._get_formalize_timeout() == 25.0

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_explicit_value_wins(self):
        n = _make_node(voice_formalize_timeout_sec=7.5)
        n._llm_timeout_sec = 25.0
        _install_param_getter(n)
        assert n._get_formalize_timeout() == 7.5

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_fallback_to_30_when_both_missing(self):
        n = _make_node(voice_formalize_timeout_sec=0.0)
        n._llm_timeout_sec = 0.0
        _install_param_getter(n)
        # llm_timeout_sec=0 → fallback 30s (страховка от зависания).
        assert n._get_formalize_timeout() == 30.0


# ─────────────────────────────────────────────────────────────────────────────
#  _load_voice_presets: cache, prompt_files, fallback на отсутствие
# ─────────────────────────────────────────────────────────────────────────────


class TestLoadVoicePresets:
    """_load_voice_presets: чтение yaml, prompt_files, кэш."""

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_loads_yaml_and_prompts(self, tmp_path):
        n = _make_node()
        _install_param_getter(n)
        yaml_path = _write_minimal_yaml(tmp_path)
        n._voice_presets_file = yaml_path

        data = n._load_voice_presets()
        assert set(data["presets"].keys()) == {"technical", "lenin"}
        # Каждый пресет имеет name + prompt_text (не пустой).
        for key in ("technical", "lenin"):
            assert data["presets"][key]["name"]
            assert "RU prompt" in data["presets"][key]["prompt_text"]
            assert "EN version" in data["presets"][key]["prompt_text"]
        assert data["default_preset"] == "technical"
        assert data["default_language"] == "ru"
        assert set(data["languages"]) == {"ru", "en"}

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_caches_until_path_changes(self, tmp_path):
        n = _make_node()
        _install_param_getter(n)
        yaml_path = _write_minimal_yaml(tmp_path)
        n._voice_presets_file = yaml_path

        first = n._load_voice_presets()
        # Второй вызов — без FS-доступа, должна вернуться кэш-копия (тот же dict).
        second = n._load_voice_presets()
        assert first is second  # именно identity, не equality

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_missing_yaml_returns_safe_fallback(self, tmp_path):
        n = _make_node()
        _install_param_getter(n)
        n._voice_presets_file = str(tmp_path / "no-such.yaml")

        data = n._load_voice_presets()
        # presets пустой, но default_*/default_language разумные — чтобы
        # _resolve_* не падали, а _formalize_with_llm видел пустой prompt_text
        # и уходил в fallback на _speak_direct.
        assert data["presets"] == {}
        assert data["default_preset"] == "technical"
        assert data["default_language"] == "ru"


# ─────────────────────────────────────────────────────────────────────────────
#  _on_quest_stt: routing для quest_llm_formalize
# ─────────────────────────────────────────────────────────────────────────────


class TestQuestLlmFormalizeRouting:
    """Роутинг: quest_llm_formalize → schedule formalize, не дословный TTS."""

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_mode_schedules_formalize_via_loop(self, tmp_path, monkeypatch):
        n = _make_node()
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)
        n._voice_mode = "quest_llm_formalize"
        # get_parameter для mode должно возвращать voice_input_mode=...
        state = {"voice_input_mode": "quest_llm_formalize"}
        n.get_parameter = lambda name: _Param(state.get(name, getattr(n, "_" + name, "")))

        scheduled = MagicMock(return_value="future-handle")

        def fake_rcts(coro, loop):
            # Закрываем coroutine, чтобы не было warning'а "never awaited".
            try:
                coro.close()
            except Exception:
                pass
            return scheduled(coro, loop)

        monkeypatch.setattr(asyncio, "run_coroutine_threadsafe", fake_rcts)

        n._on_quest_stt(_make_stt_msg("привет как дела"))

        # Формализатор запланирован на self._loop.
        assert scheduled.called
        # _speak_direct НЕ вызван — формализатор ещё не отработал.
        n._speak_direct.assert_not_called()

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_empty_text_skips(self, monkeypatch):
        """Пустая фраза в formalize-режиме — НИЧЕГО не делаем (как в quest_ttts)."""
        n = _make_node()
        n._voice_mode = "quest_llm_formalize"
        state = {"voice_input_mode": "quest_llm_formalize"}
        n.get_parameter = lambda name: _Param(state.get(name, ""))
        # Даже если бы run_coroutine_threadsafe сработал — мы должны выйти до.
        n._loop = MagicMock()
        scheduled = MagicMock()
        monkeypatch.setattr(asyncio, "run_coroutine_threadsafe", scheduled)

        n._on_quest_stt(_make_stt_msg("   "))

        scheduled.assert_not_called()
        n._speak_direct.assert_not_called()

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_schedule_failure_falls_back_to_direct_tts(self, tmp_path, monkeypatch):
        """Если asyncio.run_coroutine_threadsafe бросил — fallback на дословный TTS."""
        n = _make_node()
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)
        n._voice_mode = "quest_llm_formalize"
        state = {"voice_input_mode": "quest_llm_formalize"}
        n.get_parameter = lambda name: _Param(state.get(name, ""))

        def boom(coro, loop):
            try:
                coro.close()
            except Exception:
                pass
            raise RuntimeError("simulated scheduler failure")

        monkeypatch.setattr(asyncio, "run_coroutine_threadsafe", boom)

        n._on_quest_stt(_make_stt_msg("привет как дела"))

        # Fallback: дословный TTS — это и есть контракт «как в P7-simple».
        n._speak_direct.assert_called_once_with("привет как дела")


# ─────────────────────────────────────────────────────────────────────────────
#  _formalize_with_llm: успешный путь, fallback'и
# ─────────────────────────────────────────────────────────────────────────────


class TestFormalizeWithLlm:
    """Сам _formalize_with_llm — async-функция, прогоняем через asyncio.run."""

    def _run(self, coro):
        """Прогнать корутину в новом loop'е (тест-изоляция от self._loop)."""
        return asyncio.new_event_loop().run_until_complete(coro)

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_success_path_calls_speak_direct_with_rewrite(self, tmp_path):
        n = _make_node()
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)

        # Фейковый LLM: complete(messages, tools=[]) → response с переписанной фразой.
        fake_response = MagicMock()
        fake_response.content = "Технически выражаясь, привет."
        # side_effect, а не return_value: ``complete`` вызывается внутри
        # ``asyncio.wait_for`` и обязана вернуть НОВУЮ корутину на каждый
        # вызов. return_value=self._run(...) выполнил бы её здесь же.
        n._llm.complete = MagicMock(side_effect=lambda *a, **k: _async_return(fake_response))

        self._run(n._formalize_with_llm("привет как дела", "technical", "ru"))

        n._llm.complete.assert_called_once()
        # tools=[] — формализатор не должен давать LLM доступ к инструментам.
        args = n._llm.complete.call_args.args[0]
        assert args[0].role == "system"
        assert args[1].role == "user"
        # RU-режим: system-промпт — только RU-секция (без EN-секции),
        # user-сообщение — исходная фраза + языковая директива.
        assert "RU prompt" in args[0].content
        assert "EN version" not in args[0].content
        assert "привет как дела" in args[1].content
        assert "«русский»" in args[1].content
        assert n._llm.complete.call_args.kwargs.get("tools", []) == []

        # В итоге TTS получил переписанную фразу.
        n._speak_direct.assert_called_once_with("Технически выражаясь, привет.")

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_en_language_uses_en_prompt_section(self, tmp_path):
        """voice_output_language=en → system-промпт = EN-секция, user — «английский»."""
        n = _make_node()
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)

        fake_response = MagicMock()
        fake_response.content = "Hello."
        n._llm.complete = MagicMock(side_effect=lambda *a, **k: _async_return(fake_response))

        self._run(n._formalize_with_llm("привет", "technical", "en"))

        args = n._llm.complete.call_args.args[0]
        assert "EN prompt" in args[0].content
        assert "RU prompt" not in args[0].content
        assert "«английский»" in args[1].content
        n._speak_direct.assert_called_once_with("Hello.")

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_llm_timeout_falls_back_to_original(self, tmp_path):
        n = _make_node(voice_formalize_timeout_sec=0.05)
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)

        async def slow():
            await asyncio.sleep(5.0)
            return MagicMock(content="too late")

        n._llm.complete = MagicMock(side_effect=lambda *a, **k: slow())

        self._run(n._formalize_with_llm("привет", "technical", "ru"))

        # Fallback на дословный TTS.
        n._speak_direct.assert_called_once_with("привет")

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_empty_rewrite_falls_back_to_original(self, tmp_path):
        n = _make_node()
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)

        fake_response = MagicMock()
        fake_response.content = ""
        n._llm.complete = MagicMock(side_effect=lambda *a, **k: _async_return(fake_response))

        self._run(n._formalize_with_llm("привет", "technical", "ru"))

        n._speak_direct.assert_called_once_with("привет")

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_identical_rewrite_falls_back(self, tmp_path):
        """Если LLM вернул исходную фразу без изменений — fallback."""
        n = _make_node()
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)

        fake_response = MagicMock()
        fake_response.content = "привет как дела"
        n._llm.complete = MagicMock(side_effect=lambda *a, **k: _async_return(fake_response))

        self._run(n._formalize_with_llm("привет как дела", "technical", "ru"))

        n._speak_direct.assert_called_once_with("привет как дела")

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_llm_provider_error_falls_back(self, tmp_path):
        n = _make_node()
        _install_param_getter(n)
        n._voice_presets_file = _write_minimal_yaml(tmp_path)

        # Имитируем ProviderError из rob_box_llm.errors.
        from rob_box_llm.errors import ProviderError

        async def fail():
            raise ProviderError("upstream down")

        # Без side_effect ProviderError вылетал бы прямо здесь, в теле
        # теста, и до _formalize_with_llm дело бы не дошло.
        n._llm.complete = MagicMock(side_effect=lambda *a, **k: fail())

        self._run(n._formalize_with_llm("привет", "technical", "ru"))

        n._speak_direct.assert_called_once_with("привет")

    @pytest.mark.skipif(
        _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
        reason=_SKIP_REASON,
    )
    def test_missing_prompt_text_falls_back_immediately(self, tmp_path):
        """Если prompt_file не задан — fallback ДО вызова LLM."""
        n = _make_node()
        _install_param_getter(n)
        yaml_path = tmp_path / "voice_presets.yaml"
        yaml_path.write_text(
            yaml.safe_dump(
                {
                    "presets": {
                        # Нет prompt_file → prompt_text пустой.
                        "void": {"name": "Пустой"},
                    },
                    "languages": ["ru"],
                    "default_preset": "void",
                    "default_language": "ru",
                },
                allow_unicode=True,
            ),
            encoding="utf-8",
        )
        n._voice_presets_file = str(yaml_path)

        self._run(n._formalize_with_llm("привет", "void", "ru"))

        # LLM не дёрнут.
        n._llm.complete.assert_not_called()
        # Fallback на дословный TTS.
        n._speak_direct.assert_called_once_with("привет")


# ─────────────────────────────────────────────────────────────────────────────
#  helpers
# ─────────────────────────────────────────────────────────────────────────────


async def _async_return(value):
    return value
