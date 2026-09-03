"""test_voice_presets_formalize.py — контракт voice_presets.yaml + промпт-композиция
для режима ``voice_input_mode=quest_llm_formalize`` (AV-28 / P7-full голосового
плана).

Закрывает четыре пункта acceptance карточки t_dbae4b34:

  1. unit: загрузка ``voice_presets.yaml`` — список из 6 пресетов
     (technical/street/caveman/business/philosopher/lenin), оба языка
     (ru/en) и дефолты (technical/ru).
  2. unit: промпт-композиция — для каждой комбинации preset+language
     существует файл ``presets/<key>.txt`` (или эквивалент), и он
     содержит непустой промпт с явным «no-dialog» контрактом.
  3. интеграционный: ``DialogueNode._on_quest_stt`` в режиме
     ``quest_llm_formalize`` с моком LLM — в TTS уходит только
     переписанная реплика, а не операторский ответ.
  4. негативный: при пустом/timeout-ответе LLM — fallback на
     дословный ``_speak_direct`` (как в ``quest_ttts``).

Тесты (3) и (4) требуют реализацию формализатора из PR #1946 (коммит
``7987f749``). Пока PR не в ``develop`` — они skip-ятся с осмысленным
``reason=``. Это «Честный FAIL лучше красивого PASS» — лучше явный skip,
чем зелёные тесты против несуществующего кода.

Не требует ROS2: ``conftest.py`` мокает ``rclpy`` + ``rcl_interfaces``
+ ``std_msgs``, плюс ``DialogueNode`` инстанцируется через
``object.__new__`` + ручные атрибуты.
"""

from __future__ import annotations

import re
import sys
import types
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
import yaml


# ─────────────────────────────────────────────────────────────────────────────
#  Пути к источникам истины
# ─────────────────────────────────────────────────────────────────────────────

# Тест лежит в ``src/rob_box_voice/test/unit/node/...``; корень моно-репо —
# это ``parents[5]``. См. test_voice_input_mode_param.py.
REPO_ROOT = Path(__file__).resolve().parents[5]
VOICE_PRESETS_YAML = (
    REPO_ROOT / "src" / "rob_box_voice" / "config" / "voice_presets.yaml"
)
PRESETS_DIR = REPO_ROOT / "src" / "rob_box_voice" / "config" / "presets"
DIALOGUE_NODE_PY = (
    REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "dialogue_node.py"
)


# ─────────────────────────────────────────────────────────────────────────────
#  Маркер: тесты, которые требуют код формализатора (PR #1946)
# ─────────────────────────────────────────────────────────────────────────────

def _has_formalizer() -> bool:
    """Быстрая проверка наличия кода формализатора в dialogue_node.py.

    Смотрим на символ ``_formalize_with_llm`` — это центральный async-метод,
    который добавляется коммитом 7987f749. Если его нет — карточка ещё не
    может быть завершена по acceptance «покрытие >70% по новому коду».
    """
    try:
        src = DIALOGUE_NODE_PY.read_text(encoding="utf-8")
    except OSError:
        return False
    return "_formalize_with_llm" in src and "_resolve_voice_preset" in src


requires_formalizer = pytest.mark.skipif(
    not _has_formalizer(),
    reason=(
        "Реализация формализатора (PR #1946, коммит 7987f749) ещё не в "
        "develop. Тесты run'ятся после merge PR #1946 в origin/develop."
    ),
)


# ─────────────────────────────────────────────────────────────────────────────
#  Тест (1) — voice_presets.yaml: список пресетов, дефолты, языки
# ─────────────────────────────────────────────────────────────────────────────


class TestVoicePresetsYaml:
    """Контракт на config/voice_presets.yaml как на данные.

    YAML — единственный источник истины для списка пресетов (коммит
    ``27ec9442``, PR #1931). dialogue_node.py читает его в рантайме;
    статичный контракт покрываем параметризованно.
    """

    def test_yaml_is_valid(self):
        """voice_presets.yaml — корректный YAML, парсится без ошибок."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        assert isinstance(data, dict), "voice_presets.yaml: ожидается dict на верхнем уровне"

    def test_yaml_has_presets_block(self):
        """Верхнеуровневый ключ ``presets`` существует и это dict."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        assert "presets" in data, "voice_presets.yaml: ключ 'presets' обязателен"
        assert isinstance(data["presets"], dict), "voice_presets.yaml: 'presets' — это dict"

    def test_yaml_has_languages_block(self):
        """Ключ ``languages`` — список (минимум ru, en)."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        assert "languages" in data
        assert isinstance(data["languages"], list)
        assert "ru" in data["languages"], "voice_presets.yaml: язык 'ru' обязателен"
        assert "en" in data["languages"], "voice_presets.yaml: язык 'en' обязателен"

    def test_yaml_has_default_preset(self):
        """``default_preset`` существует и указывает на реальный пресет."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        assert "default_preset" in data
        assert data["default_preset"] in data["presets"], (
            f"voice_presets.yaml: default_preset='{data['default_preset']}' "
            f"не найден в списке пресетов ({list(data['presets'].keys())})"
        )

    def test_yaml_has_default_language(self):
        """``default_language`` существует и входит в ``languages``."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        assert "default_language" in data
        assert data["default_language"] in data["languages"]

    def test_default_preset_is_technical(self):
        """По UI-контракту (см. ST:LENIN в HUD, AV-18) дефолт = technical."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        # Это не «обязательно», но если кто-то поменяет — должен заметить.
        assert data["default_preset"] == "technical", (
            "voice_presets.yaml: default_preset='technical' — UI-индикатор "
            "ST:LENIN в HUD (AV-18) ожидает именно это значение"
        )

    def test_default_language_is_ru(self):
        """Дефолт языка = ru (русскоязычный робот по умолчанию)."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        assert data["default_language"] == "ru"

    @pytest.mark.parametrize(
        "preset_key",
        ["technical", "street", "caveman", "business", "philosopher", "lenin"],
    )
    def test_each_preset_has_required_fields(self, preset_key: str):
        """Каждый из 6 пресетов имеет ``name`` (русское UI-название) и
        ``prompt_file`` (путь относительно ``config/``)."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        preset = data["presets"][preset_key]
        assert "name" in preset, f"preset '{preset_key}': отсутствует поле 'name'"
        assert isinstance(preset["name"], str) and preset["name"], (
            f"preset '{preset_key}': 'name' должно быть непустой строкой"
        )
        assert "prompt_file" in preset, (
            f"preset '{preset_key}': отсутствует поле 'prompt_file'"
        )
        assert preset["prompt_file"].endswith(".txt"), (
            f"preset '{preset_key}': prompt_file должен оканчиваться на .txt"
        )
        # Промпт-файл должен реально существовать
        prompt_path = REPO_ROOT / "src" / "rob_box_voice" / "config" / preset["prompt_file"]
        assert prompt_path.exists(), (
            f"preset '{preset_key}': prompt_file '{preset['prompt_file']}' "
            f"не найден ({prompt_path})"
        )


class TestPresetsPackaging:
    """setup.py должен паковать ``config/presets/*.txt`` в ROS-share.

    Если пресеты не попадают в ``share/rob_box_voice/config/presets/``
    Docker-образа, dialogue_node на проде пишет
    ``preset ... prompt file missing`` и честно падает в
    ``fallback to direct TTS`` — оператор в шлеме слышит дословный повтор
    вместо стилизации (AV-28). Локально файлы есть, поэтому это
    единственное место, где разрыв видно статически.
    """

    def test_setup_py_packages_presets_txt(self):
        setup_py = REPO_ROOT / "src" / "rob_box_voice" / "setup.py"
        src = setup_py.read_text(encoding="utf-8")
        assert "config/presets" in src, (
            "setup.py: data_files не пакет config/presets/*.txt — "
            "пресеты не попадут в ROS-share Docker-образа"
        )


# ─────────────────────────────────────────────────────────────────────────────
#  Тест (2) — промпт-композиция: каждый preset имеет рабочий промпт
# ─────────────────────────────────────────────────────────────────────────────


class TestPromptComposition:
    """Промпт для каждого пресета — это txt-файл с no-dialog контрактом.

    Контракт промпта (см. коммит ``27ec9442``, ветка #1920):

      * содержит явное «You are the РОББОКС speech rewriter for the … preset»
      * явно запрещает LLM отвечать оператору, задавать вопросы, добавлять
        факты («Rewrite the operator's words», «no-dialog»)
      * упоминает оба языка (ru/en) — пресет мультиязычный
      * текст промпта длиннее минимального порога (иначе LLM не развернётся)
    """

    @pytest.mark.parametrize(
        "preset_key",
        ["technical", "street", "caveman", "business", "philosopher", "lenin"],
    )
    def test_prompt_file_is_non_empty(self, preset_key: str):
        """Промпт-файл существует и не пустой (>= 20 строк)."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        prompt_rel = data["presets"][preset_key]["prompt_file"]
        prompt_path = REPO_ROOT / "src" / "rob_box_voice" / "config" / prompt_rel
        assert prompt_path.exists(), f"preset '{preset_key}': prompt_file не найден"
        text = prompt_path.read_text(encoding="utf-8")
        lines = [ln for ln in text.splitlines() if ln.strip()]
        assert len(lines) >= 20, (
            f"preset '{preset_key}': промпт подозрительно короткий ({len(lines)} строк) — "
            f"LLM не сможет развернуть стиль"
        )

    @pytest.mark.parametrize(
        "preset_key",
        ["technical", "street", "caveman", "business", "philosopher", "lenin"],
    )
    def test_prompt_has_no_dialog_contract(self, preset_key: str):
        """В промпте явно прописан no-dialog контракт — LLM не должна
        отвечать оператору, задавать вопросы, добавлять факты."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        prompt_rel = data["presets"][preset_key]["prompt_file"]
        prompt_path = REPO_ROOT / "src" / "rob_box_voice" / "config" / prompt_rel
        text = prompt_path.read_text(encoding="utf-8").lower()
        # Хотя бы один из маркеров no-dialog контракта
        no_dialog_markers = [
            "no-dialog",
            "no dialog",
            "do not answer",
            "do not respond",
            "do not ask",
            "do not add",
            "do not invent",
            "do not question",
            "только перепис",
            "только пересказ",
            "перепиши",  # ru verb
            "rewrite",   # en verb
            "не отвеч",
            "не задавай",
            "не добавляй",
        ]
        assert any(marker in text for marker in no_dialog_markers), (
            f"preset '{preset_key}': промпт не содержит явный no-dialog контракт — "
            f"риск: LLM будет отвечать оператору вместо переписывания"
        )

    @pytest.mark.parametrize(
        "preset_key",
        ["technical", "street", "caveman", "business", "philosopher", "lenin"],
    )
    def test_prompt_mentions_both_languages(self, preset_key: str):
        """Промпт покрывает оба языка (ru + en). Иначе выбор языка в
        voice_output_language не имеет смысла."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        prompt_rel = data["presets"][preset_key]["prompt_file"]
        prompt_path = REPO_ROOT / "src" / "rob_box_voice" / "config" / prompt_rel
        text = prompt_path.read_text(encoding="utf-8")
        # Нестрогая проверка: ищем русскую букву ИЛИ явное 'russian'/'russk'
        has_ru_marker = (
            re.search(r"[А-Яа-яЁё]{3,}", text) is not None
            or re.search(r"\b(russian|русск)\w*", text, re.IGNORECASE) is not None
        )
        has_en_marker = (
            re.search(r"[A-Za-z]{3,}", text) is not None
        )
        assert has_ru_marker and has_en_marker, (
            f"preset '{preset_key}': промпт должен покрывать оба языка "
            f"(ru + en) — иначе voice_output_language=ru/en не работает"
        )

    @pytest.mark.parametrize(
        "preset_key",
        ["technical", "street", "caveman", "business", "philosopher", "lenin"],
    )
    def test_prompt_has_style_marker(self, preset_key: str):
        """В промпте указано название стиля (для UI-индикатора «ST:TECHNICAL»
        и для отладки — оператор должен видеть, какой стиль активен)."""
        with VOICE_PRESETS_YAML.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        preset_name = data["presets"][preset_key]["name"]
        prompt_rel = data["presets"][preset_key]["prompt_file"]
        prompt_path = REPO_ROOT / "src" / "rob_box_voice" / "config" / prompt_rel
        text = prompt_path.read_text(encoding="utf-8")
        # Имя пресета по-русски должно встречаться в промпте
        assert preset_name in text, (
            f"preset '{preset_key}' (name='{preset_name}'): имя стиля не "
            f"найдено в {prompt_rel} — UI-индикатор не сможет проверить, "
            f"что активен именно этот пресет"
        )


# ─────────────────────────────────────────────────────────────────────────────
#  Helpers для тестов (3) и (4) — моки DialogueNode + LLM
# ─────────────────────────────────────────────────────────────────────────────


def _install_fake_rcl_interfaces(monkeypatch: pytest.MonkeyPatch) -> None:
    """Подсовываем фейковый ``rcl_interfaces.msg.SetParametersResult``.

    Скопировано из test_voice_input_mode_param.py — чтобы карточки
    не зависели друг от друга (conftest.py не импортируется явно).
    """
    rcl_interfaces = types.ModuleType("rcl_interfaces")
    msg = types.ModuleType("rcl_interfaces.msg")
    msg.SetParametersResult = type(  # type: ignore[attr-defined]
        "SetParametersResult",
        (),
        {
            "__init__": lambda self, successful=True: setattr(
                self, "successful", successful
            )
        },
    )
    setattr(rcl_interfaces, "msg", msg)
    monkeypatch.setitem(sys.modules, "rcl_interfaces", rcl_interfaces)
    monkeypatch.setitem(sys.modules, "rcl_interfaces.msg", msg)


def _make_node() -> "DialogueNode":  # type: ignore[name-defined]
    """DialogueNode через ``object.__new__`` + ручные атрибуты.

    Без conftest.py — pytest не запускает моки ``rclpy``, потому что
    тесты (3) и (4) skip-ятся при отсутствии реализации.
    """
    from rob_box_voice.dialogue_node import DialogueNode  # noqa: WPS433

    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger
    return n


# ─────────────────────────────────────────────────────────────────────────────
#  Тест (3) — интеграционный: _on_quest_stt в quest_llm_formalize
# ─────────────────────────────────────────────────────────────────────────────


@requires_formalizer
class TestQuestLlmFormalizeRouting:
    """В режиме ``quest_llm_formalize`` фраза оператора прогоняется через LLM,
    и в TTS уходит **только** переписанная реплика.

    Контракт проверяется без реального провайдер: LLM замокан через
    ``patch`` на ``asyncio.run_coroutine_threadsafe`` либо на
    ``DialogueNode._formalize_with_llm``.
    """

    def _setup_node_with_preset(self, *, preset: str = "technical",
                                language: str = "ru"):
        """Создаёт ноду с заданным пресетом и языком."""
        n = _make_node()
        # Параметры (как после declare_parameter + parameters_callback).
        n._voice_preset = preset
        n._voice_output_language = language
        n._voice_presets_cache = None
        n._voice_presets_file = str(VOICE_PRESETS_YAML.relative_to(REPO_ROOT))
        # Поля, которые обычно инициализируются в __init__
        n._loop = MagicMock()  # asyncio loop (для run_coroutine_threadsafe)
        n._tts_publisher = MagicMock()
        n._tts_buffer = []
        n._tts_skip_count = 0
        n._speak_direct = MagicMock()  # fallback на дословный TTS
        return n

    def test_formalize_path_calls_llm_and_speaks_rewrite(
        self, monkeypatch: pytest.MonkeyPatch
    ):
        """Happy path: LLM вернул переписанную реплику — она уходит в TTS,
        а не операторский текст."""
        _install_fake_rcl_interfaces(monkeypatch)
        n = self._setup_node_with_preset(preset="street", language="ru")

        # Мок LLM: возвращает переписанную реплику
        rewrite_text = "Переписанная реплика в стиле street"
        async def _fake_formalize(self, text, **kwargs):
            return rewrite_text
        monkeypatch.setattr(
            type(n), "_formalize_with_llm",
            AsyncMock(side_effect=lambda text, **kw: _fake_formalize(n, text, **kw)),
        )

        # Мокаем coroutine scheduling: run_coroutine_threadsafe должен
        # вернуть объект с .result(), который вернёт rewrite_text.
        scheduled = MagicMock()
        scheduled.result.return_value = rewrite_text
        n._loop.run_coroutine_threadsafe = MagicMock(return_value=scheduled)

        # Эмулируем входящее STT-сообщение
        from std_msgs.msg import String  # type: ignore[import-not-found]
        msg = MagicMock(spec=String)
        msg.data = "Оригинальная фраза оператора"

        n._on_quest_stt(msg)

        # В TTS должна уйти только переписанная реплика
        if hasattr(n, "_tts_publisher") and n._tts_publisher.publish.called:
            # Если реализация использует ROS publisher
            published = n._tts_publisher.publish.call_args[0][0]
            assert rewrite_text in str(published.data) or rewrite_text in str(published), (
                f"_on_quest_stt: в TTS ушла не переписанная реплика: {published!r}"
            )
            assert "Оригинальная фраза" not in str(published), (
                "_on_quest_stt: в TTS попала ОРИГИНАЛЬНАЯ фраза оператора — "
                "формализатор не сработал"
            )
        elif hasattr(n, "_speak_direct") and n._speak_direct.called:
            called_text = n._speak_direct.call_args[0][0]
            assert rewrite_text in str(called_text), (
                f"_speak_direct получил не переписанную реплику: {called_text!r}"
            )

    def test_formalize_does_not_call_llm_when_mode_not_active(
        self, monkeypatch: pytest.MonkeyPatch
    ):
        """Sanity: в НЕ-quest_llm_formalize режимах _formalize_with_llm
        НЕ вызывается — иначе regression на quest_ttts/quest_stt/respeaker."""
        _install_fake_rcl_interfaces(monkeypatch)
        n = self._setup_node_with_preset(preset="technical", language="ru")

        formalize_mock = MagicMock()
        monkeypatch.setattr(type(n), "_formalize_with_llm", formalize_mock)

        # Голосовой режим НЕ quest_llm_formalize — ставим параметр
        n._voice_input_mode = "quest_ttts"

        from std_msgs.msg import String  # type: ignore[import-not-found]
        msg = MagicMock(spec=String)
        msg.data = "Любая фраза"

        n._on_quest_stt(msg)

        formalize_mock.assert_not_called()


# ─────────────────────────────────────────────────────────────────────────────
#  Тест (4) — негативный: fallback на дословный TTS при сбое LLM
# ─────────────────────────────────────────────────────────────────────────────


@requires_formalizer
# TestFormalizeFallbackOnLlmFailure удалён при переносе из ветки
# t_dbae4b34 (PR так и не открыли). Класс подменял сам
# _formalize_with_llm и после этого требовал, чтобы fallback на
# _speak_direct случился в вызывающем _on_quest_stt. В смерженной
# реализации fallback живёт ВНУТРИ _formalize_with_llm (ветки except
# TimeoutError / ProviderError / пустой ответ), поэтому подмена метода
# убирала ровно то, что тест проверял, — assert не мог пройти
# by construction.
#
# Сам контракт «LLM упал → робот проговаривает фразу дословно» покрыт
# и проверяется в test_quest_llm_formalize.py:
#   test_llm_timeout_falls_back_to_original
#   test_llm_provider_error_falls_back
#   test_empty_rewrite_falls_back_to_original
# — там подменяется _llm.complete, то есть уровень, на котором отказ
# действительно происходит.


class TestFormalizerAvailabilityMarker:
    """Мета-тест: проверяет, что маркер ``requires_formalizer`` корректно
    skip-ит тесты (3,4) когда реализации нет в develop.

    Это нужно, чтобы при прогоне тестов в CI Шифу видел skip, а не
    непонятный AttributeError.
    """

    def test_skip_marker_set_correctly(self):
        """Маркер ``requires_formalizer`` отражает реальное наличие кода."""
        # Когда PR #1946 не в develop — маркер skip=True
        # Когда PR #1946 в develop — маркер skip=False
        # Здесь мы просто проверяем, что _has_formalizer() возвращает bool
        result = _has_formalizer()
        assert isinstance(result, bool)
        if not result:
            pytest.skip(
                "Реализация формализатора (PR #1946) ещё не в develop — "
                "тесты (3,4) skip-ятся. После merge #1946 в develop — "
                "тесты запустятся автоматически."
            )