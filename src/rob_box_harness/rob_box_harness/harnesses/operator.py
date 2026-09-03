"""``OperatorHarness`` — каркас супервизор-агента (AV-21 / issue #1913).

**Суть**

Мозг оператора: получает команду (от Quest-голоса или Telegram-текста),
гоняет через LLM с фиксированным системным промптом, выполняет
tool-calls через ``ToolProvider``, отдаёт структурированный результат.

**Почему harness, а не «внутренний модуль supervisor-а»** — см.
``docs/plans/2026-09-02-avatar-supervisor-agent-design.md`` §2: harness
уже предоставляет lifecycle, snapshot, порты (LLM/Tools/Memory/Effects/
Transport) и конфиг-формат. Делать дубль = ADR-0021 R1 нарушен.

**Контракт инструментов (жёсткий):**

* Имена инструментов берутся из ``OPERATOR_TOOL_NAMES`` ниже.
* Схемы (JSON-Schema) берутся **только** из
  ``rob_box_core.tool_catalog``. Никаких собственных объявлений.
* Если в каталоге нет инструмента с таким именем — ``init()`` падает
  с ``ConfigError`` (тест-инвариант ловит это).

**Каркас (AV-21), не финальная интеграция:**

* ``say`` и ``play_animation`` выполняются через ``FakeToolProvider`` с
  stub-handler-ами (см. ``_register_stub_handlers``). Реальная
  интеграция с TTS-каналом и LED-каналом — карточки AV-27 / AV-28.
* LLM-провайдер — ``DummyLLMProvider`` если не задан. Тесты обязаны
  подменять на ``FakeLLMProvider`` (см. ``test_operator_harness.py``).
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from rob_box_core.tool_catalog import get_tool
from rob_box_harness.config import HarnessConfig
from rob_box_harness.harness import Harness
from rob_box_harness.tools import FakeToolProvider, ToolSpec

logger = logging.getLogger(__name__)


# ------------------------------------------------------------------
# Каталог инструментов оператора
# ------------------------------------------------------------------
#
# Это **единственное** место в этом модуле, где перечислены имена
# инструментов. Схемы (JSON-Schema) — в ``rob_box_core.tool_catalog``.
# Тест ``test_operator_no_local_schema`` грепает, что здесь нет ни
# одного ``{"type": "object", ...}`` / ``"parameters": {...}``.
#
# Порядок важен — это порядок, в котором инструменты предъявляются
# LLM. ``say`` первым, потому что это голос оператора (самое частое).
OPERATOR_TOOL_NAMES: tuple[str, ...] = (
    "say",
    "play_animation",
)


# ------------------------------------------------------------------
# Дефолтный путь к системному промпту
# ------------------------------------------------------------------
#
# Полный путь: ``<harness-package>/prompts/operator_system_prompt.txt``.
# Это сделано для согласованности с другими prompts в проекте
# (``rob_box_voice/prompts/master_prompt_compact.txt`` и т.п.). Отдельный
# файл — требование карточки (а не строка в коде).
_DEFAULT_PROMPT_FILE = "operator_system_prompt.txt"


# ------------------------------------------------------------------
# OperatorState
# ------------------------------------------------------------------


@dataclass
class OperatorState:
    """Runtime state bag для OperatorHarness.

    ``Mapping[str, Any]`` через ``__getitem__``/``__iter__``/``__len__``
    нужно для ``Harness``-контракта, который хранит ``state`` как
    mapping и сериализует в ``SessionSnapshot``. Явный dataclass
    делает контракт читаемым, а dict-контракт сохраняется через
    ``to_mapping`` ниже.
    """

    enabled: bool = True
    last_command_source: str = ""
    last_command_text: str = ""
    last_summary: str = ""
    turn_count: int = 0

    def to_mapping(self) -> Mapping[str, Any]:
        """Плоский mapping для snapshot."""
        return {
            "enabled": self.enabled,
            "last_command_source": self.last_command_source,
            "last_command_text": self.last_command_text,
            "last_summary": self.last_summary,
            "turn_count": self.turn_count,
        }


# Совместимость с типовым параметром Harness[StateT] (Mapping[str, Any]).
# Harness ожидает ``Mapping``, но в ``__init__`` родителя
# ``self.state: StateT = {}`` — здесь dataclass, который тоже
# поддерживает ``__getitem__``. Через явный cast при обращении
# (см. OperatorHarness.init ниже) и в ``state`` snapshot/restore.


# ------------------------------------------------------------------
# OperatorHarness
# ------------------------------------------------------------------


class OperatorHarness(Harness[OperatorState]):
    """Каркас супервизор-агента (AV-21).

    Не реализует LLM-диалог с пользователем (это dialogue_node).
    Реализует «команда → один или несколько tool-calls → summary».
    """

    name = "operator"

    def __init__(
        self,
        config: HarnessConfig,
        *,
        prompt_file: str | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(config, **kwargs)
        # ``prompt_file`` — опциональный override имени файла в
        # ``rob_box_harness/prompts/``. Если None — берём дефолт.
        self._prompt_file: str = prompt_file or _DEFAULT_PROMPT_FILE
        self._system_prompt: str = ""
        self._tool_specs: tuple[dict, ...] = ()

    # ── lifecycle ────────────────────────────────────────────────

    async def init(self) -> None:
        """Загрузить промпт, валидировать инструменты против каталога.

        1. Загрузить системный промпт из файла — ``RuntimeError`` если
           файла нет или он пустой (тест ловит).
        2. Проверить, что все ``OPERATOR_TOOL_NAMES`` есть в
           ``rob_box_core.tool_catalog`` — ``ConfigError`` если нет.
        3. Собрать tool_specs из каталога (НЕ объявлять локально).
        4. Зарегистрировать stub-handler-ы в ``FakeToolProvider``.
           Реальные backend-ы — карточки AV-27/AV-28.
        5. Выставить ``self.state = OperatorState(...)``.
        """
        await super().init()

        # (1) Загрузка промпта.
        self._system_prompt = self._load_system_prompt(self._prompt_file)

        # (2)+(3) Сборка tool_specs из каталога.
        self._tool_specs = self._build_tool_specs_from_catalog(OPERATOR_TOOL_NAMES)

        # (4) Stub-handler-ы для FakeToolProvider.
        if self.tools is None:
            self.tools = FakeToolProvider()
        assert isinstance(self.tools, FakeToolProvider), (
            "OperatorHarness каркас ожидает FakeToolProvider; "
            "реальная интеграция — отдельная карточка."
        )
        self._register_stub_handlers(self.tools)

        # (5) State.
        self.state = OperatorState()

    async def teardown(self) -> None:
        """Освободить ресурсы. Idempotent."""
        await super().teardown()

    # ── step ─────────────────────────────────────────────────────

    async def step(self, input_data: Any) -> Mapping[str, Any]:
        """Один turn оператора: команда → LLM → tool loop → summary.

        ``input_data`` — dict ``{"source", "client_id", "text"}`` либо
        строка (для упрощения smoke-тестов). Возвращает mapping::

            {
                "ok": bool,
                "summary": str,
                "tool_calls": [{"name", "arguments", "result"}],
                "error": str | None,
            }
        """
        # Coerce input.
        if isinstance(input_data, Mapping):
            text = str(input_data.get("text", "") or "").strip()
            source = str(input_data.get("source", "") or "")
            client_id = str(input_data.get("client_id", "") or "")
        else:
            text = str(input_data or "").strip()
            source = ""
            client_id = ""

        if not text:
            return {
                "ok": False,
                "summary": "empty_input",
                "tool_calls": [],
                "error": "empty text",
                "source": source,
                "client_id": client_id,
            }

        # Сохранить в state (для snapshot и e2e).
        assert isinstance(self.state, OperatorState)
        self.state.last_command_source = source
        self.state.last_command_text = text

        # Подготовить LLM-сообщения.
        from rob_box_llm.provider import LLMMessage

        messages: list[LLMMessage] = [
            LLMMessage(role="system", content=self._system_prompt),
            LLMMessage(role="user", content=text),
        ]

        # Если провайдер не имеет поддержки tools — fallback на
        # «голый» LLM-вызов (без tool_calls).
        if self.llm is None:
            return {
                "ok": False,
                "summary": "llm_not_configured",
                "tool_calls": [],
                "error": "LLMProvider is None",
                "source": source,
                "client_id": client_id,
            }

        # (a) Первый LLM-вызов: модель может вернуть tool_calls.
        try:
            response = await self.llm.complete(
                messages,
                tools=list(self._tool_specs) if self._tool_specs else (),
            )
        except Exception as exc:  # noqa: BLE001 — LLM-провайдеры бросают разное
            return {
                "ok": False,
                "summary": f"llm_error: {type(exc).__name__}",
                "tool_calls": [],
                "error": str(exc),
                "source": source,
                "client_id": client_id,
            }

        # Если LLM не запросил tool-calls — это валидный кейс
        # «нет подходящего инструмента» (см. design.md §6 acceptance 7).
        if not response.tool_calls:
            return {
                "ok": False,
                "summary": "no_tool",
                "tool_calls": [],
                "error": "LLM did not call any tool",
                "source": source,
                "client_id": client_id,
            }

        # (b) Tool loop: выполняем все вызовы, собираем результаты.
        executed: list[dict[str, Any]] = []
        all_ok = True
        no_such_tool = False
        for call in response.tool_calls:
            try:
                result = await self.tools.execute(call)
            except Exception as exc:  # noqa: BLE001
                # Транспортный сбой executor-а. Считаем весь turn
                # неудачным, но tool_calls[] всё равно публикуем.
                executed.append(
                    {
                        "name": call.name,
                        "arguments": dict(call.arguments),
                        "result": None,
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                )
                all_ok = False
                continue

            if result.is_error and "unknown tool" in result.content:
                # Hard invariant: LLM попросил инструмент, которого
                # нет в каталоге. Это не «программная» ошибка, это
                # нарушение инварианта (ADR-0018 в рантайме).
                no_such_tool = True

            executed.append(
                {
                    "name": call.name,
                    "arguments": dict(call.arguments),
                    "result": result.content,
                }
            )

        summary = "ok" if all_ok and not no_such_tool else "error"
        if no_such_tool:
            summary = "no_tool_in_catalog"

        # (c) Update state.
        assert isinstance(self.state, OperatorState)
        self.state.last_summary = summary
        self.state.turn_count += 1

        return {
            "ok": all_ok and not no_such_tool,
            "summary": summary,
            "tool_calls": executed,
            "source": source,
            "client_id": client_id,
        }

    # ── helpers ──────────────────────────────────────────────────

    def _load_system_prompt(self, prompt_file: str) -> str:
        """Прочитать системный промпт из файла в ``prompts/``.

        Промпт — **отдельный файл**, не строка в коде (требование
        карточки). Путь: ``<package>/prompts/<prompt_file>``.

        Raises:
            RuntimeError: если файл не найден или пустой.
        """
        # rob_box_harness/prompts/<file>
        prompts_dir = Path(__file__).resolve().parent.parent / "prompts"
        path = prompts_dir / prompt_file
        if not path.exists():
            raise RuntimeError(
                f"operator system prompt not found: {path} "
                "(check operator_system_prompt.txt in rob_box_harness/prompts/)"
            )
        text = path.read_text(encoding="utf-8").strip()
        if not text:
            raise RuntimeError(
                f"operator system prompt is empty: {path} "
                "(required by AV-21 acceptance: prompt must be loaded and non-empty)"
            )
        return text

    def _build_tool_specs_from_catalog(
        self,
        names: tuple[str, ...],
    ) -> tuple[dict, ...]:
        """Собрать OpenAI-style tool specs **из каталога**.

        Никаких собственных объявлений JSON-Schema в этом модуле
        (тест-инвариант ``test_operator_no_local_schema`` грепает файл).
        """
        specs: list[dict] = []
        for name in names:
            entry = get_tool(name)  # KeyError, если нет в каталоге
            specs.append(entry.to_openai_tool())
        return tuple(specs)

    def _register_stub_handlers(self, provider: FakeToolProvider) -> None:
        """Зарегистрировать заглушки executor-ов для say/play_animation.

        Реальная интеграция — карточки AV-27/AV-28. Здесь stub-ы
        возвращают ``ToolResult(content="ok", is_error=False)``.
        """
        for name in OPERATOR_TOOL_NAMES:
            # Берём spec из каталога — provider уже валидирован в
            # ``_build_tool_specs_from_catalog`` выше.
            spec_entry = get_tool(name)
            tool_spec = ToolSpec(
                name=name,
                description=spec_entry.description,
                parameters=dict(spec_entry.parameters),
            )

            async def _stub_handler(
                _args: Mapping[str, Any], *, _name: str = name
            ) -> str:
                logger.info("OperatorHarness stub executor: %s", _name)
                return "ok"

            provider.register(tool_spec, _stub_handler)


__all__ = [
    "OPERATOR_TOOL_NAMES",
    "OperatorHarness",
    "OperatorState",
]
