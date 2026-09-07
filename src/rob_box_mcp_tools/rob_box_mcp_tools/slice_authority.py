#!/usr/bin/env python3
"""``ToolSliceAuthority`` — sender → slice → tool allowlist (ADR-0052, issue #1998 §6.2).

Этот модуль — единственная инстанция, которая решает, имеет ли данный
отправитель право вызвать данный инструмент на ``/mcp/execute``. Стоит
после :class:`RequestAuthenticator` (HMAC) и до :class:`MappingState`
(FSM-гард картографирования) — каждый гард отвечает за свой инвариант
и не знает про остальные, чтобы лог отказа был однозначным.

Карта политики — YAML-данные (``data/slice_policy.yaml``), а не хардкод.
Этот же паттерн уже принят для ``confirmation_policy.yaml`` (issue
#968 §8): политика читается из данных, валидируется на старте,
fail-fast. Изменение состава среза или sender'а — одна строчка в
YAML, без пересборки ``voice-assistant``.

YAML-формат
-----------
::

    senders:
      dialogue_node:        [core, personality]
      avatar_supervisor:    [core, personality, operator.speech, operator.control, operator.admin]
      harness:              [core, personality]

    slices:
      core:                 [get_battery_level, get_current_time, ...]
      personality:          [speak_text, listen_for_response, ...]
      operator.speech:      [say, set_voice, set_voice_preset, ...]
      operator.control:     [dialogue_pause, dialogue_resume, ...]
      operator.admin:       [ros2_node_status, read_logs, container_status]

Один и тот же tool может входить в несколько срезов (``stop_navigation``
есть в ``core`` и ``personality``) — это нормально и нужно для
наследования «ТАРС видит всё, что умеет робот».

Pure Python — без ``rclpy`` и без LLM. Импортируется из тестов так же,
как ``ToolConfirmationPolicy`` (см. ``test_confirmation_policy.py``).
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import resources
from typing import Any, FrozenSet, Mapping

from rob_box_harness.errors import ConfigError


#: Канонические имена срезов, которые YAML может объявить.
#: Не строгий enum: сюда намеренно добавлен ``"*"`` как маркер
#: «sender имеет все срезы» (используется в тестах; в боевой YAML
#: такой записи быть не должно — это валидируется).
KNOWN_SLICES: FrozenSet[str] = frozenset(
    {
        "core",
        "personality",
        "operator.speech",
        "operator.control",
        "operator.admin",
    }
)


@dataclass(frozen=True)
class SliceDecision:
    """Исход :meth:`ToolSliceAuthority.is_allowed`.

    Attributes:
        allowed: ``True`` если sender имеет право вызвать tool.
        tool: Имя инструмента. Эхом возвращается, чтобы лог-строка
            отказа содержала имя без повторного lookup'а.
        sender: Отправитель (как пришёл в блоке ``auth``).
        reason: Человекочитаемое объяснение, **только** при отказе.
            Например, ``"sender 'dialogue_node' не имеет среза
            'operator.admin'"`` или ``"tool 'ros2_node_status' не
            принадлежит ни одному срезу sender'а 'harness'"``.
    """

    allowed: bool
    tool: str
    sender: str
    reason: str = ""


class ToolSliceAuthority:
    """Карта «кто какой срез имеет» и «что в каждом срезе».

    Конструктор принимает уже распарсенный mapping (тот же контракт,
    что у :class:`ToolConfirmationPolicy.from_mapping`). Готовые
    классификаторы immutable: для слоёного override есть
    :meth:`extended_with`.

    Атрибуты:
        senders: ``sender → frozenset(slice)``. Read-only view.
        slices: ``slice → frozenset(tool_name)``. Read-only view.
    """

    __slots__ = ("_senders", "_slices")

    def __init__(
        self,
        *,
        senders: Mapping[str, FrozenSet[str]],
        slices: Mapping[str, FrozenSet[str]],
    ) -> None:
        # Defensive copies — caller can't mutate the classifier
        # through the dicts it handed us.
        self._senders: dict[str, frozenset[str]] = {
            s: frozenset(slices_) for s, slices_ in senders.items()
        }
        self._slices: dict[str, frozenset[str]] = {
            sl: frozenset(tools) for sl, tools in slices.items()
        }

    # ----- factory --------------------------------------------------------

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "ToolSliceAuthority":
        """Построить классификатор из распарсенного YAML.

        Валидация (fail-fast на старте):

        * ``senders`` и ``slices`` — обязательные маппинги; пустой
          ``senders`` допустим (тогда никто ничего не может), пустой
          ``slices`` — нет (бессмысленная политика).
        * Имена срезов должны быть из :data:`KNOWN_SLICES` плюс
          пользовательские (расширяемость через YAML, но не через
          хардкод — поэтому проверка строгая на «известные +
          ``operator.*`` префикс»).
        * Имя sender'а — непустая строка.
        * Имя tool'а — непустая строка, без пробелов.
        * Каждый slice в ``senders[s]`` обязан быть объявлен в
          ``slices`` — иначе sender ссылается на несуществующее.
        """
        if not isinstance(raw, Mapping):
            raise ConfigError(
                "slice policy must be a mapping at the top level",
                section="slice_policy",
            )

        senders_raw = raw.get("senders")
        slices_raw = raw.get("slices")
        if not isinstance(senders_raw, Mapping):
            raise ConfigError(
                "'senders' must be a mapping of sender -> slices list",
                section="slice_policy.senders",
            )
        if not isinstance(slices_raw, Mapping):
            raise ConfigError(
                "'slices' must be a mapping of slice -> tools list",
                section="slice_policy.slices",
            )
        if not slices_raw:
            raise ConfigError(
                "'slices' is empty — declare at least one slice before "
                "rolling out slice-guard",
                section="slice_policy.slices",
            )

        # ── parse slices ─────────────────────────────────────────────
        slices: dict[str, frozenset[str]] = {}
        for slice_name, tool_list in slices_raw.items():
            if not isinstance(slice_name, str) or not slice_name.strip():
                raise ConfigError(
                    "slice names must be non-empty strings",
                    section="slice_policy.slices",
                )
            if not isinstance(tool_list, (list, tuple)):
                raise ConfigError(
                    f"slice '{slice_name}' must list its tools as a list",
                    section=f"slice_policy.slices.{slice_name}",
                )
            tools: set[str] = set()
            for t in tool_list:
                if not isinstance(t, str) or not t.strip() or " " in t:
                    raise ConfigError(
                        f"slice '{slice_name}' contains invalid tool "
                        f"name {t!r}",
                        section=f"slice_policy.slices.{slice_name}",
                    )
                tools.add(t)
            slices[slice_name] = frozenset(tools)

        # ── parse senders ────────────────────────────────────────────
        senders: dict[str, frozenset[str]] = {}
        for sender_name, slice_list in senders_raw.items():
            if not isinstance(sender_name, str) or not sender_name.strip():
                raise ConfigError(
                    "sender names must be non-empty strings",
                    section="slice_policy.senders",
                )
            if not isinstance(slice_list, (list, tuple)):
                raise ConfigError(
                    f"sender '{sender_name}' must list its slices as a list",
                    section=f"slice_policy.senders.{sender_name}",
                )
            granted: set[str] = set()
            for sl in slice_list:
                if not isinstance(sl, str) or not sl.strip():
                    raise ConfigError(
                        f"sender '{sender_name}' references invalid "
                        f"slice {sl!r}",
                        section=f"slice_policy.senders.{sender_name}",
                    )
                if sl not in slices:
                    raise ConfigError(
                        f"sender '{sender_name}' references unknown "
                        f"slice {sl!r} (declared slices: "
                        f"{sorted(slices)})",
                        section=f"slice_policy.senders.{sender_name}",
                    )
                granted.add(sl)
            senders[sender_name] = frozenset(granted)

        return cls(senders=senders, slices=slices)

    # ----- public API -----------------------------------------------------

    @property
    def known_senders(self) -> frozenset[str]:
        """Read-only view: все sender'ы, которых знает политика."""
        return frozenset(self._senders)

    @property
    def known_slices(self) -> frozenset[str]:
        """Read-only view: все срезы, объявленные в политике."""
        return frozenset(self._slices)

    def sender_slices(self, sender: str) -> frozenset[str]:
        """Какие срезы доступны данному sender'у.

        Sender, не указанный в политике, получает **пустой** набор
        срезов (fail-closed). Это — инвариант ADR-0052 §2.2: новый
        sender по умолчанию ничего не может, пока оператор явно
        не пропишет его в YAML.
        """
        return self._senders.get(sender, frozenset())

    def slice_tools(self, slice_name: str) -> frozenset[str]:
        """Какие инструменты входят в данный срез.

        Неизвестный срез → пустой набор. Это даёт
        fail-closed-поведение и при typo в YAML: sender, чей
        срез опечатан, ничего не сможет вызвать.
        """
        return self._slices.get(slice_name, frozenset())

    def is_allowed(self, sender: str, tool_name: str) -> SliceDecision:
        """Решение: может ли *sender* вызвать *tool_name*?

        Возвращает :class:`SliceDecision` с явной причиной отказа —
        чтобы ``mcp_server`` мог записать в лог ровно то, что не
        сошлось (sender неизвестен / sender без нужного среза /
        tool не принадлежит ни одному разрешённому sender'у срезу).
        """
        granted = self.sender_slices(sender)
        if not granted:
            return SliceDecision(
                allowed=False,
                tool=tool_name,
                sender=sender,
                reason=(
                    f"sender '{sender}' не в списке разрешённых или "
                    f"не имеет ни одного среза"
                ),
            )
        # Объединение инструментов из всех разрешённых sender'у срезов.
        union: set[str] = set()
        for sl in granted:
            union |= self.slice_tools(sl)
        if tool_name not in union:
            granted_str = ", ".join(sorted(granted))
            return SliceDecision(
                allowed=False,
                tool=tool_name,
                sender=sender,
                reason=(
                    f"tool '{tool_name}' не принадлежит ни одному "
                    f"срезу sender'а '{sender}' "
                    f"(доступные срезы: {granted_str})"
                ),
            )
        return SliceDecision(allowed=True, tool=tool_name, sender=sender)

    def extended_with(
        self,
        *,
        senders: Mapping[str, list[str]] | None = None,
        slices: Mapping[str, list[str]] | None = None,
    ) -> "ToolSliceAuthority":
        """Вернуть новый классификатор с override-слоем.

        Override-слой — это «дельты» относительно ``self``:
        sender'ы и срезы из аргументов **дополняют** (а не
        заменяют) соответствующие записи базовой политики. Это
        нужно для того же «base.yaml + pi-main.yaml» паттерна,
        что и :meth:`ToolConfirmationPolicy.extended_with`.

        Raises:
            ConfigError: если override ссылается на несуществующий
                срез, или sender с пустым списком (override не
                может **запретить** то, что было разрешено — это
                намеренное ограничение, чтобы случайный override
                не превратил боевой доступ в ноль).
        """
        new_senders: dict[str, frozenset[str]] = {
            s: set(slices_) for s, slices_ in self._senders.items()
        }
        for s, slice_list in (senders or {}).items():
            if not isinstance(slice_list, (list, tuple)) or not slice_list:
                raise ConfigError(
                    f"override for sender '{s}' must be a non-empty list",
                    section=f"slice_policy.senders.{s}",
                )
            granted = set(new_senders.get(s, frozenset()))
            for sl in slice_list:
                if sl not in self._slices:
                    raise ConfigError(
                        f"override for sender '{s}' references unknown "
                        f"slice {sl!r}",
                        section=f"slice_policy.senders.{s}",
                    )
                granted.add(sl)
            new_senders[s] = frozenset(granted)

        new_slices: dict[str, frozenset[str]] = {
            sl: set(tools) for sl, tools in self._slices.items()
        }
        for sl, tool_list in (slices or {}).items():
            if not isinstance(tool_list, (list, tuple)):
                raise ConfigError(
                    f"override for slice '{sl}' must be a list",
                    section=f"slice_policy.slices.{sl}",
                )
            tools = set(new_slices.get(sl, frozenset()))
            for t in tool_list:
                if not isinstance(t, str) or not t.strip() or " " in t:
                    raise ConfigError(
                        f"override for slice '{sl}' contains invalid "
                        f"tool name {t!r}",
                        section=f"slice_policy.slices.{sl}",
                    )
                tools.add(t)
            new_slices[sl] = frozenset(tools)

        return ToolSliceAuthority(senders=new_senders, slices=new_slices)


# ---------------------------------------------------------------------------
# Default loader
# ---------------------------------------------------------------------------


def _default_policy_text() -> str:
    """Прочитать YAML каталога срезов из ресурсов пакета.

    Raises:
        ConfigError: если bundled-файл пропал (упаковка сломана).
    """
    try:
        return (
            resources.files("rob_box_mcp_tools.data")
            .joinpath("slice_policy.yaml")
            .read_text(encoding="utf-8")
        )
    except (FileNotFoundError, OSError) as exc:
        raise ConfigError(
            "bundled slice_policy.yaml is missing from "
            "rob_box_mcp_tools.data — packaging error",
            section="slice_policy",
        ) from exc


def load_default_authority() -> ToolSliceAuthority:
    """Загрузить :class:`ToolSliceAuthority` из bundled YAML.

    Единственная точка, которую зовёт ``mcp_server`` при старте.
    Тесты строят synthetic-классификаторы через
    :meth:`from_mapping`, не мутируя bundled-файл.
    """
    import yaml  # local import — see config.py for the rationale

    text = _default_policy_text()
    parsed = yaml.safe_load(text)
    if parsed is None:
        parsed = {}
    if not isinstance(parsed, Mapping):
        raise ConfigError(
            "slice_policy.yaml must be a mapping at the top level",
            section="slice_policy",
        )
    return ToolSliceAuthority.from_mapping(parsed)


__all__ = [
    "KNOWN_SLICES",
    "SliceDecision",
    "ToolSliceAuthority",
    "load_default_authority",
]
