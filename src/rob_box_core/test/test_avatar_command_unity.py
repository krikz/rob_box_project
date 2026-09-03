"""test_unity_of_behavior.py — AV-22 acceptance #3, главный артефакт карточки.

Тест единства поведения «мотивируй народ» из Quest и из Telegram → одинаковая
последовательность tool-call'ов (worker-brief §3.3, Issue #1914, OpenSpec
``supervisor-agent``). Без этого теста «единый вход» — слова.

Контракт:

1. Один и тот же текст команды (``"мотивируй народ"``) подаётся в оба
   producer'а — ``publish_avatar_command_from_quest_via`` и
   ``publish_avatar_command_via``.
2. Mock-supervisor-agent подписан на ``/avatar/command`` (через общий
   pub/sub, в тесте — общий список) и публикует детерминированный
   ответ в ``/avatar/command_result`` через тот же контракт
   ``build_command_result``.
3. Mock-LLM возвращает **фиксированную** последовательность tool-call'ов
   вне зависимости от ``source`` — это и есть «единство поведения».
4. В тесте проверяется, что обе стороны получили от mock-агента
   одинаковый ``tool_calls`` с одинаковым ``request_id``-коррелятором.

Не требует rclpy — общий список ``captured`` играет роль pub/sub.
"""

from __future__ import annotations

from typing import Any

from rob_box_core.avatar_command import (
    AVATAR_COMMAND_RESULT_TOPIC,
    AVATAR_COMMAND_TOPIC,
    decode_command,
    decode_command_result,
    publish_avatar_command_from_quest_via,
    publish_avatar_command_via,
)


# ─── Mock-инфраструктура ─────────────────────────────────────────────────


class _Channel:
    """Минимальный in-memory pub/sub для unit-тестов.

    Topic → список подписчиков. Каждый подписчик — callable(msg: str).
    """

    def __init__(self):
        self._subs: dict[str, list[Any]] = {}

    def subscribe(self, topic: str, cb) -> None:
        self._subs.setdefault(topic, []).append(cb)

    def publish(self, topic: str, payload: str) -> None:
        for cb in list(self._subs.get(topic, [])):
            cb(payload)


class _MockSupervisorAgent:
    """Mock супервизор-агента (AV-21) для теста единства.

    Подписан на ``/avatar/command``, публикует в ``/avatar/command_result``
    детерминированный ответ: ``tool_calls`` зависит ТОЛЬКО от
    содержимого ``text``, не от ``source``. Это и есть «единство».

    Mock-LLM (отдельный компонент в реальной системе) здесь не нужен —
    «единство» проверяется на уровне «агент видит одинаковый input →
    решает сделать одинаковые tool-call'ы».
    """

    def __init__(self, channel: _Channel, *, tool_catalog: dict[str, list[str]]):
        self._channel = channel
        self._catalog = tool_catalog
        # История для assert: что пришло в /avatar/command.
        self.received_commands: list[dict] = []
        # История исходящих результатов (для assert в тесте).
        self.published_results: list[dict] = []
        self._channel.subscribe(AVATAR_COMMAND_TOPIC, self._on_command)

    def _on_command(self, raw: str) -> None:
        cmd = decode_command(raw)
        self.received_commands.append(cmd)
        tool_calls = self._catalog.get(cmd["text"], ["noop"])
        from rob_box_core.avatar_command import build_command_result

        result = build_command_result(
            request_id=cmd["request_id"],
            ok=True,
            summary=f"executed: {cmd['text']!r}",
            tool_calls=tool_calls,
        )
        from rob_box_core.avatar_command import encode_command_result

        wire = encode_command_result(result)
        self.published_results.append(result)
        self._channel.publish(AVATAR_COMMAND_RESULT_TOPIC, wire)


# ─── Сами тесты ──────────────────────────────────────────────────────────


def test_unity_quest_and_telegram_produce_same_tool_calls():
    """AC: одна фраза из quest и из telegram → одинаковые tool-call'ы.

    Это «главный артефакт» карточки (Issue #1914, §3 acceptance).
    """
    channel = _Channel()

    # Mock-каталог: text → tool_calls. Один и тот же для обеих сторон.
    catalog = {
        "мотивируй народ": ["play_track", "set_volume"],
        "поехали": ["navigate_to_waypoint"],
    }
    agent = _MockSupervisorAgent(channel, tool_catalog=catalog)

    # Подписчик на результаты (отдельный, чтобы не зациклиться).
    results = []

    def collect_result(raw: str):
        results.append(decode_command_result(raw))

    channel.subscribe(AVATAR_COMMAND_RESULT_TOPIC, collect_result)

    # ── 1. Quest-сторона публикует ──────────────────────────────────────
    class _Pub:
        def __init__(self, ch, topic):
            self.ch = ch
            self.topic = topic

        def publish(self, msg):
            self.ch.publish(self.topic, msg.data)

    quest_pub = _Pub(channel, AVATAR_COMMAND_TOPIC)
    quest_request_id = publish_avatar_command_from_quest_via(
        quest_pub,
        text="мотивируй народ",
        session_id="sess-quest",
    )

    # ── 2. Telegram-сторона публикует ту же фразу ───────────────────────
    telegram_pub = _Pub(channel, AVATAR_COMMAND_TOPIC)
    telegram_request_id = publish_avatar_command_via(
        telegram_pub,
        text="мотивируй народ",
        chat_id=42,
    )

    # ── 3. Агент получил ОБА команды ────────────────────────────────────
    assert (
        len(agent.received_commands) == 2
    ), f"ожидалось 2 команды, пришло {len(agent.received_commands)}"

    # ── 4. Оба результата пришли, оба с одинаковыми tool_calls ──────────
    assert len(results) == 2
    assert results[0]["tool_calls"] == ["play_track", "set_volume"]
    assert results[1]["tool_calls"] == ["play_track", "set_volume"]
    assert (
        results[0]["tool_calls"] == results[1]["tool_calls"]
    ), "единство поведения нарушено: tool_calls разные для одной фразы"

    # ── 5. request_id коррелирует (агент пробрасывает request_id) ───────
    assert results[0]["request_id"] == quest_request_id
    assert results[1]["request_id"] == telegram_request_id
    assert quest_request_id != telegram_request_id


def test_unity_different_text_different_tool_calls():
    """Контр-тест: разные фразы → разные tool-calls."""
    channel = _Channel()
    catalog = {
        "мотивируй народ": ["play_track", "set_volume"],
        "стоп": ["stop_music"],
    }
    agent = _MockSupervisorAgent(channel, tool_catalog=catalog)
    results = []

    def collect_result(raw: str):
        results.append(decode_command_result(raw))

    channel.subscribe(AVATAR_COMMAND_RESULT_TOPIC, collect_result)

    class _Pub:
        def __init__(self, ch, topic):
            self.ch = ch
            self.topic = topic

        def publish(self, msg):
            self.ch.publish(self.topic, msg.data)

    quest_pub = _Pub(channel, AVATAR_COMMAND_TOPIC)
    publish_avatar_command_from_quest_via(
        quest_pub,
        text="мотивируй народ",
        session_id="s1",
    )
    publish_avatar_command_from_quest_via(
        quest_pub,
        text="стоп",
        session_id="s1",
    )

    assert len(results) == 2
    assert results[0]["tool_calls"] == ["play_track", "set_volume"]
    assert results[1]["tool_calls"] == ["stop_music"]


def test_request_id_correlates_command_with_result():
    """AC: ответ агента приходит по ``request_id`` (Issue #1914, §3 acceptance)."""
    channel = _Channel()
    catalog = {"test": ["a", "b", "c"]}
    agent = _MockSupervisorAgent(channel, tool_catalog=catalog)
    results: list[dict] = []

    def collect_result(raw: str):
        results.append(decode_command_result(raw))

    channel.subscribe(AVATAR_COMMAND_RESULT_TOPIC, collect_result)

    class _Pub:
        def __init__(self, ch, topic):
            self.ch = ch
            self.topic = topic

        def publish(self, msg):
            self.ch.publish(self.topic, msg.data)

    quest_pub = _Pub(channel, AVATAR_COMMAND_TOPIC)
    rid = publish_avatar_command_from_quest_via(
        quest_pub,
        text="test",
        session_id="s",
    )

    assert len(results) == 1
    assert results[0]["request_id"] == rid


def test_telegram_and_quest_have_different_client_id_but_same_payload():
    """AC: ``source`` и ``client_id`` разные (server-side формируются), но
    ``text`` одинаковый → одинаковая tool-calls."""
    channel = _Channel()
    catalog = {"go": ["navigate"]}
    agent = _MockSupervisorAgent(channel, tool_catalog=catalog)
    results: list[dict] = []

    def collect_result(raw: str):
        results.append(decode_command_result(raw))

    channel.subscribe(AVATAR_COMMAND_RESULT_TOPIC, collect_result)

    class _Pub:
        def __init__(self, ch, topic):
            self.ch = ch
            self.topic = topic

        def publish(self, msg):
            self.ch.publish(self.topic, msg.data)

    pub = _Pub(channel, AVATAR_COMMAND_TOPIC)
    publish_avatar_command_from_quest_via(pub, text="go", session_id="sess-1")
    publish_avatar_command_via(pub, text="go", chat_id=999)

    assert len(agent.received_commands) == 2
    quest_cmd, telegram_cmd = agent.received_commands
    assert quest_cmd["source"] == "quest"
    assert telegram_cmd["source"] == "telegram"
    assert quest_cmd["client_id"] != telegram_cmd["client_id"]
    assert quest_cmd["text"] == telegram_cmd["text"] == "go"
    # И оба результата — одинаковые.
    assert results[0]["tool_calls"] == results[1]["tool_calls"] == ["navigate"]
