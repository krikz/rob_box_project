"""Issue #2132 acceptance test — sender адаптера берётся из node.get_name().

Фон
----
До #2132 ``LLMToolCallAdapter.__init__`` был жёстко зашит на
``sender="dialogue_node"``. Это означало, что ``avatar_supervisor``
(ТАРС) подписывал свои ``/mcp/execute`` запросы как ``dialogue_node``.
На стороне ``mcp_server`` это проходило auth-гард (HMAC валиден), но
срез-гард применял срезы *личности* (``core`` + ``personality``) к
запросам, у которых фактический отправитель — оператор. Результат:
все операторские инструменты (``say``, ``dialogue_pause``,
``ros2_node_status`` и т.д.) резались «🛇 Slice blocked 'say' для
sender='dialogue_node'».

Что фиксирует этот файл
------------------------

1. ``DEFAULT_ALLOWED_SENDERS`` теперь явно содержит
   ``avatar_supervisor`` (раньше там были только ``dialogue_node`` и
   ``harness`` — это отрезало бы подпись на этапе verify, ДО среза).
2. ``LLMToolCallAdapter`` подписывает запрос именем, которое вернул
   ``node.get_name()``, а не жёстким литералом.
3. Парный happy path: подписанный ``avatar_supervisor``-ом запрос
   проходит ``verify()`` с дефолтным ``allowed_senders``.

Тест НЕ требует ``rclpy`` (адаптер тянет его на импорте) — мы
подменяем ``rclpy.*`` через ``types.ModuleType`` в ``sys.modules``,
точно как в существующем ``test_mcp_server.py``. Это локально
запускаемо и не зависит от наличия ROS2 в окружении разработчика.
"""

from __future__ import annotations

import importlib
import json
import sys
import time
import types
from typing import Any, Dict, List, Optional, Tuple

import pytest


# ---------------------------------------------------------------------------
# Stub rclpy (адаптер импортирует rclpy, rclpy.node, rclpy.qos, rclpy.callback_groups)
# ---------------------------------------------------------------------------


class _FakeNode:
    """Минимум API, который ``LLMToolCallAdapter`` зовёт из ROS2-ноды.

    Имя ноды задаётся через ``name`` — адаптер читает его через
    ``get_name()`` (см. инициализатор), и это имя становится ``sender``
    HMAC-подписи.
    """

    def __init__(self, name: str) -> None:
        self._name = name
        self._logger = _FakeLogger()
        self._publishers: Dict[str, "_FakePublisher"] = {}
        self._subscriptions: List[Any] = []

    def get_name(self) -> str:
        return self._name

    def get_logger(self) -> "_FakeLogger":
        return self._logger

    def create_publisher(self, msg_type, topic: str, qos):
        pub = _FakePublisher(topic)
        self._publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic: str, callback, qos, callback_group=None):
        self._subscriptions.append((topic, callback))
        return object()


class _FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.published: List[str] = []

    def publish(self, msg) -> None:
        # ``msg.data`` — JSON-строка; нам важно уметь её парсить,
        # чтобы прочитать ``auth.sender`` в тестах ниже.
        self.published.append(msg.data)


class _FakeLogger:
    def __init__(self) -> None:
        self.infos: List[str] = []
        self.warns: List[str] = []
        self.errors: List[str] = []

    def info(self, msg: str) -> None:
        self.infos.append(msg)

    def warning(self, msg: str) -> None:
        self.warns.append(msg)

    def error(self, msg: str) -> None:
        self.errors.append(msg)


def _install_fake_rclpy() -> None:
    """Подсунуть rclpy-стабы в ``sys.modules`` ДО импорта ``llm_adapter``.

    Это позволяет тесту жить без установленного ROS2. Возвращаемое
    значение — нет, эффект — глобальный.
    """
    if "rclpy" in sys.modules and getattr(
        sys.modules["rclpy"], "_is_rob_box_test_stub", False
    ):
        return  # уже подменено

    class _StubQoSProfile:
        """Реальный ``QoSProfile`` принимает kwargs (``reliability=...``).

        Stub-класс делает то же, иначе ``llm_adapter.__init__`` падает
        с ``TypeError: QoSProfile() takes no arguments``.
        """

        def __init__(self, *args, **kwargs) -> None:
            self._args = args
            self._kwargs = kwargs

    class _StubReentrantCallbackGroup:
        def __init__(self) -> None:
            pass

    class _StubStringMsg:
        """``std_msgs/String`` — обёртка с полем ``data``."""

        def __init__(self, data: str = "") -> None:
            self.data = data

    rclpy = types.ModuleType("rclpy")
    rclpy._is_rob_box_test_stub = True
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = _FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = _StubQoSProfile
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_cb = types.ModuleType("rclpy.callback_groups")
    rclpy_cb.ReentrantCallbackGroup = _StubReentrantCallbackGroup

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = _StubStringMsg

    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos
    sys.modules["rclpy.callback_groups"] = rclpy_cb
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg


def _import_llm_adapter():
    """(Re)import ``llm_adapter`` со свежим stub-rclpy.

    При первом вызове подменяем ``rclpy`` в ``sys.modules``, после
    этого ``importlib.import_module`` поднимет уже подменённый
    ``llm_adapter``. Если адаптер уже загружен из другого теста —
    сбрасываем и берём свежую копию.
    """
    _install_fake_rclpy()
    sys.modules.pop("rob_box_mcp_tools.llm_adapter", None)
    return importlib.import_module("rob_box_mcp_tools.llm_adapter")


def _import_mcp_auth():
    """(Re)import ``mcp_auth`` чисто (он не тянет rclpy)."""
    sys.modules.pop("rob_box_mcp_tools.mcp_auth", None)
    return importlib.import_module("rob_box_mcp_tools.mcp_auth")


# ---------------------------------------------------------------------------
# Acceptance: DEFAULT_ALLOWED_SENDERS содержит avatar_supervisor
# ---------------------------------------------------------------------------


class TestAllowedSenders:
    """Без #2132 ``DEFAULT_ALLOWED_SENDERS`` = ``{dialogue_node, harness}``.

    Подпись ``avatar_supervisor`` отклонялась на этапе
    ``sender not in self._allowed_senders`` — ДО среза. Сейчас
    ``avatar_supervisor`` обязан быть в whitelist.
    """

    def test_avatar_supervisor_in_default_allowed_senders(self) -> None:
        mcp_auth = _import_mcp_auth()
        assert "avatar_supervisor" in mcp_auth.DEFAULT_ALLOWED_SENDERS, (
            "DoD #2132: avatar_supervisor обязан быть в whitelist "
            "DEFAULT_ALLOWED_SENDERS, иначе HMAC.verify() отвергнет "
            "его подпись ещё ДО slice-гарда"
        )

    def test_dialogue_node_still_in_default_allowed_senders(self) -> None:
        """Защита от регрессии: предыдущие senders не должны пропасть."""
        mcp_auth = _import_mcp_auth()
        assert "dialogue_node" in mcp_auth.DEFAULT_ALLOWED_SENDERS
        assert "harness" in mcp_auth.DEFAULT_ALLOWED_SENDERS

    def test_unknown_sender_still_rejected(self, tmp_path, monkeypatch) -> None:
        """Не переборщить: чужие имена по-прежнему отклоняются.

        Базовый guard: ``mcp_server`` с дефолтным ``allowed_senders``
        НЕ должен принимать запрос от случайного имени.
        """
        mcp_auth = _import_mcp_auth()
        token_file = str(tmp_path / ".mcp_token")
        # Локальный секрет — иначе verify() жалуется на missing token.
        token_file_path = type("_P", (), {})()
        monkeypatch.setenv(mcp_auth.ENV_TOKEN, "test-token-1234")
        monkeypatch.setenv(mcp_auth.ENV_TOKEN_FILE, token_file)
        monkeypatch.delenv(mcp_auth.ENV_ALLOW_UNAUTH, raising=False)

        sender = mcp_auth.RequestAuthenticator.from_env(sender="random_attacker")
        server = mcp_auth.RequestAuthenticator.from_env()
        signed = sender.sign(
            {"tool_name": "say", "parameters": {}, "request_id": "x"}
        )
        ok, err = server.verify(signed)
        assert ok is False
        assert "не в списке разрешённых" in err


# ---------------------------------------------------------------------------
# Acceptance: sender адаптера берётся из node.get_name()
# ---------------------------------------------------------------------------


class TestAdapterSenderFromNodeName:
    """Главный DoD-тест карточки #2132.

    До фикса здесь был жёсткий литерал ``sender="dialogue_node"`` —
    проверка через node.get_name() невозможна (тест бы прошёл и
    ДО, и ПОСЛЕ фикса). Поэтому мы смотрим на **результат подписи**:
    sender, который попадает в HMAC ``auth.sender``, должен
    совпадать с ``node.get_name()``.
    """

    def test_dialogue_node_signs_as_dialogue_node(self) -> None:
        llm = _import_llm_adapter()
        mcp_auth = _import_mcp_auth()

        # Локальный секрет, чтобы подпись реально добавлялась.
        import os
        os.environ[mcp_auth.ENV_TOKEN] = "shared-test-secret"
        os.environ.pop(mcp_auth.ENV_TOKEN_FILE, None)
        os.environ.pop(mcp_auth.ENV_ALLOW_UNAUTH, None)

        node = _FakeNode("dialogue_node")
        adapter = llm.LLMToolCallAdapter(node)
        try:
            request_id = adapter.execute_tool_call("say", {"text": "тест"})
            published = node._publishers["/mcp/execute"].published
            assert published, "адаптер не опубликовал /mcp/execute"
            payload = json.loads(published[-1])
            assert payload["auth"]["sender"] == "dialogue_node", (
                f"sender должен браться из node.get_name(); "
                f"получили {payload['auth']['sender']!r}"
            )
            assert payload["request_id"] == request_id
        finally:
            os.environ.pop(mcp_auth.ENV_TOKEN, None)

    def test_avatar_supervisor_signs_as_avatar_supervisor(self) -> None:
        """Главный DoD #2132: ``avatar_supervisor`` подписывает как сам себя.

        До фикса здесь возвращалось ``dialogue_node`` — и эта подпись
        проходила auth-гард (HMAC валиден), но срезы применялись от
        личности, что и ломало ТАРС.
        """
        llm = _import_llm_adapter()
        mcp_auth = _import_mcp_auth()

        import os
        os.environ[mcp_auth.ENV_TOKEN] = "shared-test-secret"
        os.environ.pop(mcp_auth.ENV_TOKEN_FILE, None)
        os.environ.pop(mcp_auth.ENV_ALLOW_UNAUTH, None)

        node = _FakeNode("avatar_supervisor")
        adapter = llm.LLMToolCallAdapter(node)
        try:
            adapter.execute_tool_call("say", {"text": "проверка среза"})

            published = node._publishers["/mcp/execute"].published
            assert published
            payload = json.loads(published[-1])
            assert payload["auth"]["sender"] == "avatar_supervisor", (
                "DoD #2132: avatar_supervisor-нода ОБЯЗАНА подписывать "
                "запросы как 'avatar_supervisor', иначе срезы будут "
                "применяться от личности и все operator.*-тулы "
                "зарежутся на транспорте. "
                f"Получили sender={payload['auth']['sender']!r}"
            )
        finally:
            os.environ.pop(mcp_auth.ENV_TOKEN, None)

    def test_explicit_sender_override(self) -> None:
        """Явный ``sender=`` через kwarg должен побеждать ``node.get_name()``.

        Это нужно для тестов и редких форов, где имя ROS-ноды и имя
        подписчика должны различаться.
        """
        llm = _import_llm_adapter()
        mcp_auth = _import_mcp_auth()

        import os
        os.environ[mcp_auth.ENV_TOKEN] = "shared-test-secret"
        os.environ.pop(mcp_auth.ENV_TOKEN_FILE, None)
        os.environ.pop(mcp_auth.ENV_ALLOW_UNAUTH, None)

        node = _FakeNode("some_test_ros_node")
        adapter = llm.LLMToolCallAdapter(node, sender="dialogue_node")
        try:
            adapter.execute_tool_call("get_battery_level", {})
            published = node._publishers["/mcp/execute"].published
            assert published
            payload = json.loads(published[-1])
            assert payload["auth"]["sender"] == "dialogue_node", (
                "явный sender= kwarg должен побеждать node.get_name()"
            )
        finally:
            os.environ.pop(mcp_auth.ENV_TOKEN, None)

    def test_avatar_supervisor_signature_is_accepted_by_default_server(
        self,
        tmp_path,
        monkeypatch,
    ) -> None:
        """End-to-end: подпись avatar_supervisor-а принимается сервером.

        Склеиваем предыдущие два теста: конструируем адаптер от
        ``avatar_supervisor``-ноды, берём то, что он подписал и
        публикует, и прогоняем через ``RequestAuthenticator`` с
        дефолтным ``allowed_senders``. До #2132 это падало с
        «отправитель 'avatar_supervisor' не в списке разрешённых».
        """
        llm = _import_llm_adapter()
        mcp_auth = _import_mcp_auth()

        token = "shared-test-secret-end-to-end"
        # Файл секрета вместо ENV — тест не должен зависеть от
        # глобального ENV-состояния других тестов.
        token_file = str(tmp_path / ".mcp_token")
        monkeypatch.setenv(mcp_auth.ENV_TOKEN, token)
        monkeypatch.setenv(mcp_auth.ENV_TOKEN_FILE, token_file)
        monkeypatch.delenv(mcp_auth.ENV_ALLOW_UNAUTH, raising=False)

        node = _FakeNode("avatar_supervisor")
        adapter = llm.LLMToolCallAdapter(node)
        adapter.execute_tool_call("say", {"text": "привет"})

        published = node._publishers["/mcp/execute"].published[-1]
        payload = json.loads(published)

        # «Серверная» сторона — дефолтный whitelist.
        server = mcp_auth.RequestAuthenticator.from_env()
        ok, err = server.verify(payload)
        assert ok is True, (
            f"DoD #2132: подпись avatar_supervisor-а ОБЯЗАНА "
            f"проходить verify() с дефолтным allowed_senders. "
            f"Получили err={err!r}"
        )
