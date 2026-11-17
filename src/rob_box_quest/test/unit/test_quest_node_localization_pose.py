"""Unit-тесты фикса issue #2618: замена tf2_ros.Buffer+TransformListener
на лёгкую подписку ``/rtabmap/localization_pose``.

Контракт:

* ``_on_localization_pose`` обновляет ``_latest_map_pose`` (x, y, yaw, ts)
  из ``PoseWithCovarianceStamped``-сообщения (``msg.pose.pose``);
* ``_map_pose`` возвращает ``None`` пока ни одной локализации не пришло;
* ``_map_pose`` после локализации возвращает ``(x, y, yaw)``-tuple
  с правильно посчитанным yaw (кватернион → atan2);
* кватернион с нулевой мнимой частью → ``yaw == 0``;
* кватернион «поворот на 90° вокруг z» → ``yaw == π/2``;
* malformed-сообщение (атрибут ``pose`` не iterable / пустой) не валит
  callback и НЕ затирает предыдущий снимок;
* ``_on_localization_pose`` НЕ трогает tf (нет ``lookup_transform``,
  нет подписки на ``/tf``/``/tf_static``) — это центральный пункт
  acceptance #2618: убираем busy-loop в покое.

Запуск:
    PYTHONPATH=src/rob_box_quest:src/rob_box_core:src/rob_box_harness \\
        pytest src/rob_box_quest/test/unit/test_quest_node_localization_pose.py -v
"""

from __future__ import annotations

import math
from unittest.mock import MagicMock

import pytest

from conftest import quest_node_mod  # noqa: F401  — фикстура ставит ROS-stub


def _make_pose_stamped(
    *,
    x: float = 1.5,
    y: float = -2.25,
    qw: float = 1.0,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
):
    """Минимальный PoseWithCovarianceStamped-эквивалент для ``_on_localization_pose``.

    Внутри callback-а используются:
    * ``msg.pose.pose.position.x/y/z``
    * ``msg.pose.pose.orientation.{w,x,y,z}``

    Остальное (header, frame_id) callback-у не нужно — поэтому делаем
    MagicMock с минимально нужными атрибутами.
    """
    msg = MagicMock()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = qw
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    return msg


def _make_host():
    """Минимальный host (как и в test_quest_node_avatar.py): все
    поля ``self`` в callback-е либо None, либо MagicMock.

    В ``_on_localization_pose`` используются:
    * ``self._latest_map_pose`` (запись)
    * ``self.get_logger().debug(...)`` — MagicMock-логгер поглощает.
    * никаких ``self._tf_buffer`` / ``self._tf_listener`` — это и есть
      центральное утверждение теста.
    """
    host = MagicMock()
    host._latest_map_pose = None
    return host


# ── _map_pose: до первой локализации → None ───────────────────────────────


def test_map_pose_returns_none_before_any_localization(quest_node_mod):
    """Без локализации ``_map_pose`` не должен ничего возвращать."""
    host = _make_host()
    result = quest_node_mod.QuestNode._map_pose(host)
    assert result is None


# ── _on_localization_pose: базовый сценарий ────────────────────────────────


def test_on_localization_pose_stores_xy_yaw_snapshot(quest_node_mod):
    """После одного callback-а ``_latest_map_pose`` = (x, y, yaw, ts).

    yaw вычисляется через ``atan2`` — для нулевого кватерниона должен
    получиться 0.0.
    """
    host = _make_host()
    msg = _make_pose_stamped(x=1.5, y=-2.25, qw=1.0)

    quest_node_mod.QuestNode._on_localization_pose(host, msg)

    snap = host._latest_map_pose
    assert snap is not None
    assert len(snap) == 4
    x, y, yaw, ts = snap
    assert x == pytest.approx(1.5)
    assert y == pytest.approx(-2.25)
    assert yaw == pytest.approx(0.0, abs=1e-9)
    # ts — монотонное время; просто проверим, что оно float и >= 0.
    assert isinstance(ts, float)
    assert ts >= 0.0


def test_on_localization_pose_computes_yaw_from_quaternion(quest_node_mod):
    """Кватернион «90° вокруг z» → yaw = π/2.

    Кватернион поворота на угол θ вокруг оси z:
        qw = cos(θ/2), qz = sin(θ/2), qx = qy = 0
    Для θ = π/2: qw = qz = √2/2 ≈ 0.7071.

    atan2 в коде: ``2 * (q.w*q.z + q.x*q.y)`` / ``1 - 2*(q.y² + q.z²)``
    Для qw=qz=√2/2:
        числитель = 2 * (0.5 + 0) = 1
        знаменатель = 1 - 2 * (0 + 0.5) = 0
    → atan2(1, 0) = π/2 ✓
    """
    host = _make_host()
    s = math.sqrt(2) / 2
    msg = _make_pose_stamped(x=0.0, y=0.0, qw=s, qz=s)

    quest_node_mod.QuestNode._on_localization_pose(host, msg)

    snap = host._latest_map_pose
    assert snap is not None
    yaw = snap[2]
    assert yaw == pytest.approx(math.pi / 2, abs=1e-9)


def test_map_pose_returns_3tuple_after_localization(quest_node_mod):
    """После локализации ``_map_pose`` отдаёт ровно ``(x, y, yaw)`` —
    3-tuple, без ``ts``. ``publish_map_pose`` подписан на этот контракт.
    """
    host = _make_host()
    msg = _make_pose_stamped(x=3.0, y=4.0, qw=1.0)

    quest_node_mod.QuestNode._on_localization_pose(host, msg)

    result = quest_node_mod.QuestNode._map_pose(host)
    assert result is not None
    assert isinstance(result, tuple)
    assert len(result) == 3
    assert result[0] == pytest.approx(3.0)
    assert result[1] == pytest.approx(4.0)
    assert result[2] == pytest.approx(0.0, abs=1e-9)


# ── malformed msg: callback не должен валиться / затирать снимок ──────────


def test_on_localization_pose_malformed_msg_keeps_previous_snapshot(quest_node_mod):
    """Если rtabmap прислал кривое сообщение (нет атрибута ``pose``),
    callback глотает исключение и НЕ затирает предыдущий снимок.

    Это поведение продиктовано тем, что rclpy subscription-callback
    исключение НЕ пробрасывает наверх (исполнитель молча логирует и
    идёт дальше), но мы дополнительно фиксируем в коде try/except
    чтобы и self.get_logger().debug(...) сработал, и снимок остался.
    """
    host = _make_host()
    # Сначала кладём валидный снимок.
    good = _make_pose_stamped(x=10.0, y=20.0, qw=1.0)
    quest_node_mod.QuestNode._on_localization_pose(host, good)
    snapshot_before = host._latest_map_pose
    assert snapshot_before is not None

    # Теперь «плохое» сообщение: ``pose`` — MagicMock, у которого
    # ``position`` при первом обращении к ``.x`` падает с AttributeError.
    bad = MagicMock()
    bad.pose.pose.position.x = property(  # type: ignore[assignment]
        lambda _self: (_ for _ in ()).throw(AttributeError("simulated malformed msg"))
    )

    # Не должно валить тест.
    quest_node_mod.QuestNode._on_localization_pose(host, bad)

    # Снимок должен остаться прежним.
    assert host._latest_map_pose == snapshot_before


# ── контракт acceptance #2618: никаких tf2 ─────────────────────────────────


def test_no_tf2_state_on_node_after_fix(quest_node_mod):
    """После фикса quest_node не должен иметь активного tf-стэка.

    Проверяем, что ``_tf_buffer`` и ``_tf_listener`` явно выставлены
    в ``None`` (см. fix в quest_node.py:2104-2105). Это и есть
    центральное отличие от старого кода с
    ``Buffer() + TransformListener(...)``.
    """
    # Читаем исходник и убеждаемся, что в конструкторе оба поля = None.
    src_path = quest_node_mod.__file__
    with open(src_path, encoding="utf-8") as fh:
        text = fh.read()

    # Конкретный фрагмент из fix-а (issue #2618).
    assert "self._tf_buffer = None" in text, (
        "fix regressed: tf_buffer не должен инициализироваться как Buffer() "
        "(issue #2618 — это и есть busy-loop в покое)"
    )
    assert "self._tf_listener = None" in text, (
        "fix regressed: tf_listener не должен создаваться "
        "(TransformListener подписывается на /tf + /tf_static и пересобирает "
        "WaitSet на каждом кадре 15–100 Гц)"
    )
    # И одновременно — НЕ должно быть прежней инициализации через tf2_ros.
    assert "from tf2_ros import Buffer, TransformListener" not in text, (
        "fix regressed: tf2_ros.Buffer+TransformListener убраны в fix #2618"
    )


def test_localization_pose_subscription_uses_correct_topic(quest_node_mod):
    """Фикс подписывается на ``/rtabmap/localization_pose`` (PoseWithCovarianceStamped).

    Альтернативы (типа ``/odom`` или ``/pose``) дают позу в ``odom``-фрейме
    или вообще отсутствуют на роботе — поэтому выбран именно
    ``/rtabmap/localization_pose``: там уже ``map → base_link`` в одном
    сообщении, без tf-цепочки.
    """
    src_path = quest_node_mod.__file__
    with open(src_path, encoding="utf-8") as fh:
        text = fh.read()
    assert "/rtabmap/localization_pose" in text, (
        "fix regressed: подписка на /rtabmap/localization_pose обязательна "
        "для map_2d-позы (issue #2618)"
    )
    # rtabmap публикует PoseWithCovarianceStamped; подписка PoseStamped
    # на rmw_zenoh молча не получает ничего (замер на роботе 16.09).
    assert "PoseWithCovarianceStamped,\n" in text, (
        "fix regressed: type подписки должен быть PoseWithCovarianceStamped"
    )

