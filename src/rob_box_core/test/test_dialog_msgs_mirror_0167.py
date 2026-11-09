"""Зеркало ``.msg`` в данных ядра совпадает с ``rob_box_dialog_msgs`` байт в байт (ADR-0167 Ш1а-2).

Заглушка сообщений (``rob_box_core.msg_stub``) строит типы из ``rob_box_core/dialog_msgs/*.msg`` - данных пакета ядра,
чтобы ядро без чекаута не искало ``.msg`` вне себя. Владелец описаний - ``rob_box_dialog_msgs``: по ним rosidl
собирает настоящий тип робота. Поле переименовали там и забыли здесь - красный тест, а не заглушка, расходящаяся
с настоящим типом (ради чего заглушка и строится из ``.msg``).

Обновить зеркало::

    cp src/rob_box_dialog_msgs/msg/*.msg src/rob_box_core/rob_box_core/dialog_msgs/
"""

from __future__ import annotations

from pathlib import Path

from rob_box_core import msg_stub

SRC = Path(__file__).resolve().parents[2]
OWNER = SRC / "rob_box_dialog_msgs" / "msg"
MIRROR = SRC / "rob_box_core" / "rob_box_core" / msg_stub.DIALOG_MSGS_DATA
FIX = "cp src/rob_box_dialog_msgs/msg/*.msg src/rob_box_core/rob_box_core/dialog_msgs/"


def _texts(directory: Path) -> dict:
    # Переводы строк не сравниваем: чекаут Windows (autocrlf) отдаёт CRLF, Linux - LF; смысл описания тот же.
    return {p.name: p.read_bytes().replace(b"\r\n", b"\n") for p in sorted(directory.glob("*.msg"))}


def test_owner_messages_are_found():
    assert len(_texts(OWNER)) >= 20, OWNER


def test_mirror_has_the_same_messages_as_the_owner():
    assert sorted(_texts(MIRROR)) == sorted(_texts(OWNER)), FIX


def test_mirror_is_byte_for_byte_the_owner():
    owner, mirror = _texts(OWNER), _texts(MIRROR)
    differ = [name for name in owner if mirror.get(name) != owner[name]]
    assert not differ, f"зеркало отстало: {differ}; {FIX}"


def test_stub_reads_the_package_data_not_the_checkout():
    msg_dir = msg_stub.dialog_msgs_dir()
    assert msg_dir.name == msg_stub.DIALOG_MSGS_DATA
    assert Path(str(msg_dir)).resolve() != OWNER.resolve()
    module = msg_stub.build_dialog_msgs_module()
    assert sorted(n for n in vars(module) if not n.startswith("_")) == sorted(p[:-4] for p in _texts(OWNER))
