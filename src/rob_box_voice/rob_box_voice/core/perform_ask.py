"""Окно ответа на «Сейчас играет …, выключить и зачитать?» (ADR-0150 §5.6.3, О9 (б) + О9′; PR-7c ч.3в #3700).

Выступлению нужен бит, а играет чужая музыка: блок спрашивает человека (``perform.ask_replace_music``). Когда вопрос
прозвучал, здесь открывается кадр ``PerformAsk`` с окном ``REPLY_WINDOW_S`` (то же окно, что у вопроса об имени и
приветствия, + грейс микрофона) и ``pending_ask`` сессии ``Ask(yes_no)``: «да»/«нет» без вейка в окне — адресовано
(``address.PERFORM_ANSWER``), Tier-1 грамматика да/нет (``grammar._answer``) отдаёт ответ выступлению. Нет ответа —
выступление само считает это «нет» по своему сроку (``PERFORM_ASK_WAIT_S``). Функции, а не методы ``DialogueNode``.
"""

from __future__ import annotations

from typing import Any, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Ask
from rob_box_voice.core.identity_question import session

KIND = "PerformAsk"
ASK = Ask(expects="yes_no", key="perform.ask_replace_music")


def open_window(node: Any, title: str) -> None:
    """Вопрос прозвучал — окно ответа открыто (с грейсом микрофона после речи)."""
    current = session(node)
    current.pop_scene(KIND)
    current.push_scene(KIND, "code", ttl_s=float(kn.IDENTITY_MIC_GRACE_S + kn.REPLY_WINDOW_S),
                       payload={"asked_at": current.now() + kn.IDENTITY_MIC_GRACE_S, "title": title})
    current.set_pending_ask(ASK)
    node.get_logger().info(f"💬 [reply-window] perform «{title}»: окно да/нет открыто ttl={kn.REPLY_WINDOW_S}s")


def window(node: Any) -> Optional[Any]:
    """Кадр ``PerformAsk`` с открытым окном (истёкшие сессия снимает) или ``None``."""
    current = session(node)
    current.expire()
    return current.snapshot.top(KIND)


def close(node: Any) -> None:
    """Ответ принят (или выступления больше нет) — окно и вопрос сессии снимаются."""
    current = session(node)
    if current.pop_scene(KIND) is not None:
        current.set_pending_ask(None)


__all__ = ["ASK", "KIND", "close", "open_window", "window"]
