"""Переспрос про личность по ack регистрации — отложить, задать, прочитать ответ.

Issue #2828. PR #2798 научил ``dialogue_node`` переспрашивать вслух, когда
ack регистрации несёт ``voice_conflict`` (голос похож на знакомого, имя
другое) или ``name_twin`` (имя совпало, голос не дотянул). Смысл вопроса —
отложенный вариант C из ADR-0127: решение «один это человек или двое»
принимает человек, а не косинус. Но в PR #2798 были два дефекта, из-за
которых вопрос ничего не решал:

1. **Столкновение с ответом хода.** Ack приходит, пока LLM ещё дописывает
   ответ того же хода (``register_speaker`` → приветствие). Вопрос уходил в
   TTS напрямую, а через секунду за ним шло «Привет, Борис!». Робот
   спрашивал и сам себе отвечал (run 35854757524, шаг n204).
2. **Ответ никто не читал.** Ни одной публикации в ``/voice/speaker/merge``
   в ``dialogue_node`` не было: «это я» ничего не склеивало, «мы разные»
   ничего не подтверждало. Вопрос ради вопроса.

Здесь живёт состояние этого переспроса (без ROS, чтобы проверять его без
сборки ноды и не раздувать CC методов ``DialogueNode``, ADR-0021):

* ``hold``/``take_held`` — вопрос, пришедший посреди хода, ждёт точки
  выдачи ответа; там он ЗАМЕНЯЕТ ответ хода (или, если ответ уже ушёл,
  звучит после него — последним);
* ``arm``/``consume`` — после того как вопрос прозвучал, следующая живая
  реплика читается как ответ, один раз и не позже ``ANSWER_TTL_S``;
* ``pop_hint_lines`` — разовая подсказка в ``<system_context>``: LLM не
  видела вопроса (он шёл мимо неё) и должна узнать и вопрос, и исход.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

#: Сколько ждём ответа на переспрос. Дольше — ответ уже не про это:
#: человек отошёл, разговор ушёл в сторону, и случайное «да» через пять
#: минут не должно склеить два профиля. От «да» из чужого разговора
#: главным образом защищает то, что ожидание снимается ПЕРВОЙ же живой
#: репликой (``consume``); TTL — только потолок. Меньше минуты брать
#: нельзя: в E2E шаги идут с интервалом 60–85 с, а «не расслышал» и
#: повтор отодвигают ответ ещё дальше (run 35857257981: 84 с между
#: вопросом и ответом убили 30-секундное окно #2809).
ANSWER_TTL_S: float = 180.0

# Слова и обороты сравниваются так же, как в
# ``dialogue_node.classify_identity_confirmation``: слова — целиком,
# обороты — подстрокой.
_DIFFERENT_WORDS = frozenset({
    "разные", "разных", "другой", "другая", "другие", "другого",
})
_DIFFERENT_PHRASES = (
    "не я", "не он", "не она", "не тот", "не та ", "не один", "не одно",
    "обознал",
)
_SAME_PHRASES = (
    "это я", "я и есть", "тот самый", "та самая", "тот же", "та же",
    "один и тот же", "одно и то же", "один человек", "одно лицо",
)


def identity_ack_plan(ack: dict, question: str) -> dict:
    """Что спрашиваем и что склеивать при «это я» — из ack регистрации.

    ``new_id`` — профиль, заведённый этой регистрацией; ``known_id`` —
    тот, с кем его перепутать можно (тёзка или похожий голос). Склейка
    при «это я» идёт ``new_id`` → ``known_id``: у старого профиля больше
    эмбеддингов и своя история, его имя и остаётся (так устроен
    ``speaker_id_node._on_merge_request``).
    """
    kind = "twin" if ack.get("name_twin") else "conflict"
    other = ack.get("name_twin") or ack.get("voice_conflict") or {}
    return {
        "kind": kind,
        "question": question,
        "new_id": str(ack.get("speaker_id") or ""),
        "new_name": str(ack.get("name") or ""),
        "known_id": str(other.get("speaker_id") or ""),
        "known_name": str(other.get("name") or ""),
    }


def classify_identity_ack_answer(
    text: str, kind: str, yes_no: Optional[bool] = None
) -> Optional[bool]:
    """``True`` — один человек, ``False`` — разные, ``None`` — непонятно.

    ``yes_no`` — ответ ``classify_identity_confirmation`` на тот же текст
    (переиспользуем словарь #2818, а не заводим второй). Он годится только
    для ``twin``: «Ты тот самый Дэнчик?» — «да» значит «тот самый». На
    ``conflict`` вопрос двойной («вы разные люди или это ты под другим
    именем?»), и голое «да»/«нет» ничего не говорит — там ``None``.

    Оба сигнала сразу («не тот самый») — тоже ``None``: безопасный исход,
    профили остаются раздельными, склеить потом можно, разлепить — нечем.
    """
    norm = text.strip().lower()
    words = set(norm.replace(",", " ").replace(".", " ").replace("!", " ").split())
    different = bool(words & _DIFFERENT_WORDS) or any(
        p in norm for p in _DIFFERENT_PHRASES
    )
    same = any(p in norm for p in _SAME_PHRASES)
    if different != same:
        return same
    if different or kind != "twin":
        return None
    return yes_no


def identity_ack_hint_lines(plan: dict, same: Optional[bool]) -> list:
    """Строки для ``<user_profile>``: какой был вопрос и чем кончилось."""
    known = plan.get("known_name") or "знакомый"
    new = plan.get("new_name") or known
    asked = f"Ты только что спросил: «{plan.get('question', '')}». "
    if same is True:
        outcome = (
            f"Человек ответил, что это один человек: профиль «{new}» "
            f"склеен с «{known}», в базе осталось имя «{known}»."
        )
    elif same is False:
        outcome = (
            f"Человек ответил, что это разные люди: профили «{new}» и "
            f"«{known}» остаются раздельными."
        )
    else:
        outcome = (
            "Ответ не удалось однозначно понять — ничего не склеено, "
            "профили остаются раздельными."
        )
    return [
        f"    <identity_answer>{asked}{outcome} Коротко подтверди "
        "это и про личность больше не переспрашивай.</identity_answer>",
    ]


class IdentityAckQuestion:
    """Состояние одного переспроса по ack регистрации (см. модуль)."""

    def __init__(
        self,
        ttl_s: float = ANSWER_TTL_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._lock = threading.Lock()
        self._ttl_s = ttl_s
        self._clock = clock
        self._held: Optional[dict] = None
        self._awaiting: Optional[dict] = None
        self._asked_at = 0.0
        self._hint_lines: list = []

    def hold(self, plan: dict) -> None:
        """Ход ещё идёт — придержать вопрос до выдачи его ответа."""
        with self._lock:
            self._held = plan

    def take_held(self) -> Optional[dict]:
        """Забрать придержанный вопрос (один раз)."""
        with self._lock:
            plan, self._held = self._held, None
            return plan

    def arm(self, plan: dict) -> None:
        """Вопрос прозвучал — ждём ответ следующей живой репликой."""
        with self._lock:
            self._awaiting = plan
            self._asked_at = self._clock()

    def consume(
        self, text: str, yes_no: Optional[bool] = None
    ) -> Optional[tuple]:
        """Прочитать реплику как ответ: ``(plan, same)`` или ``None``.

        ``None`` — вопроса не было или он протух; тогда реплика обычная.
        Ожидание снимается на ПЕРВОЙ же реплике после вопроса, даже если
        ответ непонятен: иначе «да» из совсем другого разговора через
        минуту склеило бы профили (тот же довод, что у #2809).
        """
        with self._lock:
            plan, self._awaiting = self._awaiting, None
            if plan is None or self._clock() - self._asked_at > self._ttl_s:
                return None
        same = classify_identity_ack_answer(text, plan["kind"], yes_no)
        with self._lock:
            self._hint_lines = identity_ack_hint_lines(plan, same)
        return plan, same

    def pop_hint_lines(self) -> list:
        """Разовая подсказка для ``<system_context>`` этой реплики."""
        with self._lock:
            lines, self._hint_lines = self._hint_lines, []
            return lines
