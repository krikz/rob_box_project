"""Утверждение о состоянии подсистемы робота без данных о ней (#4286, класс К1 #4020; ADR-0148: решает код).

ТАРС на «как сеть, есть связь» (живой 03.11 №13, #4253) сказал «Wi-Fi и роутер живы»: в ходе - ``get_current_time`` и
``show_metrics up``, в ``recent_actions`` - ``internet_available``, а о Wi-Fi и роутере - ни тула, ни поля. Слова о
состоянии подсистемы честны, только когда модель видела данные о ней: исход ``done`` тула подсистемы или поле в
данных (``knowledge.STATE_CLAIMS``).

Знание здесь одно: что считается утверждением (:func:`claims` - подсистема и слово состояния в одном предложении, не
вопрос, не «данных нет»/«гляну»), чем оно подкреплено (:func:`backed`) и как вырезать неподкреплённое из речи
(:func:`without_claims`). Ход (``turn``) и аудит логов (``scripts/dialog/honesty_audit.py``) зовут эти функции.
"""

from __future__ import annotations

import re
from dataclasses import replace
from typing import TYPE_CHECKING, Any, Collection, Iterable, List, Mapping, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases

if TYPE_CHECKING:  # аудит логов (scripts/dialog) зовёт :func:`claims` без каталога тулов, что тянет command
    from rob_box_dialog.command import Command

#: Предложение: до знака конца включительно (последнее - и без знака).
_SENTENCE = re.compile(r"[^.!?…]+[.!?…]*")
#: Части предложения (подсистема и её состояние - в одной; оговорки ``STATE_CLAIM_UNLESS`` - там же): «данных о Wi-Fi
#: нет, но роутер жив». Двоеточие не делит: «ROS2-ноды: упавших нет» - одно утверждение.
_PART = re.compile(r"[^,;—–()]+")
_WORD = re.compile(r"[\w-]+")


def _words(text: str) -> Tuple[str, ...]:
    return tuple(w.lower().replace("ё", "е").strip("-") for w in _WORD.findall(text or ""))


def _names(words: Sequence[str], stems: Sequence[str]) -> bool:
    """Слово начинается с основы - само или частью через дефис («ROS2-ноды» - «ноды»; «ros2-control» - не «нод»)."""
    return any(part.startswith(tuple(stems)) for w in words if w for part in (w, *w.split("-")) if part)


def _unless(part: str) -> bool:
    low = part.lower().replace("ё", "е")
    return any(stem in low if " " in stem else _names(_words(part), (stem,)) for stem in kn.STATE_CLAIM_UNLESS)


def _states(words: Sequence[str]) -> bool:
    """Слово состояния или число («Батарея 90%»)."""
    return any(w.startswith(kn.STATE_CLAIM_STEMS) or w in kn.STATE_CLAIM_FORMS or w[:1].isdigit() for w in words)


def _part_claims(part: str, claim: kn.StateClaim) -> bool:
    """Часть предложения (между запятыми, двоеточием, тире) утверждает состояние подсистемы: она названа, слово
    состояния рядом с ней, оговорки нет. В разных частях - не утверждение о ней: «cadvisor отвечают по сети, значит
    канал работает» (живой 03.11, #4286) - «сеть» не подлежащее."""
    return any(_names(_words(p), claim.subjects) and _states(_words(p)) and not _unless(p) for p in _PART.findall(part))


def _claimed(sentence: str, claim: kn.StateClaim) -> bool:
    """Предложение утверждает состояние подсистемы (:func:`_part_claims`). Вопрос («роутер жив?») - не утверждение."""
    return not sentence.rstrip().endswith("?") and _part_claims(sentence, claim)


def claims(text: str) -> Tuple[str, ...]:
    """Подсистемы (ключи ``knowledge.STATE_CLAIMS``), о состоянии которых ``text`` что-то утверждает, по порядку
    таблицы."""
    sentences = _SENTENCE.findall(text or "")
    return tuple(kind for kind, claim in kn.STATE_CLAIMS.items() if any(_claimed(s, claim) for s in sentences))


def _keys(value: Any) -> Iterable[str]:
    if isinstance(value, Mapping):
        for key, item in value.items():
            yield str(key).lower()
            yield from _keys(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            yield from _keys(item)


def backed(kind: str, results: Iterable[Mapping[str, Any]]) -> bool:
    """Утверждение о подсистеме ``kind`` подкреплено исходами ``results`` (``{"tool", "status", "data"}``: строки
    ``recent_actions`` и исходы ``query`` хода): исход ``done`` тула подсистемы или ключ данных с её подстрокой."""
    claim = kn.STATE_CLAIMS[kind]
    for result in results:
        if result.get("status") != "done":
            continue
        if result.get("tool") in claim.tools:
            return True
        if any(part in key for key in _keys(result.get("data")) for part in claim.keys):
            return True
    return False


def unbacked(text: str, results: Sequence[Mapping[str, Any]]) -> Tuple[str, ...]:
    """Подсистемы, о которых ``text`` утверждает, а данных о них нет (пусто - нарушения нет)."""
    return tuple(kind for kind in claims(text) if not backed(kind, results))


def evidence(results: Iterable[Any]) -> List[Mapping[str, Any]]:
    """Исходы как строки для :func:`backed`: ``ActionResult`` хода и словари ``recent_actions`` - одной формой."""
    rows: List[Mapping[str, Any]] = []
    for r in results:
        if isinstance(r, Mapping):
            rows.append(r)
        else:
            rows.append({"tool": r.tool, "status": r.status, "data": r.data})
    return rows


def _join(parts: Sequence[str]) -> str:
    text = " ".join(p.strip() for p in parts if p.strip())
    return re.sub(r"\s+([,.!?…;:])", r"\1", text)


def _cut(sentence: str, kinds: Collection[str]) -> str:
    """Предложение без частей-утверждений (:func:`_part_claims`): «Связь с интернетом есть, Wi-Fi и роутер живы.» без
    ``wifi`` - «Связь с интернетом есть.»."""
    claimed = [kn.STATE_CLAIMS[k] for k in kinds if _claimed(sentence, kn.STATE_CLAIMS[k])]
    if not claimed:
        return sentence
    kept: List[str] = []
    for part in re.split(r"(,)", sentence):
        if any(_part_claims(part, c) for c in claimed):
            if kept and kept[-1] == ",":
                kept.pop()
            continue
        kept.append(part)
    text = "".join(kept).strip().lstrip(",").strip()
    if not any(ch.isalnum() for ch in text):
        return ""
    end = sentence.rstrip()[-1:]
    if end in ".!…" and not text.endswith(end):
        text = text.rstrip(",;: ") + end
    return text[:1].upper() + text[1:]


def without_claims(text: str, kinds: Collection[str]) -> str:
    """``text`` без утверждений о подсистемах ``kinds``: «Связь с интернетом есть, Wi-Fi и роутер живы.» без ``wifi`` -
    «Связь с интернетом есть.». Остались одни знаки - пусто."""
    return _join([_cut(s, kinds) for s in _SENTENCE.findall(text or "")])


def phrase(kinds: Sequence[str]) -> str:
    """Фраза кода: о каких подсистемах данных нет (``knowledge.STATE_CLAIM_PHRASE``)."""
    labels = [kn.STATE_CLAIMS[k].label for k in kinds]
    subjects = labels[0] if len(labels) == 1 else ", ".join(labels[:-1]) + " и " + labels[-1]
    return phrases.text(kn.STATE_CLAIM_PHRASE, subjects=subjects)


def offending(agent: str, cmd: Command, results: Sequence[Mapping[str, Any]]) -> Tuple[str, ...]:
    """Неподкреплённые утверждения в речи команды агента из ``knowledge.STATE_CLAIM_AGENTS`` (другим - пусто)."""
    if agent not in kn.STATE_CLAIM_AGENTS or not cmd.speech:
        return ()
    return unbacked(cmd.speech, results)


def unclaimed(cmd: Command, kinds: Collection[str]) -> Command:
    """Команда, чья речь без утверждений ``kinds``, а рядом - фраза кода, о чём данных нет (повтора у модели больше
    нет: ложное не звучит, правда о данных - звучит)."""
    rest = without_claims(cmd.speech or "", kinds)
    said = " ".join(p for p in (rest, phrase([k for k in kn.STATE_CLAIMS if k in kinds])) if p)
    return replace(cmd, speech=said or None)


__all__ = ["backed", "claims", "evidence", "offending", "phrase", "unbacked", "unclaimed", "without_claims"]
