"""Основы слов темы песни - одно правило для тула библиотеки и для диалога (#4216, М11 n707).

Тема «енота», «Енот» и «енот-механик» - одна: основа слова без двух последних букв, но не короче :data:`STEM_MIN`.
Слова о самой песне («песня», «трек») - не тема: «расскажи про песню про енота» - тема «енот». Тул ``music_library``
(``engine.gen.last_generated``) по этим основам узнаёт свежесочинённый трек в ``search``, диалог - что реплика называет
тему только что заказанной песни (``rob_box_dialog.track_ref``).
"""

from __future__ import annotations

import re
from typing import Tuple

#: Слова короче не тема. Основа слова - без двух последних букв, но не короче ``STEM_MIN``: «енота» и «енот-механик» -
#: «енот».
WORD_MIN = 4
STEM_MIN = 4
#: Слова о самой песне - не тема («расскажи про песню про енота»: тема - «енот», а не «песня»).
GENERIC_STEMS: Tuple[str, ...] = ("песн", "трек", "композиц", "музык", "мелоди", "инструментал", "произведен", "саундтрек")
_WORD = re.compile(r"[^\W\d_]+")


def topic_stems(text: str) -> Tuple[str, ...]:
    """Основы слов темы запроса (строчные, ё → е; слова о самой песне отброшены)."""
    words = (w.casefold().replace("ё", "е") for w in _WORD.findall(text or ""))
    stems = (w[:max(STEM_MIN, len(w) - 2)] for w in words if len(w) >= WORD_MIN)
    return tuple(dict.fromkeys(s for s in stems if not s.startswith(GENERIC_STEMS)))


__all__ = ["GENERIC_STEMS", "STEM_MIN", "WORD_MIN", "topic_stems"]
