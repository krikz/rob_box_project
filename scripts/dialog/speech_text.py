"""Варианты правил «текст речи» системного промпта для консоли - только стенд, промпт робота не меняется.

``robot`` - раздел «# 2. SPEECH TEXT» как на роботе (``master_prompt_compact.txt``). ``plain`` - голый текст: модель
пишет слова и обычную пунктуацию, без кавычек, тире прямой речи, разметки и эмодзи; паузы, ударения и типографику
под голос или экран делает код. Проверка гипотезы 29.10: кавычки внутри реплик ломают JSON команды (робот 14:15 UTC,
«анекдот про вайп кодеров» → ``bad_json`` → «Не понял, как это сделать»).
"""

from __future__ import annotations

import re
from typing import Callable, Optional

ROBOT, PLAIN = "robot", "plain"
VARIANTS = (ROBOT, PLAIN)

#: Раздел промпта робота от заголовка до следующего раздела «# 3.».
_SECTION_RE = re.compile(r"^# 2\. SPEECH TEXT\n.*?(?=^# 3\.)", re.MULTILINE | re.DOTALL)

PLAIN_SECTION = """# 2. SPEECH TEXT
Everything in `speech` and `perform.segments` goes to a speech engine as is. The robot's code formats it for each
voice and screen (pauses, stress, quotes, dashes). Write PLAIN TEXT:
- Brief: 1–2 sentences per answer; don't explain what you just did.
- Words and the punctuation . , ! ? : ; only (a hyphen inside a word is fine). No quotation marks of any kind
  (" « » „ “ '), no dash before direct speech, no brackets, no markup (SSML, tags, Markdown), no emoji.
- Numbers, formulas and symbols spelled out in Russian words: 2024 → две тысячи двадцать четвёртый,
  78% → семьдесят восемь процентов, α → альфа.
- Only Cyrillic and Latin letters: a foreign word or idiom goes in Russian transliteration (аригато);
  a chunk in CJK / Devanagari / Arabic / Greek is not spoken (`unsupported_script`) — rewrite it.
- Direct speech in `perform`: one segment = the words of one speaker, who speaks is `role_per_segment`. Don't
  quote the words and don't put the speaker's name with a colon in front of them.
- A pause is the end of a sentence or a new segment; no pause tags. Pitch, speed and volume — the dedicated tools.

"""


def patch(variant: str) -> Optional[Callable[[str], str]]:
    """Правка системного промпта для варианта; ``robot`` - без правки. Раздела нет - ``ValueError``: промпт робота
    поменялся, и вариант молча сравнивал бы не то."""
    if variant == ROBOT:
        return None
    if variant != PLAIN:
        raise ValueError(f"вариант текста речи {variant!r}: есть {', '.join(VARIANTS)}")

    def plain(system: str) -> str:
        patched, found = _SECTION_RE.subn(PLAIN_SECTION, system, count=1)
        if not found:
            raise ValueError("в системном промпте нет раздела «# 2. SPEECH TEXT» до «# 3.»")
        return patched

    return plain


__all__ = ["PLAIN", "PLAIN_SECTION", "ROBOT", "VARIANTS", "patch"]
