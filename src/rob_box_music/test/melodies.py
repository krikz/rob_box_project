"""Тестовые RTTTL-мелодии (свои, не из архива) и профили тем для ``compose`` (ADR-0149 PR-3a)."""

from __future__ import annotations

from rob_box_music.arrange.compose import compose
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import ThemeProfile


def rtttl(name: str, bpm: int, tokens: str) -> str:
    return f"{name}:d=8,o=5,b={bpm}:{tokens}"


#: 8 тактов до мажора восьмыми и четвертями, без хроматики; бас-тоника в начале каждой фразы.
LONG = rtttl("long", 130, ",".join([
    "c,e,g,e,4f,a,f", "4e,g,c6,4b,g", "a,f,d,f,4g,e,c", "4d,f,a,2g",
    "c,e,g,e,4f,a,f", "4e,g,c6,4b,g", "a,b,c6,a,4g,f,e", "4d,b4,4c,4p",
]))
#: Полтора такта ля-минора шестнадцатыми — мотив = фраза + ответ.
SHORT = rtttl("short", 100, "16e6,16d6,16c6,16d6,8e6,16c6,16d6,8e6,16d6,16c6,8b,16e6,16e6,16d6,16c6,16d6,8e6,16c6,4a")
#: Долгие ноты вне любого одного лада (до-диез и ре-диез поверх до мажора).
CHROMATIC = rtttl("chroma", 120, "4c,4c#,4d,4d#,4e,4f,4f#,4g,4g#,4a,4a#,4b,4c6,4c#6,4d6,4d#6")
#: Темп ринг-тона вдвое ниже трека: доли растягиваются вдвое.
SLOW = rtttl("slow", 65, "16c,16e,16g,16e,8f,16a,16f,8e,16g,16c6,8b,16g,16a,16f,16d,16f,8g,16e,16c,8d,16f,16a,4g")

MELODIES = {"long": LONG, "short": SHORT, "chroma": CHROMATIC, "slow": SLOW}


def profile(root: int = 9, mode: str = "minor", hooks=("long", "short", "slow", "chroma"), bpm: int = 132,
            row: str = "test") -> ThemeProfile:
    return ThemeProfile("тест", "club", bpm, root, mode, tuple(hooks), row)


def compose_p(prof: ThemeProfile, track_no: int, set_seed: int = 0, **kw):
    """``compose`` по seeded-плану профиля: так трек получают тесты PR-3a (тема → хук)."""
    return compose(seeded_plan(prof, set_seed), track_no, **kw)
