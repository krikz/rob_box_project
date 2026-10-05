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
#: Мусор: все двенадцать звуков поровну тритонами — ни один лад не берёт больше 7/12 длительности (< 0.6, I12).
CHROMATIC = rtttl("chroma", 120, ",".join(["4c,4f#,4c#6,4g,4d,4g#,4d#6,4a,4e,4a#,4f,4b"] * 2))
#: Темп ринг-тона вдвое ниже трека: доли растягиваются вдвое.
SLOW = rtttl("slow", 65, "16c,16e,16g,16e,8f,16a,16f,8e,16g,16c6,8b,16g,16a,16f,16d,16f,8g,16e,16c,8d,16f,16a,4g")

#: Ля минор в пределах октавы с проходящими полутонами (ре-диез, ля-диез) — как «Инспектор Гаджет» (#3409).
PASSING = rtttl("passing", 130, ",".join([
    "a,b,c6,d6,4e6,4p", "d6,e6,d#6,e6,4c6,4p", "a,b,c6,d6,4e6,4p", "4c6,b,a#,2a",
    "a,b,c6,d6,4e6,4p", "d6,e6,d#6,e6,4c6,4p", "d6,c6,b,c6,4a,4p", "4c6,4b,2a",
]))
#: До мажор в октаве 5–6 с двумя низами c4 — шире коридора хука, как «Терминатор» (#3409).
WIDE = rtttl("wide", 130, ",".join([
    "c,e,g,e,4c6,4p", "4c4,e,g,4c6,4p", "d,f,a,f,4d6,4p", "4b,g,e,2c",
    "c,e,g,e,4c6,4p", "4c4,e,g,4c6,4p", "d,f,a,f,4d6,4p", "4b,g,d,2c",
]))
#: Два голоса через три октавы: в коридор лида встаёт половина нот — мотив переносом не спасти.
SPREAD = rtttl("spread", 130, ",".join(["c4,c7,e4,e7,g4,g7,e4,e7"] * 8))

MELODIES = {"long": LONG, "short": SHORT, "chroma": CHROMATIC, "slow": SLOW}


def profile(root: int = 9, mode: str = "minor", hooks=("long", "short", "slow", "chroma"), bpm: int = 132,
            row: str = "test") -> ThemeProfile:
    return ThemeProfile("тест", "club", bpm, root, mode, tuple(hooks), row)


def compose_p(prof: ThemeProfile, track_no: int, set_seed: int = 0, **kw):
    """``compose`` по seeded-плану профиля: так трек получают тесты PR-3a (тема → хук)."""
    return compose(seeded_plan(prof, set_seed), track_no, **kw)
