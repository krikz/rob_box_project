"""План сета ``SetPlan`` и ``seeded_plan(profile, seed)`` без LLM (ADR-0149 §4.4–§4.6; PR-3b).

* **Один темп на сет** (§4.4, решение Шифу В6): ``SetPlan.bpm`` — темп профиля темы в окне стиля
  (``knowledge.STYLES``, club 128–138). У треков своего темпа нет: ``compose`` берёт его из плана.
* **Длина сета и дуга энергии** (ADR-0147 §3.4, встроена по §4.6): сет конечный — ``n_tracks`` треков (по
  умолчанию :data:`DEFAULT_TRACKS`, число из фразы человека — :data:`MAX_TRACKS` максимум; 06.10 сет без конца дошёл
  до 53-го трека). Энергия — волна ``knowledge.ENERGY_WAVE`` по номеру трека (длинный сет повторяет её), последний
  трек — спад к уровню интро (:func:`arc_energy`); короткий сет начинает волну позже, чтобы пик был перед спадом.
* **Ход тоники** (§8.1: перенос ``dj_set_walk.related_root``): чистая квинта вверх на трек — соседи по кругу
  квинт делят 6 из 7 нот (Camelot +1), за 12 треков все 12 тоник. Старый ``dj_set_walk`` импортирует
  :func:`root_shift` отсюда — одна реализация.
* **Жанровое окно** (ADR-0152 §3.5, PR-8, В1): ``SetPlan.genre`` — окно клуба (``Style.genre_windows``: club/deep/
  breaks) выбирается на сет сидом со штрафом за окна прошлых сетов (:func:`pick_genre`); темп сета — в окне
  (:func:`plan_bpm`), пул бочек и пэдов, рисунок бочки дропа — из окна (``knowledge.genre_style``). Внутри сета окно
  не меняется. Окно — не стиль (ADR-0153): стиль — набор таблиц, окно — подвыбор внутри него.
* **Свинг сета** — в окне стиля, от сида; один на весь сет, как грув у диджея.
* **Материал партитуры** (ADR-0154 §3.5, PR-5): ``TrackPlan.material`` — как ``kick``/``template``, решает план
  (:func:`plan_materials`): тема нашла партитуры по названию — первые треки берут их по одной, трек 1 — материал №1.
* **Тоника сета** (PR-3d, ось «тоника» ``music_history``): тоника темы, если её не было в последних
  ``TONIC_MEMORY`` треках истории; иначе — выбор сидом (``diversity.weighted_pick``) среди тоник, которых там не
  было. Темп сета от истории не зависит.

Уточнение плана от LLM по JSON-схеме — ``reasoner`` (PR-10): ``reasoner.apply`` меняет ``tracks``, не темп.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field, replace
from typing import Callable, Dict, Mapping, Optional, Sequence, Tuple

from . import knowledge as kn
from .diversity import last_opener, opening_order, recent_hooks, recent_values, weighted_pick
from .theme import ThemeProfile

#: Шаг тоники между соседними треками: чистая квинта вверх, полутонов.
FIFTH = 7
#: Длина сета без числа, потолок и средний трек — из ``knowledge`` (одна таблица знания), здесь — короткие имена.
DEFAULT_TRACKS = kn.SET_TRACKS
MAX_TRACKS = kn.SET_MAX_TRACKS
TRACK_SECONDS = kn.SET_TRACK_SECONDS
#: Сколько последних треков истории не должна повторять тоника нового сета.
TONIC_MEMORY = 4


@dataclass(frozen=True)
class TrackPlan:
    no: int  # с 1
    energy: int  # 1..5
    root_shift: int  # полутонов от тоники сета, 0..11
    kick: str = ""  # бочка из пула стиля (``knowledge.KICK_SOUNDS``); пусто — ``compose`` выбирает по истории сам
    template: str = ""  # форма трека (ключ ``Style.forms``); пусто — ``compose`` выбирает по энергии и истории сам
    #: Материал партитуры трека (``ScoreMaterial.material_id``, ADR-0154 §3.5); None — хук темы, как раньше. Вне
    #: ``repr``: ``repr(step)`` входит в sha трека, трек без материала не меняется ни байтом.
    material: Optional[str] = field(default=None, repr=False)


@dataclass(frozen=True)
class SetPlan:
    set_id: str
    seed: int
    profile: ThemeProfile  # тема, лад, хуки, тоника сета
    bpm: int  # единственный темп сета
    swing: float  # доля восьмой, на которую опаздывают нечётные 16-е хэтов
    tracks: Tuple[TrackPlan, ...]  # первые треки; дальше — :meth:`track`
    genre: str = kn.DEFAULT_GENRE  # жанровое окно клуба сета (``Style.genre_windows``), не меняется внутри сета
    timbre: str = ""  # семья тембров сета (ключ ``Style.timbres``); пусто — по строке темы (:attr:`family`)

    def track(self, no: int) -> TrackPlan:
        """План трека ``no`` (с 1): из ``tracks`` (поправка LLM, PR-10); за концом сета — волна (одиночный трек
        ``request_music`` и тесты)."""
        return self.tracks[no - 1] if 1 <= no <= len(self.tracks) else track_plan(no)

    def root(self, no: int) -> int:
        """Тоника трека ``no``, pitch class 0..11."""
        return (self.profile.root + self.track(no).root_shift) % 12

    @property
    def family(self) -> str:
        """Семья тембров сета: выбранная планом (:func:`pick_timbre`) или, если её нет, по строке темы."""
        return self.timbre or kn.family_of(kn.STYLES[self.style], self.profile.row)

    @property
    def style(self) -> str:
        """Ключ ``knowledge.STYLES`` сета (ADR-0153: один стиль на сет) — стиль профиля темы."""
        return self.profile.style

    @property
    def table(self) -> kn.Style:
        """Таблицы стиля сета в его жанровом окне: то, что читают ``arrange/*`` (``knowledge.genre_style``)."""
        return kn.genre_style(kn.STYLES[self.style], self.genre)


def track_energy(no: int) -> int:
    """Энергия трека ``no`` (с 1) по волне ``knowledge.ENERGY_WAVE``."""
    return kn.ENERGY_WAVE[(max(1, no) - 1) % len(kn.ENERGY_WAVE)]


def arc_energy(no: int, n_tracks: int) -> int:
    """Энергия трека ``no`` (с 1) сета из ``n_tracks`` треков: волна ``ENERGY_WAVE``, последний трек — спад к уровню
    интро (``ENERGY_WAVE[0]``). Сет короче волны начинает её позже — пик на предпоследнем треке: 3 трека → 4, 5, 2."""
    wave = kn.ENERGY_WAVE
    if n_tracks > 1 and no >= n_tracks:
        return wave[0]
    shift = max(0, wave.index(max(wave)) - (n_tracks - 2))
    return wave[(shift + max(1, no) - 1) % len(wave)]


def set_tracks(tracks: object) -> int:
    """Длина сета, названная человеком: целое 1..:data:`MAX_TRACKS` (строка из цифр — тоже), иначе ``ValueError`` с
    понятной причиной."""
    if isinstance(tracks, str) and tracks.strip().isdigit():
        tracks = int(tracks)
    if type(tracks) is not int or not 1 <= tracks <= MAX_TRACKS:
        raise ValueError(f"tracks={tracks!r}: длина сета — целое число треков от 1 до {MAX_TRACKS}")
    return tracks


def root_shift(no: int) -> int:
    """Сдвиг тоники трека ``no`` (с 1) от тоники сета: трек 1 — 0, каждый следующий — квинта выше."""
    return FIFTH * (max(1, no) - 1) % 12


def track_plan(no: int, n_tracks: int = 0) -> TrackPlan:
    """План трека ``no``; ``n_tracks`` — длина сета (дуга :func:`arc_energy`), 0 — волна без конца."""
    return TrackPlan(no, arc_energy(no, n_tracks) if n_tracks else track_energy(no), root_shift(no))


def set_root(profile: ThemeProfile, seed: int, history: Sequence[Mapping] = ()) -> int:
    """Тоника сета: тоника темы, если её не было в последних ``TONIC_MEMORY`` треках, иначе — сидом из остальных."""
    recent = recent_values(history[:TONIC_MEMORY], "root")
    if kn.ROOTS[profile.root] not in recent:
        return profile.root
    options = [r for r in kn.ROOTS if r not in recent]
    return kn.ROOTS.index(weighted_pick(options, recent, random.Random(f"root:{seed}:{profile.theme}")))


def recent_set_values(history: Sequence[Mapping], field: str) -> list:
    """Значения оси ``field`` прошлых СЕТОВ, свежие первыми: строки истории (свежие первыми) свёрнуты по ``set_id``
    (у сета одно значение); строки без значения (записаны до появления оси) пропускаются."""
    out: list = []
    last = object()
    for row in history:
        value, set_id = row.get(field), row.get("set_id")
        if not value or (out and set_id == last):
            continue
        out.append(value)
        last = set_id
    return out


def recent_genres(history: Sequence[Mapping]) -> list:
    """Окна прошлых СЕТОВ, свежие первыми (:func:`recent_set_values`)."""
    return recent_set_values(history, "genre")


def pick_timbre(style: kn.Style, row: Optional[str], history: Sequence[Mapping], rng: random.Random) -> str:
    """Семья тембров сета (#3460, A16b). Тема из таблицы — её семья (``knowledge.THEME_TIMBRE``). Тема вне таблицы —
    сид (``rng`` от сида и темы) со штрафом за семьи прошлых сетов (``weighted_pick``, ось ``timbre``); с прошлым
    сетом подряд одна семья не повторяется. Раньше такие темы всегда получали ``default_timbre`` (все пять тем
    случайной серии 06.10 звучали ``warm``)."""
    if row in kn.THEME_TIMBRE:
        return kn.THEME_TIMBRE[row]
    recent = recent_set_values(history, "timbre")
    families = tuple(style.timbres)
    options = [f for f in families if not recent or f != recent[0]] or list(families)
    return weighted_pick(options, recent, rng)


def pick_genre(style: kn.Style, history: Sequence[Mapping], rng: random.Random) -> str:
    """Окно сета: сид + штраф за окна прошлых сетов (``weighted_pick``, ось ``genre``); с прошлым сетом подряд одно
    окно не повторяется."""
    recent = recent_genres(history)
    windows = tuple(style.genre_windows)
    options = [g for g in windows if not recent or g != recent[0]] or list(windows)
    return weighted_pick(options, recent, rng)


def plan_bpm(window: kn.GenreWindow, theme_bpm: int, seed: int, theme: str) -> int:
    """Темп сета: темп темы, если он в окне; иначе — сидом внутри окна (один темп на сет, ADR-0149 I7)."""
    lo, hi = window.bpm
    return theme_bpm if lo <= theme_bpm <= hi else random.Random(f"bpm:{seed}:{theme}").randint(lo, hi)


def pick_kick(style: kn.Style, history: Sequence[Mapping], rng: random.Random) -> str:
    """Бочка трека из пула стиля: сид + штраф за недавние в истории (``weighted_pick``, ось ``kick``), с прошлым
    треком подряд не повторяется."""
    recent = recent_values(history, "kick")
    options = [k for k in style.kick_pool if not recent or k != recent[0]] or list(style.kick_pool)
    return weighted_pick(options, recent, rng)


def plan_kicks(style: kn.Style, seed: int, theme: str, n_tracks: int,
               history: Sequence[Mapping] = ()) -> Tuple[str, ...]:
    """Бочки первых ``n_tracks`` треков сета: каждая выбрана со штрафом за прошлые сеты и предыдущие треки плана."""
    recent = [k for k in recent_values(history, "kick") if k]
    kicks = []
    for no in range(1, n_tracks + 1):
        rows = [{"kick": k} for k in kicks[::-1] + recent]
        kicks.append(pick_kick(style, rows, random.Random(f"kick:{seed}:{theme}:{no}")))
    return tuple(kicks)


def pick_template(style: kn.Style, energy: int, history: Sequence[Mapping], rng: random.Random) -> str:
    """Форма трека энергии ``energy`` из ``Style.energy_forms``: сид + штраф за недавние (ось ``template``), с
    прошлым треком подряд не повторяется."""
    recent = recent_values(history, "template")
    allowed = style.energy_forms.get(energy) or tuple(style.forms)
    options = [t for t in allowed if not recent or t != recent[0]] or list(allowed)
    return weighted_pick(options, recent, rng)


def plan_templates(style: kn.Style, seed: int, theme: str, energies: Sequence[int],
                   history: Sequence[Mapping] = ()) -> Tuple[str, ...]:
    """Формы первых треков сета (энергия — по номеру): трек 1 — ``Style.opening_form`` (блэнда на входе нет, тему
    человек ждёт сразу, #3427), остальные — :func:`pick_template` со штрафом за прошлые сеты и предыдущие треки."""
    recent = [t for t in recent_values(history, "template") if t]
    out = []
    for no, energy in enumerate(energies, 1):
        rows = [{"template": t} for t in out[::-1] + recent]
        out.append(style.opening_form if no == 1
                   else pick_template(style, energy, rows, random.Random(f"template:{seed}:{theme}:{no}")))
    return tuple(out)


def plan_materials(materials: Sequence[str], n_tracks: int, history: Sequence[Mapping] = (),
                   set_id: Optional[str] = None, fit: Optional[Callable[[str, int], Optional[str]]] = None,
                   rejected: Optional[Dict[str, str]] = None) -> Tuple[Optional[str], ...]:
    """Материалы партитур первых треков сета (ADR-0154 §3.5): трек 1 — материал №1 темы, следующие — по одному
    следующему, пока материалы не кончились (дальше — хуки темы, ``None``). Очередь — та же, что у хуков первого
    трека (:func:`diversity.opening_order`, #3399/#3495): звучавший в последних ``HOOK_FRESH_SETS`` сетах — после
    свежих, открывший прошлый сет — последним; в истории материал записан как ``melody_name`` (``Hook.source``).

    ``fit(material_id, track_no)`` — причина негодности материала для трека или ``None`` (#3500; критерий —
    ``hook.material_unfit``, тот же ``from_material``, что у ``compose``): негодный выбывает из очереди сета, порядок
    очереди сохраняется — свежесть считается по годным. Выбывает навсегда: отказ почти не зависит от тоники трека
    (мотив, размер, ``key_fit`` — нет; только перенос октавой), а перепроверка отвергнутых на каждом треке стоила
    ×треки (замер на роботе 159 мс вместо ≤ 50, M7); назначенный треку материал проверен на тонике именно этого
    трека. Причина — в ``rejected`` (вызывающий пишет её в лог); ``fit=None`` — без отбора."""
    if not materials:
        return (None,) * n_tracks
    recent = recent_hooks(history, set_id, {m: m for m in materials})
    queue = opening_order(list(dict.fromkeys(materials)), recent, last_opener(history, set_id))
    out = []
    for no in range(1, n_tracks + 1):
        pick = None
        while queue and pick is None:
            reason = fit(queue[0], no) if fit is not None else None
            if reason is None:
                pick = queue[0]
            elif rejected is not None:
                rejected.setdefault(queue[0], reason)
            queue.pop(0)
        out.append(pick)
    return tuple(out)


def _material_fit(materials: Mapping[str, object], window: kn.Style, bpm: int,
                  profile: ThemeProfile, references: Sequence[str] = ()) -> Callable[[str, int], Optional[str]]:
    """``fit`` для :func:`plan_materials`: ``hook.material_unfit`` с темпом сета, тоникой трека, коридором хука
    окна и RTTTL-эталонами темы ``references`` — как в ``compose._from_material`` (#3542: материал без главного мотива
    эталона уступает трек следующему годному, а не RTTTL-хуку). Материал не читается (``materials`` не отдал) — это
    тоже причина."""
    from .arrange.compose import hook_register  # compose импортирует set_plan — импорт здесь, не наверху
    from .arrange.hook import material_unfit
    register = hook_register(window)

    def fit(material_id: str, no: int) -> Optional[str]:
        try:
            material = materials[material_id]
        except (KeyError, OSError, ValueError) as exc:
            return f"не читается: {type(exc).__name__}: {exc}"
        root = (profile.root + root_shift(no)) % 12
        return material_unfit(material, bpm, root, profile.mode, register, references)  # type: ignore[arg-type]

    return fit


def seeded_plan(profile: ThemeProfile, seed: int, n_tracks: int = DEFAULT_TRACKS, set_id: str = "v2",
                history: Sequence[Mapping] = (), genre: Optional[str] = None,
                materials: Optional[Mapping[str, object]] = None,
                rejected: Optional[Dict[str, str]] = None, references: Sequence[str] = ()) -> SetPlan:
    """План сета мгновенно, без сети и LLM: детерминирован по ``(profile, seed, history)``; ``history`` — строки
    ``music_history`` (свежие первыми). ``n_tracks`` — длина сета: треков в плане столько, сколько сыграет сет.
    ``genre`` — окно, заданное явно (тема, оператор); ``None`` — :func:`pick_genre`. ``materials`` —
    ``{material_id: ScoreMaterial}`` для отбора годных (#3500, :func:`plan_materials`); нет — без отбора;
    ``rejected`` получает ``{material_id: причина}`` негодных; ``references`` — RTTTL ``profile.theme_hooks``
    (эталоны главного мотива, как у ``compose``)."""
    profile = replace(profile, root=set_root(profile, seed, history))
    base = kn.STYLES[profile.style]
    genre = pick_genre(base, history, random.Random(f"genre:{seed}:{profile.theme}")) if genre is None else genre
    window = kn.genre_style(base, genre)
    bpm = plan_bpm(base.genre_windows[genre], profile.bpm, seed, profile.theme)
    s_lo, s_hi = window.swing
    swing = round(s_lo + random.Random(f"plan:{seed}:{profile.theme}").random() * (s_hi - s_lo), 3)
    n = max(1, n_tracks)
    kicks = plan_kicks(window, seed, profile.theme, n, history)
    plans = [track_plan(no, n) for no in range(1, n + 1)]
    forms = plan_templates(window, seed, profile.theme, [p.energy for p in plans], history)
    picked = plan_materials(profile.materials, n, history, set_id,
                            None if materials is None else _material_fit(materials, window, bpm, profile, references),
                            rejected)
    tracks = tuple(replace(p, kick=kicks[p.no - 1], template=forms[p.no - 1], material=picked[p.no - 1])
                   for p in plans)
    timbre = pick_timbre(base, profile.row, history, random.Random(f"timbre:{seed}:{profile.theme}"))
    return SetPlan(set_id, seed, profile, bpm, swing, tracks, genre, timbre)


__all__ = ["DEFAULT_TRACKS", "FIFTH", "MAX_TRACKS", "SetPlan", "TONIC_MEMORY", "TRACK_SECONDS", "TrackPlan", "arc_energy",
           "pick_genre", "pick_kick",
           "pick_template", "pick_timbre", "plan_bpm", "plan_materials", "plan_kicks", "plan_templates", "recent_genres", "recent_set_values", "root_shift", "seeded_plan",
           "set_root", "set_tracks", "track_energy", "track_plan"]
