"""ADR-0158 В17 (#3648, Шифу 14.10: «то что откинуло Suno — значит надо пробовать ещё раз что-то другое»).

Живой сет `gen84956`: «Имперский марш» — ``rejected: This audio matches an existing recording in our catalog`` (задача
14acb8fa…, 0 кредитов) гасил генерацию всего сета: Кантина не заказана, треки 6–7 — те же файлы, что 2–3. Без сети и
денег: фейковый KIE (отказ — ``state=fail``), настоящий кэш ``GeneratedMusicLibrary``, фейковый ffmpeg.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.gen.cover import CoverMaker, CoverResult, cover_tries
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_music import knowledge as kn
from rob_box_music.source import CoverWork, SourceAsk, cover_rejected_text, cover_slot

from .test_adr0158_pr4_cover import SITAR, Kie as CoverKie, _track
from .test_adr0158_pr8_gen_set import Kie, fake_ffmpeg

pytestmark = pytest.mark.unit

MAIN = CoverWork("Star Wars", "pdmx:main", spoken="Звёздные войны")
MARCH = CoverWork("Имперский марш", "starwars_4", spoken="Имперский марш")
CANTINA = CoverWork("Star Wars - Cantina", "starwars", spoken="Кантина")
CLUB = SourceAsk(genre="club")


def key(work):
    return cover_slot("club", work.title)


class Suno:
    """``CoverMaker.make`` без сети: ``reject`` — произведения (или ``(произведение, № мелодии)``), которые Suno не
    берёт; взятое — 2 варианта в настоящий кэш (сет их играет)."""

    def __init__(self, lib, reject=(), reason="rejected"):
        self.lib, self.reject, self.reason, self.calls = lib, set(reject), reason, []

    def make(self, track, work, ask, set_spent=None, early=True, attempt=kn.COVER_TRIES[0]):
        self.calls.append((work.title, track, attempt))
        if work.title in self.reject or (work.title, track) in self.reject:
            return CoverResult(reason=self.reason)
        task = f"T{len(self.calls)}"
        ids = tuple(self.lib.save_track(b"MP3:90.0", prompt="cover", title=f"{work.title} {v}", is_instrumental=True,
                                        kind="cover", style_key=ask.genre, task_id=task, variant=v,
                                        duration_ms=90000, work_id=work.title)["id"] for v in range(2))
        return CoverResult(ids[0], "", task, 12.0, track_ids=ids)


SOURCES = {MAIN: (("pdmx:main", "m0"),),
           MARCH: (("rtttl:starwars_4", "i0"), ("rtttl:starwars_imp", "i1"), ("rtttl:starwars_5", "i2")),
           CANTINA: (("rtttl:starwars", "c0"),)}


def _set(tmp_path, suno, plan, pieces=(MAIN, MARCH, CANTINA), speak=None, kie=None):
    lib = suno.lib
    make = lambda work, ask, spent: cover_tries(suno.make, SOURCES[work], work, ask, set_spent=spent,  # noqa: E731
                                                early=False)
    covers = gs.SetCovers({key(w): w for w in plan}, make, pieces)
    return gs.GenSet(provider=kie or Kie(), library=lib, ask=CLUB, genre="club", theme="", set_id="gen9", bpm=120,
                     tracks=len(plan), cache=[], keys=[key(w) for w in plan], covers=covers, speak=speak,
                     spawn=lambda fn: fn(), run=fake_ffmpeg)


def _play(gen, n):
    tracks = [gen.next_track(no, "AB"[(no - 1) % 2]) for no in range(1, n + 1)]
    return [gen._items[t.track_id] for t in tracks]  # noqa: SLF001


def test_rejected_source_is_retried_with_another_melody_of_the_same_piece(tmp_path):
    """Отказ по первой мелодии марша → вторая попытка: другой хук и другой рендер (``COVER_TRIES[1]``) — взят."""
    suno = Suno(GeneratedMusicLibrary(str(tmp_path / "lib")), reject={(MARCH.title, "i0")})
    result = cover_tries(suno.make, SOURCES[MARCH], MARCH, CLUB, set_spent=0.0, early=False)
    assert result.track_id and not result.reason
    assert [(t, a) for _w, t, a in suno.calls] == [("i0", kn.COVER_TRIES[0]), ("i1", kn.COVER_TRIES[1])]
    assert kn.COVER_TRIES[1].semitones != 0  # у пьесы с одной мелодией вторая попытка — всё равно другой исходник


@pytest.mark.parametrize("reason", ["no_credits", "budget_exhausted", "provider_down"])
def test_other_failures_are_not_retried(tmp_path, reason):
    suno = Suno(GeneratedMusicLibrary(str(tmp_path / "lib")), reject={MARCH.title}, reason=reason)
    assert cover_tries(suno.make, SOURCES[MARCH], MARCH, CLUB).reason == reason and len(suno.calls) == 1


def test_piece_rejected_everywhere_next_piece_takes_its_slots_files_never_repeat(tmp_path):
    """Приёмка #3648: марш не взят ни одной из ``COVER_TRIES`` попыток → его слоты берут другие произведения
    (Кантина заказана), заказы идут дальше, ни один файл не звучит дважды; фраза кода называет пьесу один раз."""
    said = []
    suno = Suno(GeneratedMusicLibrary(str(tmp_path / "lib")), reject={MARCH.title})
    gen = _set(tmp_path, suno, [MAIN, MARCH, CANTINA, MAIN, MARCH, CANTINA], speak=said.append)
    played = _play(gen, 6)
    ids = [item.id for item, _how in played]
    assert len(set(ids)) == 6 and all(how != "repeat" for _i, how in played)
    assert [c[0] for c in suno.calls].count(MARCH.title) == len(kn.COVER_TRIES)  # попытки — один раз на сет
    assert CANTINA.title in {c[0] for c in suno.calls} and not gen.gen_reason
    assert MARCH.title not in {item.title.rsplit(" ", 1)[0] for item, _how in played}
    assert said == [cover_rejected_text(MARCH, True)] == ["«Имперский марш» сервис не взял — играю другие."]
    assert gen.ordered <= kn.GEN_SET_MAX_TASKS and gen.spent == 12.0 * len(gen.task_ids)


def test_respread_least_played_piece_not_a_neighbour():
    final = CoverWork("Star Wars - Final", "starwars_2")
    plan = [key(w) for w in (MAIN, MARCH, CANTINA, final, MARCH, MAIN)]
    alive = lambda style: [cover_slot(style, w.title) for w in (MAIN, CANTINA, final)]  # noqa: E731
    out = gs.respread(plan, 1, {key(MARCH)}, alive)
    assert out == [key(w) for w in (MAIN, final, CANTINA, final, CANTINA, MAIN)]
    assert gs.respread(plan, 1, {key(MARCH)}, lambda style: [])[1] == "club"  # других нет — инструментал стиля


def test_all_pieces_rejected_set_falls_back_to_style_instrumental_not_repeats(tmp_path):
    """Ни одно произведение темы Suno не взял: слоты — инструментал стиля по тегам (новые файлы), не повтор."""
    said, kie = [], Kie(seconds=90.0)
    suno = Suno(GeneratedMusicLibrary(str(tmp_path / "lib")), reject={MAIN.title, MARCH.title})
    gen = _set(tmp_path, suno, [MAIN, MARCH, MAIN, MARCH], pieces=(MAIN, MARCH), speak=said.append, kie=kie)
    played = _play(gen, 4)
    assert len({item.id for item, _how in played}) == 4 and {item.genre for item, _how in played} == {"club"}
    assert [j.kind for j in kie.jobs] == ["instrumental"] * 2
    assert said == [cover_rejected_text(MAIN, True), cover_rejected_text(MARCH, False)]


def test_no_credits_still_stops_ordering_as_before(tmp_path):
    said = []
    suno = Suno(GeneratedMusicLibrary(str(tmp_path / "lib")), reject={MARCH.title}, reason="no_credits")
    gen = _set(tmp_path, suno, [MAIN, MARCH, CANTINA, MAIN], speak=said.append)
    _play(gen, 2)
    assert gen.gen_reason == "no_credits" and CANTINA.title not in {c[0] for c in suno.calls}
    assert len(said) == 1 and said[0].startswith("Дописать") and "не вышло" in said[0]


def test_maker_remembers_the_rejected_source_and_does_not_order_it_again(tmp_path):
    """Suno отклонил исходник (``state=fail`` после заказа) — sha рендера в ``gen_rejected``; тот же исходник второй
    раз не заказывается (ни в сете, ни пьесой), другой рендер того же трека — заказывается."""
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    kie = CoverKie(stages=("failed",))
    maker = CoverMaker(kie, lambda: lib, workdir=str(tmp_path))
    assert maker.make(_track(), SITAR, SourceAsk()).reason == "rejected" and len(kie.submitted) == 1
    rows = lib._conn.execute("SELECT task_id, work_id FROM gen_rejected").fetchall()  # noqa: SLF001
    assert [tuple(r) for r in rows] == [("TASK9", SITAR.title)]
    assert maker.make(_track(), SITAR, SourceAsk()).reason == "rejected" and len(kie.submitted) == 1
    maker.make(_track(), SITAR, SourceAsk(), attempt=kn.COVER_TRIES[1])
    assert len(kie.submitted) == 2 and kie.submitted[1].audio_weight == kn.COVER_TRIES[1].audio_weight
    assert lib.ledger.spent_since("2000-01-01") == 0.0  # отказ по каталогу кредитов не тратит


def test_submit_time_rejection_is_not_remembered(tmp_path):
    """Отказ на самом заказе (параметры, не детектор каталога) — не признак исходника: не запоминается."""
    from rob_box_mcp_tools.engine.gen.provider import GenUnavailable

    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    maker = CoverMaker(CoverKie(fail=GenUnavailable("rejected", "422")), lambda: lib, workdir=str(tmp_path))
    assert maker.make(_track(), SITAR, SourceAsk()).reason == "rejected"
    assert lib._conn.execute("SELECT COUNT(*) FROM gen_rejected").fetchone()[0] == 0  # noqa: SLF001
