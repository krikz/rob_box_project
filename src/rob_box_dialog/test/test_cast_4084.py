"""#4084: голоса по ролям — число голосов = числу ролей, у роли и рассказчика один голос на всё выступление; песня и
рэп — голосом робота.

Живой случай 27.10 (dev-0d94634, Шифу у робота): «Робот, спой песню про енота» — куплеты через один голосом
``Russian_ReliableMan`` и ``Russian_HandsomeChildhoodFriend``; анекдот про мужика и робота — пять голосов, «мужик» —
третьим голосом, рассказчик то один, то другой. Причина: голос каждого сегмента выбирала модель
(``voice_per_segment``). Теперь модель называет героев (``cast``) и чья реплика в сегменте (``role_per_segment``), а
голоса даёт код (:mod:`rob_box_dialog.cast`, ADR-0148).

Тесты проверяют поведение: команда модели → валидатор (голоса решает код) → владелец выступления (``PerformBlock``) →
голоса в запросах речи (``Say.voice``).
"""

import pytest

from perform_sim import Sim

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import cast as cs
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Busy, Command, Form, parse, validate
from rob_box_dialog.plan.perform import PerformBlock, Say

ALL = [t.name for t in TOOL_CATALOG]
#: Каталог ролей MiniMax (русские голоса без рассказчика и каста марафона) — как его отдаёт ``plan_host.perform_busy``.
POOL = (
    ("Russian_ReliableMan", "male"), ("Russian_HandsomeChildhoodFriend", "male"), ("Russian_AttractiveGuy", "male"),
    ("Russian_Bad-temperedBoy", "male"), ("Russian_BrightHeroine", "female"), ("Russian_AmbitiousWoman", "female"),
    ("Russian_CrazyQueen", "female"), ("Russian_PessimisticGirl", "female"),
)
BUSY = Busy(voices=tuple(v for v, _ in POOL), voice_genders=POOL)

#: Песня про енота 27.10 — как её прислала модель: голос «на куплет».
RACCOON = {"kind": "song", "backing": "calm", "segments": [
    "Ночью, когда весь город спит, енот выходит на промысел.",
    "Полосатый хвост, как тельняшка, шуршит по мусорным бакам.",
    "Енот-енот, ночной пилот, по крышам он идёт.",
    "Утром лужица, усы в сметане — енот спит на диване.",
    "Енот-енот, ночной пилот, по крышам он идёт.",
], "voice_per_segment": ["Russian_ReliableMan", "Russian_HandsomeChildhoodFriend"] * 2 + ["Russian_ReliableMan"]}

HEDGEHOG_AND_FOX = {"kind": "tale", "backing": "calm", "segments": [
    "Жил-был в лесу ёжик.",
    "Лиса, давай дружить!",
    "Давай, колючий, только не уколи.",
    "И пошли они вместе по тропинке.",
    "Смотри, лиса, грибы!",
    "Чур, мои!",
    "Так и подружились ёж и лиса.",
], "cast": [{"name": "Ёж", "gender": "male"}, {"name": "Лиса", "gender": "female"}],
    "role_per_segment": ["", "Ёж", "лиса", "рассказчик", "ёж", "Лиса", ""]}

#: Анекдот 27.10 14:28: мужик, продавец, робот и рассказчик — модель тогда дала пять голосов.
JOKE = {"kind": "tale", "segments": [
    "Мужик приходит в магазин электроники.", "Продавец: — Конечно! Вот робот.", "Мужик: — Отлично, беру.",
    "Продавец: — Не знаю, что он умеет.", "Мужик: — А что, у вас в библиотеке нет инструкции?",
    "Робот гордо: — Я сам инструкция!", "Мужик: — Ладно, включай.", "Робот: — Уже включился.",
    "Так мужик и ушёл с роботом.",
], "cast": [{"name": "мужик", "gender": "male"}, {"name": "продавец", "gender": "male"},
            {"name": "робот", "gender": "male"}],
    "role_per_segment": ["", "продавец", "мужик", "продавец", "мужик", "робот", "мужик", "робот", ""]}


def _command(perform, busy=BUSY, form=None):
    res = validate(parse({"perform": perform}), ALL, busy=busy, form=form)
    assert isinstance(res, Command), res
    return res


def _voices_heard(perform, busy=BUSY, form=None):
    """Голоса запросов пред-синтеза владельца выступления — по одному на сегмент, в порядке звучания. Пред-синтез
    уходит окном (#4151), поэтому выступление прогоняется целиком на фейковых владельцах (:mod:`perform_sim`)."""
    sim = Sim(_command(perform, busy, form).perform)
    sim.start()
    sim.run(600)
    return [say.voice for _, say in sim.effects if isinstance(say, Say) and not say.key and not say.commit]


# --- песня и рэп — голосом робота ---------------------------------------------------------------------------------


def test_raccoon_song_of_27_10_is_sung_in_one_robot_voice():
    assert _voices_heard(RACCOON) == [""] * len(RACCOON["segments"])


def test_rap_of_seven_segments_is_one_voice_even_with_heroes_named_by_the_model():
    rap = {"kind": "rap", "backing": "beat", "segments": [f"Куплет {i}: енот на бите." for i in range(7)],
           "voice_per_segment": ["Russian_ReliableMan", "Russian_BrightHeroine"] * 3 + [""],
           "cast": [{"name": "Енот", "gender": "male"}], "role_per_segment": ["Енот", ""] * 3 + ["Енот"]}
    voices = _voices_heard(rap)
    assert len(voices) == 7 and set(voices) == {""}


def test_song_without_roles_is_one_voice():
    song = {"kind": "song", "backing": "calm", "segments": ["Ля-ля, енот.", "Ля-ля, полоскун.", "Ля-ля, спать."]}
    assert set(_voices_heard(song)) == {""}


def test_live_sing_route_offers_no_roles_and_still_sings_in_one_voice():
    """Путь живого пения (#4079) — схема без ``cast``/``role_per_segment``; модель их всё же прислала — голос робота."""
    live = Form(sing="live:song")
    assert "cast" not in kn.with_sing(kn.command_schema("json"), live.sing)["properties"]["perform"]["properties"]
    assert set(_voices_heard(RACCOON, form=live)) == {""}


# --- сказка и анекдот — голосов ровно по ролям ------------------------------------------------------------------


def test_hedgehog_and_fox_with_a_narrator_is_three_voices_each_role_constant():
    voices = _voices_heard(HEDGEHOG_AND_FOX)
    roles = [cs.role_key(r) for r in HEDGEHOG_AND_FOX["role_per_segment"]]
    assert len(set(voices)) == 3  # рассказчик + ёж + лиса
    by_role = {}
    for role, voice in zip(roles, voices):
        assert by_role.setdefault(role, voice) == voice, (role, voice, by_role)
    assert by_role[""] == ""  # рассказчик — голос робота
    assert dict(POOL)[by_role[cs.role_key("Ёж")]] == "male" and dict(POOL)[by_role["лиса"]] == "female"


def test_joke_of_27_10_is_four_voices_and_the_man_keeps_his_voice():
    voices = _voices_heard(JOKE)
    roles = [cs.role_key(r) for r in JOKE["role_per_segment"]]
    assert len(set(voices)) == 4  # рассказчик + мужик + продавец + робот
    for role in ("мужик", "продавец", "робот", ""):
        assert len({v for r, v in zip(roles, voices) if r == role}) == 1, role
    assert "" not in {v for r, v in zip(roles, voices) if r}  # герой не звучит голосом рассказчика


def test_an_edit_does_not_recolour_the_heroes_already_heard():
    """«А пусть лиса...» посреди сказки: правка с другим порядком героев и новым героем — прежние голоса у прежних."""
    block = PerformBlock(_command(HEDGEHOG_AND_FOX).perform, "b-4084", log=lambda _: None)
    busy = Busy(frozenset({"action.perform"}), active_segment=2, voices=BUSY.voices, voice_genders=POOL,
                cast=tuple(block.cast.items()))
    edit = {"kind": "tale", "segments": ["Пришёл волк.", "Лиса, беги!", "Не боюсь!"],
            "cast": [{"name": "Волк", "gender": "male"}, {"name": "Лиса", "gender": "female"},
                     {"name": "Ёж", "gender": "male"}], "role_per_segment": ["", "Ёж", "Лиса"]}
    res = validate(parse({"edit_pending": True, "perform": edit}), ALL, busy=busy)
    assert isinstance(res, Command), res
    assert res.perform.voice_per_segment == ("", block.cast[cs.role_key("Ёж")], block.cast["лиса"])
    wolf = validate(parse({"edit_pending": True, "perform": {**edit, "role_per_segment": ["", "Волк", "Лиса"]}}),
                    ALL, busy=busy)
    assert wolf.perform.voice_per_segment[1] not in set(block.cast.values()) | {""}


# --- модуль ролей ---------------------------------------------------------------------------------------------------


def test_role_key_folds_case_yo_spaces_and_narrator_names():
    assert cs.role_key("  Ёжик  Колючий ") == "ежик колючий"
    assert [cs.role_key(n) for n in ("", "Рассказчик", "АВТОР", "narrator")] == ["", "", "", ""]


def test_assign_prefers_the_role_gender_then_neutral_then_any_and_never_repeats():
    pool = (("m1", "male"), ("n1", "neutral"), ("f1", "female"))
    cast = (cs.Role("a", "female"), cs.Role("b", "female"), cs.Role("c", "female"), cs.Role("d", "female"))
    assert cs.assign(cast, ["a", "b", "c", "d"], pool) == {"a": "f1", "b": "n1", "c": "m1", "d": ""}


def test_free_voices_are_the_catalogue_without_the_voices_of_the_running_heroes():
    pool = (("m1", "male"), ("n1", "neutral"), ("f1", "female"), ("f2", "female"))
    assert cs.free_voices(pool) == ["m1", "n1", "f1", "f2"]
    assert cs.free_voices(pool, {"ёж": "m1", "лиса": "f1"}) == ["n1", "f2"]
    assert cs.free_voices(pool, {"ёж": "gone-from-catalogue"}) == ["m1", "n1", "f1", "f2"]


def test_problem_counts_only_heroes_without_a_voice_against_the_free_voices():
    """Сверка голосов (ревью PR #4112): герой идущего выступления (``fixed``) нового голоса не просит, а его голос
    из каталога вычтен; каталог пуст - не сверяется."""
    pool = (("m1", "male"), ("f1", "female"), ("f2", "female"))
    cast = (cs.Role("ёж"), cs.Role("лиса"), cs.Role("волк"), cs.Role("сова"))
    roles = ["ёж", "лиса", "волк", "сова"]
    held = {cs.role_key("ёж"): "m1"}  # Busy.cast - по ключу роли (без регистра, ё = е)
    assert cs.problem("tale", 3, cast, roles[:3], pool) is None, "трём героям - три голоса"
    assert cs.problem("tale", 4, cast, roles, pool)[:2] == (cs.BAD_CAST, "героев с репликами 4, свободных голосов для них 3")
    assert cs.problem("tale", 3, cast, roles[:3], pool, held) is None, "ёж держит свой голос, лиса и волк - два свободных"
    assert cs.problem("tale", 4, cast, roles, pool, held)[:2] == (
        cs.BAD_CAST, "героев с репликами 3, свободных голосов для них 2 (остальные заняты героями идущего выступления)")
    assert cs.problem("tale", 4, cast, roles, (), held) is None, "каталог неизвестен - не сверяется"


def test_segment_voices_is_none_without_heroes_and_for_songs():
    cast = (cs.Role("лиса", "female"),)
    assert cs.segment_voices("tale", cast, ["", ""], POOL) is None
    assert cs.segment_voices("song", cast, ["лиса", ""], POOL) is None
    assert cs.segment_voices("poem", cast, ["лиса", ""], POOL) == ("Russian_BrightHeroine", "")


@pytest.mark.parametrize("kind", kn.PERFORM_KINDS)
def test_only_tales_and_poems_have_roles(kind):
    assert cs.has_roles(kind) == (kind in ("tale", "poem"))
