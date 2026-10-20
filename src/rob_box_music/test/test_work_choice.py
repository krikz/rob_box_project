"""ADR-0157 PR-3 (#3560): кандидаты произведения по композитору и названию, детерминированный выбор (§3.2, §3.3)."""

from rob_box_music import knowledge as kn
from rob_box_music.choice import Choice, ChoiceQuestion, DeterministicChooser, Option, recency, score
from rob_box_music.works import FIELD_CONSUMERS, TITLE_EXACT, TITLE_PARTIAL, WorkRef, WorkWord, key_in_title, key_slot

BACH = WorkWord(("bach",))
TOCCATA = WorkWord(("toccata",))

#: Находки поиска «баха» на роботе 12.10 (theme_search): паразиты Panjabi MC — первыми.
RECORDS = {
    "mundiant_2": {"title": "Mundian To Bach Ke", "artist": "Panjabi MC"},
    "brandenb": {"title": "Brandenburg Concert No. 3", "artist": "Bach"},
    "unknown_75": {"title": "J.S.Bach", "artist": "J.S.Bach"},
    "toccataa": {"title": "Toccata And Fugue", "artist": "Bach"},
    "bachelor": {"title": "Bachelor Song", "artist": "Bachelors"},
}


def test_composer_field_has_a_consumer():
    assert FIELD_CONSUMERS["composer"]


def test_bach_is_a_composer_field_not_a_word_of_a_foreign_title():
    """R3: «Bach» в названии «Mundian To Bach Ke» (Panjabi MC) — не Бах; «Bachelors» — не «bach» (слово целиком)."""
    ids = [c.id for c in WorkRef(composer=(BACH,)).rtttl(RECORDS)]
    assert ids == ["brandenb", "unknown_75", "toccataa"]


def test_key_words_are_a_slot_and_never_match_minority():
    """«Токката ре минор»: «ре минор» — слот D minor, а не слова поиска (Minority, Green Day, прогон 09.10 #17)."""
    work, key = key_slot("токкату ре минор")
    assert (work, key) == ("токкату", "D minor")
    records = {"minority": {"title": "Minority", "artist": "Green Day"}, **RECORDS}
    cands = {c.id: c for c in WorkRef(title=(TOCCATA,), key=key).rtttl(records)}
    assert cands["toccataa"].title_match == TITLE_PARTIAL  # название совпало, тональности в нём нет
    assert cands["minority"].title_match == 0.0


def test_key_in_title_reads_english_and_german_spellings():
    assert key_in_title("D minor", "Toccata and Fugue in D Minor, BWV 565")
    assert key_in_title("D minor", "Toccata d-moll")
    assert key_in_title("A# major", "Sonata in B flat major")
    assert key_in_title("C# minor", "Nocturne in C# Minor")
    assert not key_in_title("D minor", "Toccata in D Major")
    assert not key_in_title("D minor", "Minority")
    assert not key_in_title("", "Toccata in D minor")


def test_title_match_is_exact_only_with_the_named_key():
    ref = WorkRef(title=(TOCCATA,), key="D minor")
    assert ref.title_match("Toccata and Fugue in D Minor", 1) == TITLE_EXACT
    assert ref.title_match("Toccata in C", 1) == TITLE_PARTIAL
    assert ref.title_match("Prelude", 0) == 0.0
    assert WorkRef(title=(TOCCATA,)).title_match("Toccata", 1) == TITLE_EXACT


def _option(oid, title=0.0, composer=0.0, fit=0.0, recent=0.0, rating=0.0):
    return Option(oid, {"title_match": title, "composer_match": composer, "fit": fit, "recent": recent,
                        "rating": rating})


class FakeModel:
    """Модельный провайдер под контрактом (PR-7 подставит ``llm_harness``/``jev``): отвечает заданным ``id``."""

    def __init__(self, answer=None, error=None):
        self.answer, self.error, self.calls = answer, error, 0

    def choose(self, question, *, deadline_ms=kn.CHOOSER_DEADLINE_MS):
        self.calls += 1
        if self.error:
            raise self.error
        return Choice(self.answer, 0.9, "fake")


def test_score_is_the_weighted_sum_of_the_adr():
    assert score(_option("x", title=1, composer=1, fit=1, recent=1, rating=1).features) == 3 + 2 + 1 - 2 + 0.5


def test_clear_gap_answers_without_calling_the_model():
    model = FakeModel("b")
    q = ChoiceQuestion("токката", (_option("a", title=1, fit=1), _option("b", title=0.5)))
    choice = DeterministicChooser(model).choose(q)
    assert choice == Choice("a", 1.0, "deterministic") and model.calls == 0


def test_ambiguous_question_goes_to_the_model_and_its_answer_must_be_a_candidate():
    q = ChoiceQuestion("баха", (_option("a", composer=1), _option("b", composer=1)))
    assert DeterministicChooser(FakeModel("b")).choose(q).id == "b"
    assert DeterministicChooser(FakeModel("messiah")).choose(q).provider == "deterministic"  # вне перечня — нет
    assert DeterministicChooser(FakeModel(error=TimeoutError())).choose(q).id == "a"
    assert DeterministicChooser().choose(q) == Choice("a", 0.5, "deterministic")


def test_three_bach_records_with_history_a_b_choose_c():
    """Приёмка #3560: при трёх записях Баха и истории ``[a, b]`` выбирается ``c``; очередь — c, затем давнее."""
    recent = ["b", "a"]
    q = ChoiceQuestion("баха", tuple(_option(x, composer=1, recent=recency(x, recent)) for x in "abc"))
    choice, order = DeterministicChooser().order(q)
    assert choice.id == "c" and order == ["c", "a", "b"]


def test_play_stats_counts_versions_as_one_within_the_window_and_penalty_is_the_codes_formula():
    from rob_box_music.choice import Plays, penalty, play_stats, score

    now = 100000.0
    played = [("tetris_2", now - 120), ("other", now - 500), ("tetris", now - 3000), ("tetris", now - 90000)]
    same = {"tetris": "k", "tetris_2": "k", "other": "o"}
    assert play_stats("tetris", played, now, same) == Plays(2, 2)  # версии — одна мелодия; старше окна не считается
    assert play_stats("fresh", played, now, same) == Plays(0, None)
    features = {"title_match": 1.0, "recent": 0.5}
    assert penalty(features) == 1.0 and score(features) == score({"title_match": 1.0}) - penalty(features)


def test_dice_prefers_higher_score_and_penalised_repeat_falls_rarer_with_fixed_seed():
    """#3564 (решение Шифу 13.10): код — «коэффициент и кубик», не argmax: вероятность ∝ exp(score/T); недавний повтор
    (штраф −2) выпадает в ≈e² раз реже свежего; детерминирован при заданном ``rng``."""
    import math
    import random
    from collections import Counter

    from rob_box_music.choice import DICE, dice, dice_weights

    q = ChoiceQuestion("баха", (Option("fresh_a", {"composer_match": 1.0}),
                                Option("fresh_b", {"composer_match": 1.0}),
                                Option("played", {"composer_match": 1.0, "recent": 1.0})))
    weights = dice_weights(q)
    assert weights[0] == weights[1] == 1.0
    assert weights[2] == math.exp(kn.CHOOSER_WEIGHTS["recent"] / kn.CHOOSER_DICE_TEMPERATURE)
    rng = random.Random(3564)
    counts = Counter(dice(q, rng).id for _ in range(4000))
    assert counts["played"] < counts["fresh_a"] / 4 and counts["played"] < counts["fresh_b"] / 4
    assert counts["played"] > 0  # не запрет, а редкость
    first = dice(q, random.Random(7))
    assert first == dice(q, random.Random(7)) and first.provider == DICE
    assert first.confidence == weights[q.ids().index(first.id)] / sum(weights)


def test_weighted_pick_and_dice_share_one_roulette():
    from rob_box_music import diversity

    class Fixed:
        def __init__(self, value):
            self.value = value

        def random(self):
            return self.value

    assert diversity.roulette(["a", "b", "c"], [1.0, 2.0, 1.0], Fixed(0.3)) == "b"
    assert diversity.weighted_pick(["a", "b"], [], Fixed(0.6)) == diversity.roulette(["a", "b"], [1.0, 1.0], Fixed(0.6))
