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
