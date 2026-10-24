"""#3890: корпус фактов марафона (акты 2, 9, 10) и запросы к нему.

Факты — как их сохраняет LLM по репликам ``scripts/e2e/gen_night_marathon.py``
(n203 Саша, n206 Борис). Запросы: ключевые слова (находят) и описательные с
перечислением через запятую (раньше слово с запятой искалось как подстрока
«слово,» и не находилось). Ложные срабатывания: запрос про то, чего нет в
памяти, и факт чужого человека не возвращаются.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.voice_memory import VoiceMemory, query_words

SASHA = "spk-sasha"
BORIS = "spk-boris"

CORPUS = [
    (SASHA, "Саша пьёт только зелёный чай без сахара"),
    (SASHA, "Саша не ест лук, даже жареный"),
    (BORIS, "Борис болеет за Спартак"),
    (BORIS, "Борис всегда приносит пиццу"),
]

#: (запрос, speaker, подстрока ожидаемого факта или None если должно быть пусто)
QUERIES = [
    # ключевые слова
    ("чай", SASHA, "чай"),
    ("лук", SASHA, "лук"),
    ("Спартак", BORIS, "Спартак"),
    ("пиццу", BORIS, "пиццу"),
    ("пицца", BORIS, None),  # другая форма: поиск без стемминга честно пуст
    # описательные, с запятыми и перечислением
    ("чай, чайные предпочтения", SASHA, "чай"),
    ("что я не ем, лук", SASHA, "лук"),
    ("Спартак, футбол", BORIS, "Спартак"),
    # ложные срабатывания
    ("футбол", BORIS, None),
    ("кофе", SASHA, None),
    ("чай", BORIS, None),  # чужой факт
    ("Спартак", SASHA, None),  # чужой факт
]


@pytest.fixture()
def memory(tmp_path):
    m = VoiceMemory(db_path=str(tmp_path / "vm.db"), session_id="issue_3890")
    for spk, fact in CORPUS:
        m.save_fact(fact, category="preference", speaker_id=spk)
    yield m
    m.close()


@pytest.mark.parametrize("query,speaker,expected", QUERIES)
def test_corpus_query(memory, query, speaker, expected):
    hits = memory.search(query, limit=5, speaker_id=speaker)
    if expected is None:
        assert hits == []
    else:
        assert any(expected in h["content"] for h in hits), hits
        assert all(h["speaker_id"] == speaker for h in hits)


def test_query_words_drops_punctuation():
    assert query_words("Чай, чайные; предпочтения!") == ["чай", "чайные", "предпочтения"]
    assert query_words(" , ") == []
