"""Падеж имени (ревью PR #4108, пачка мелочи 4): «зови меня Тамарой» - это имя «Тамара».

Человек просит звать иначе в творительном («зови меня Тамарой», «называй меня Сашей»), а профиль голоса и гейт
``register_speaker`` ждут именительный: без приведения фраза «зови меня Тамарой» давала бы имя «Тамарой», гейт сравнивал бы
его с ``name='Тамара'`` и ложно отказывал «себя ты так не называл». Здесь - решения кода: ``words.nominative_name`` (одно
место, таблицы ``knowledge.NAME_CASE_*``), ключ сравнения ``words.name_key`` / ``words.same_name`` и их потребители
(``reply_grammar.classify_rename``, ``speaker_adapter.named_self`` / ``rename_step``). Сам гейт на проводе - в
``rob_box_mcp_tools/test/test_issue_4054_register_gate_v2.py``.
"""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.reply_grammar import Rename, classify_rename, classify_reply
from rob_box_dialog.speaker_adapter import named_self, rename_step
from rob_box_dialog.words import (introduced_name, name_key, name_tokens, nominative_name, profile_name, same_name,
                                  self_intro_name)

TAMARA = {"is_known": True, "speaker_id": "e35517b2", "name": "Тамара", "confidence": 0.93}


@pytest.mark.parametrize("word, nominative", [
    ("тамарой", "тамара"), ("сашей", "саша"), ("машей", "маша"), ("катей", "катя"), ("женей", "женя"),
    ("ильей", "илья"), ("любовью", "любовь"), ("анной", "анна"),
    ("борисом", "борис"), ("сергеем", "сергей"), ("андреем", "андрей"), ("игорем", "игорь"),
    ("дмитрием", "дмитрий"), ("артемом", "артем"), ("петром", "петр"),
    ("тамару", "тамара"), ("сашу", "саша"), ("юлю", "юля"),
    ("аскаром", "аскар"), ("никитой", "никита"),
], ids=str)
def test_instrumental_and_accusative_name_goes_to_nominative(word, nominative):
    """Творительный (и винительный: «запиши меня как Тамару») → именительный: по таблице окончаний с проверкой по словарю
    имён; незнакомое имя - по однозначным «-ой»/«-ом»."""
    assert nominative_name(word) == nominative


@pytest.mark.parametrize("word", [
    "тамара", "саша", "андрей", "сергей", "тимофей", "артем", "игорь", "любовь", "аскар", "шифу", "денчик", "том", "ной",
    "эйджик", "робби",
], ids=str)
def test_nominative_name_stays_as_it_is(word):
    """Имя в именительном (словарное, незнакомое, короткое) не трогается: «Тимофей» не «Тимофя», «Шифу» не «Шифа»."""
    assert nominative_name(word) == word


def test_every_dictionary_name_survives_the_normalization():
    """Словарь имён - именительные: приведение к именительному ни одно из них не меняет."""
    assert all(nominative_name(name) == name for name in kn.KNOWN_FIRST_NAMES)


# --- ревью PR #4118: винительный мужских имён, беглая гласная, женские имена, совпавшие с родительным ----------------


@pytest.mark.parametrize("word, nominative", [
    ("бориса", "борис"), ("сергея", "сергей"), ("игоря", "игорь"), ("андрея", "андрей"), ("дмитрия", "дмитрий"),
    ("артема", "артем"), ("петра", "петр"), ("николая", "николай"), ("макара", "макар"), ("олега", "олег"),
], ids=str)
def test_masculine_accusative_goes_to_nominative(word, nominative):
    """«Запиши меня как Бориса», «переименуй меня в Сергея»: винительный мужского одушевлённого = родительный (-а/-я)."""
    assert nominative_name(word) == nominative


@pytest.mark.parametrize("word, nominative", [
    ("львом", "лев"), ("льва", "лев"), ("павлом", "павел"), ("павла", "павел"),
], ids=str)
def test_name_with_a_fleeting_vowel_goes_to_its_dictionary_form(word, nominative):
    """«Львом» → «лев», а не «льв» (``NAME_CASE_STEMS``): основа косвенных падежей сверяется со словарём через таблицу."""
    assert nominative_name(word) == nominative


@pytest.mark.parametrize("word", [
    "александра", "валерия", "виталия", "влада", "владислава", "вячеслава", "ярослава", "евгения", "валентина",
], ids=str)
def test_female_name_written_like_a_masculine_genitive_stays_female(word):
    """«Меня зовут Валерия» - женское имя, не «Валерий»: такие имена стоят в словаре и как есть (именительный)."""
    assert word in kn.KNOWN_FIRST_NAMES and nominative_name(word) == word
    assert introduced_name(name_tokens(f"меня зовут {word}")) == word


@pytest.mark.parametrize("word", ["кузьма", "фома", "шифа", "станислава", "эльдара", "кроватью", "льву"], ids=str)
def test_unknown_word_with_an_ambiguous_ending_is_left_as_it_is(word):
    """Слово, которого нет в словаре имён и которое не «-ой»/«-ом» (``_sure_form``), не трогается: гадать по «-а»/«-я»/
    «-ью» код не должен. Дательный («льву») - не падеж просьбы «зови меня», его тоже нет."""
    assert nominative_name(word) == word


def test_same_name_in_nominative_and_accusative():
    assert same_name("Борис", "Бориса") and same_name("Сергей", "сергея") and same_name("Лев", "Львом")
    assert not same_name("Борис", "Борисов") and not same_name("Валерия", "Валерий")


def test_the_profile_name_is_the_nominative_in_one_place():
    """``profile_name`` - имя для профиля голоса: падеж приводится, заглавная, ё в именительном сохраняется."""
    assert [profile_name(n) for n in ("Тамарой", " бориса ", "сергеем", "Львом", "Аскаром")] == [
        "Тамара", "Борис", "Сергей", "Лев", "Аскар"]
    assert [profile_name(n) for n in ("тамара", "Алёна", "Валерия", "Иван Петров", "ИВАН ПЕТРОВ", "", None)] == [
        "Тамара", "Алёна", "Валерия", "Иван петров", "Иван петров", "", ""]


@pytest.mark.parametrize("text, name", [
    ("запиши меня как Бориса", "Борис"),
    ("переименуй меня в Сергея", "Сергей"),
    ("запомни меня как Игоря", "Игорь"),
    ("Робот, переименуй меня в Льва", "Лев"),
    ("зови меня Валерией", "Валерия"),
    ("запиши меня как Валерия", "Валерия"),
], ids=lambda value: str(value)[:40])
def test_rename_in_accusative_names_the_person_in_nominative(text, name):
    """Фраза переименования: и ``classify_rename``, и ``named_self`` (его читает гейт ``register_speaker``) - именительный."""
    assert classify_rename(text) == Rename(name)
    assert named_self(text) == name


@pytest.mark.parametrize("text, name", [
    ("зови меня Тамарой", "Тамара"),
    ("называй меня Сашей", "Саша"),
    ("Робот, зови меня Борисом", "Борис"),
    ("запиши меня как Тамару", "Тамара"),
    ("зови меня Тамара", "Тамара"),
    ("зови меня Аскаром", "Аскар"),
    ("зови меня Тимофей", "Тимофей"),
], ids=lambda value: str(value)[:40])
def test_call_me_in_any_case_names_the_person_in_nominative(text, name):
    """Просьба звать иначе: имя в ответе грамматики и в ``named_self`` (его читает гейт ``register_speaker``) - именительное."""
    assert classify_rename(text) == Rename(name)
    assert named_self(text) == name


def test_correction_names_both_names_in_nominative():
    """«Меня зовут не Сашей, а Тамарой»: и новое имя, и поправляемое - в именительном (профиль «Саша» ищется по нему)."""
    assert classify_rename("меня зовут не Сашей, а Тамарой") == Rename("Тамара", "Саша")
    assert classify_rename("меня зовут не Нина, а Тамара") == Rename("Тамара", "Нина")


def test_introduction_in_instrumental_is_nominative_too():
    """Одна реализация ``introduced_name``: «зови меня Тамарой» находит и ``self_intro_name`` (строчное), и
    ``classify_reply`` (ответ на «как тебя зовут?»)."""
    assert self_intro_name("зови меня Тамарой") == "тамара"
    reply = classify_reply("зови меня Тамарой")
    assert (reply.kind, reply.name) == ("name", "Тамара")


@pytest.mark.parametrize("left, right", [
    ("Тамара", "Тамарой"), ("тамарой", "Тамара"), ("Саша", "Сашей"), ("Алёна", "Алена"), ("Борис", "  Борисом "),
    ("Тамара", "Тамара"),
], ids=str)
def test_names_in_different_case_are_one_name(left, right):
    assert same_name(left, right) and name_key(left) == name_key(right)


@pytest.mark.parametrize("left, right", [
    ("Тамара", "Нина"), ("Саша", "Саши"), ("Борис", "Борисов"), ("", ""), (None, None), ("Тамара", ""), (None, "Тамара"),
], ids=str)
def test_different_or_empty_names_are_not_the_same(left, right):
    """Разные имена остаются разными; пустое имя не равно ничему, даже пустому."""
    assert not same_name(left, right)


def test_multiword_name_is_compared_as_it_is():
    """«Иван Петров» - не одно слово: падеж не трогаем, только регистр и ё."""
    assert name_key("Иван Петров") == "иван петров"
    assert same_name("Иван Петров", "иван петров") and not same_name("Иван Петров", "Иваном Петровым")


def test_my_own_name_in_instrumental_is_not_a_rename():
    """Голос уверенно «Тамара» + «зови меня Тамарой»: имя то же - ``identity.name_same``, а не переименование в «Тамарой»."""
    step = rename_step(classify_rename("зови меня Тамарой"), TAMARA)
    assert step is not None and step.rename == ()
    assert "Тамара" in step.phrase and "Тамарой" not in step.phrase


def test_other_name_in_instrumental_renames_to_nominative():
    """Голос уверенно «Тамара» + «называй меня Сашей»: профиль переименовывается в «Саша», не в «Сашей»."""
    step = rename_step(classify_rename("называй меня Сашей"), TAMARA)
    assert step is not None and step.rename == ("e35517b2", "Саша")
