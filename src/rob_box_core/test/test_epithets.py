#!/usr/bin/env python3
"""test_epithets.py — клички спикеров (issue #1787): чистая логика «текст → теги → кличка».

Модуль переехал из ``rob_box_voice/core`` в ``rob_box_core`` (ADR-0164 этап 4) — знание о кличках в одном
месте, голос и лицо импортируют. Хранение клички в ``speakers.db`` проверяет
``rob_box_voice/test/unit/utils/test_epithet_storage.py``.
"""

from __future__ import annotations

import pytest

from rob_box_core import epithets as ep

# ── Словарь ──────────────────────────────────────────────────────────────────


def test_lexicon_has_no_duplicate_epithets():
    """Кличка обязана быть уникальной ГЛОБАЛЬНО, а не внутри кластера.

    Иначе два спикера из разных кластеров получили бы одинаковую метку и
    эпитет перестал бы решать задачу, ради которой заведён.
    """
    pool = [e for group in ep.EPITHET_LEXICON.values() for e in group]
    pool += list(ep.DEFAULT_POOL_NEUTRAL) + list(ep.DEFAULT_POOL_RESTLESS)
    duplicates = {e for e in pool if pool.count(e) > 1}
    assert not duplicates, f"дубли в словаре: {duplicates}"
    assert len(pool) >= 200, f"словарь мал: {len(pool)} < 200 (research §4.1)"


def test_every_keyword_cluster_has_epithets():
    """Кластер без эпитетов молча деградирует в дефолтный пул."""
    assert set(ep.CLUSTER_KEYWORDS) == set(ep.EPITHET_LEXICON)


# ── Теги и валентность ───────────────────────────────────────────────────────


def test_extract_tags_finds_dominant_topic():
    tags = ep.extract_tags(
        [
            "обожаю шахматы, особенно ферзевый гамбит",
            "вчера разобрал эндшпиль и пару дебютов на доске",
        ]
    )
    assert tags
    assert tags[0].cluster == "шахматы"
    assert tags[0].share == pytest.approx(1.0)


def test_extract_tags_ignores_too_short_input():
    """Одно слово — не тема. Порог MIN_WORDS_FOR_TAGS защищает от случайности."""
    assert ep.extract_tags(["шахматы"]) == []


def test_extract_tags_empty_when_no_keywords():
    assert ep.extract_tags(["ну не знаю даже что тебе на это сказать сегодня"]) == []


def test_score_sentiment_range():
    assert ep.score_sentiment(["всё бесит, устал, надоело"]) < 0
    assert ep.score_sentiment(["спасибо, отлично, очень нравится"]) > 0
    assert ep.score_sentiment(["стол стул окно"]) == 0.0
    assert ep.score_sentiment([]) == 0.0


# ── Выбор клички ─────────────────────────────────────────────────────────────


def test_epithet_is_deterministic_across_calls():
    """Одинаковый speaker_id → одинаковая кличка (crc32, не hash())."""
    tags = ep.extract_tags(["паяльник и микросхема, транзистор, прошивка платы"])
    first = ep.choose_epithet(tags, speaker_id="abc-123")
    second = ep.choose_epithet(tags, speaker_id="abc-123")
    assert first.label == second.label


def test_two_namesakes_same_topic_get_different_epithets():
    """Главный сценарий issue #1787: «два Дениса-шахматиста».

    Именно на нём research (§4.2) ломал чисто словарный подход. Здесь он
    закрыт передачей ``taken`` — коллизия невозможна по построению.
    """
    tags = ep.extract_tags(
        ["шахматы, дебют, эндшпиль", "ферзь, ладья, рокировка и гамбит"]
    )
    first = ep.choose_epithet(tags, speaker_id="denis-1")
    second = ep.choose_epithet(tags, speaker_id="denis-2", taken=[first.label])
    assert first.label != second.label
    assert first.source_cluster == second.source_cluster == "шахматы"


def test_epithet_falls_back_to_default_pool_without_topics():
    """Молчун без тем всё равно получает кличку сразу (research §5.1)."""
    candidate = ep.choose_epithet([], speaker_id="quiet")
    assert candidate.source_cluster == "default"
    assert candidate.label in ep.DEFAULT_POOL_NEUTRAL


def test_restless_sentiment_switches_default_pool():
    candidate = ep.choose_epithet([], speaker_id="quiet", sentiment=-0.9)
    assert candidate.label in ep.DEFAULT_POOL_RESTLESS


def test_exhausted_pool_gets_numeric_suffix():
    """Когда занято всё — уникальность важнее красоты."""
    taken = [e for group in ep.EPITHET_LEXICON.values() for e in group]
    taken += list(ep.DEFAULT_POOL_NEUTRAL) + list(ep.DEFAULT_POOL_RESTLESS)
    candidate = ep.choose_epithet([], speaker_id="last-one", taken=taken)
    assert candidate.label.endswith("-2")
    assert candidate.label not in taken


# ── Пересмотр ────────────────────────────────────────────────────────────────


def test_should_review_respects_interval():
    day = ep.SECONDS_PER_DAY
    assert ep.should_review(None, 1000.0) is True
    assert ep.should_review(0.0, 10 * day) is False
    assert ep.should_review(0.0, 31 * day) is True


def test_find_distinctive_topic_only_for_new_dominant_theme():
    tags = ep.extract_tags(
        ["шахматы, дебют, эндшпиль", "ферзь, ладья, рокировка и гамбит"]
    )
    # Та же тема, что уже в профиле → повода менять кличку нет.
    assert ep.find_distinctive_topic(["шахматы"], tags) is None
    # Новая доминирующая тема → повод есть.
    assert ep.find_distinctive_topic(["техно"], tags) == "шахматы"


def test_find_distinctive_topic_ignores_weak_theme():
    weak = [ep.TagScore(cluster="спорт", hits=1, share=0.1)]
    assert ep.find_distinctive_topic([], weak) is None


# ── Слой 2: кличка от LLM ────────────────────────────────────────────────────


def test_llm_prompt_mentions_topic_and_replies():
    prompt = ep.build_llm_prompt(
        ["разбирал эндшпиль", "люблю дебюты"],
        fallback="Гроссмейстер",
        cluster="шахматы",
        hints=["Гроссмейстер", "Магистр", "Кадет"],
    )
    assert "шахматы" in prompt
    assert "эндшпиль" in prompt
    # Кличка не для ушей юзера — это должно быть сказано модели прямо.
    assert "НИКОГДА не произносится вслух" in prompt


def test_llm_prompt_survives_empty_history():
    prompt = ep.build_llm_prompt([], fallback="Странник")
    assert "Странник" in prompt


def test_sanitize_llm_epithet_strips_quotes_and_signature():
    assert ep.sanitize_llm_epithet("  «Ночной паяльщик»  ") == "Ночной паяльщик"
    assert ep.sanitize_llm_epithet("Кличка: Ночной паяльщик") == "Ночной паяльщик"


def test_sanitize_llm_epithet_cuts_explanation_of_chatty_answer():
    """Модели любят добавить пояснение — берём только кличку до разделителя."""
    assert ep.sanitize_llm_epithet("Ночной паяльщик — он всё паяет") == "Ночной паяльщик"
    assert ep.sanitize_llm_epithet("Ночной паяльщик, любит моторы") == "Ночной паяльщик"
    assert (
        ep.sanitize_llm_epithet("Кличка: «Мудрый собеседник» (любит спорт)")
        == "Мудрый собеседник"
    )


def test_sanitize_llm_epithet_rejects_garbage():
    for bad in (
        None,
        "",
        "   ",
        "к",                    # одно слово
        "Сверхдлинноеслововыходящеезаграницы",
        "ночной паяльщик",      # с маленькой буквы — не кличка
        "Агент 007",            # цифры
        "🤖 Робот",             # эмодзи
        "Не могу придумать",    # отказ модели, а не кличка
        "Я бы назвал Кулибиным",
    ):
        assert ep.sanitize_llm_epithet(bad) is None, bad


def test_sanitize_llm_epithet_rejects_taken_label():
    assert (
        ep.sanitize_llm_epithet("Ночной паяльщик", taken=["ночной паяльщик"]) is None
    )


# ── Issue #2864: кличка «Незнакомец» у знакомого человека ────────────────────


@pytest.mark.parametrize("bad", [
    "Незнакомец",          # живой прогон: так LLM назвала представившегося Сашу
    "Незнакомка",
    "Неизвестный",
    "Неопознанный",
    "Аноним",
    "Инкогнито",
    "Безымянный",
    "Гость",
    "Гостья",
    "Чужак",
    "Чужестранец",
    "Посторонний",
    "Некто",
    "Stranger",
    "Кулибин-Незнакомец",  # часть составной клички
    "Незнакомёц",      # ё вместо е не спасает
    # Issue #2887 — многословная кличка: достаточно одного слова.
    "Мудрый незнакомец",
    "Тихий гость с гитарой",
    "Ночной аноним",
    "Паяльщик-чужак моторов",
])
def test_sanitize_llm_epithet_rejects_stranger_meaning(bad):
    """Кличка знакомого голоса не может означать «неизвестный человек».

    Иначе на «кого запомнил?» LLM считает её отдельным человеком
    (n211: «запомнил троих» при двух профилях в БД).
    """
    assert ep.sanitize_llm_epithet(bad) is None, bad
    assert ep.check_llm_epithet(bad)[1] == "stranger", bad


@pytest.mark.parametrize("good", [
    "Мудрый собеседник",
    "Тихий наблюдатель",
    "Вечный странник",
    "Ночной паяльщик моторов",
    # Issue #2887 — ложные срабатывания основы «гост» в многословной кличке.
    "Гостеприимный повар",
    "Гостиничный администратор",
])
def test_sanitize_llm_epithet_keeps_ordinary_labels(good):
    assert ep.sanitize_llm_epithet(good) == good


def test_llm_prompt_forbids_stranger_epithets():
    prompt = ep.build_llm_prompt(["привет"], fallback="Наблюдатель")
    assert "незнакомц" in prompt.lower()


# ── Issue #2886: у отказа есть причина ───────────────────────────────────────


@pytest.mark.parametrize("raw, taken, reason", [
    (None, (), "empty"),
    ("", (), "empty"),
    ("Агент 007", (), "invalid"),
    ("мудрый собеседник", (), "invalid"),
    ("Незнакомец", (), "stranger"),
    ("Мудрый собеседник", ("мудрый собеседник",), "taken"),
])
def test_issue_2886_check_llm_epithet_names_reject_reason(raw, taken, reason):
    label, why = ep.check_llm_epithet(raw, taken=taken)
    assert label is None
    assert why == reason


def test_issue_2886_sobesednik_is_not_a_stranger_label():
    """«Собеседник» не значит «незнакомец»: фильтр #2864 его пропускает,
    отказ Борису в issue #2886 — по занятости, не по фильтру."""
    assert ep.is_stranger_epithet("Собеседник") is False
    label, why = ep.check_llm_epithet("Мудрый собеседник", taken=())
    assert label == "Мудрый собеседник" and why is None


# ── Issue #2887: кличка 2–4 слова (решение Шифу 23.09.2026) ─────────────────


@pytest.mark.parametrize("good", [
    "Мудрый собеседник-спортсмен",      # пример из issue: дефис — не слово
    "Ночной паяльщик моторов",          # пример из issue
    "Мастер на все руки",               # 4 слова, предлог допустим
    "Шахматный Стратег",
])
def test_issue_2887_accepts_two_to_four_words(good):
    assert ep.check_llm_epithet(good) == (good, None)


@pytest.mark.parametrize("bad", [
    "Собеседник",                        # 1 слово — было нормой, теперь нет
    "Кулибин",
    "Звездочёт-Второй",                  # 1 слово через дефис
    "Мудрый старый добрый ночной паяльщик",  # 5 слов
    "Один два три четыре пять шесть",
])
def test_issue_2887_rejects_one_and_five_plus_words(bad):
    """Не обрезаем: обрывок фразы хуже словарной клички, которая уже в БД."""
    assert ep.check_llm_epithet(bad) == (None, "word_count")


def test_issue_2887_rejects_overlong_label():
    label = "Сверхвнимательный многоуважаемый электротехник-испытатель"
    assert len(label) > ep.LLM_EPITHET_MAX_LEN
    assert ep.check_llm_epithet(label) == (None, "invalid")


def test_issue_2887_taken_is_checked_on_whole_label():
    """Занятость — по всей кличке целиком, без учёта регистра, ё и пробелов."""
    taken = ["Мудрый собеседник"]
    assert ep.check_llm_epithet("Мудрый  Собеседник", taken=taken) == (None, "taken")
    assert ep.check_llm_epithet("Мудрый собеседник", taken=["мудрый собеседник"])[1] == "taken"
    assert ep.check_llm_epithet("Тёплый собеседник", taken=["Теплый собеседник"])[1] == "taken"
    # Общее слово — не занятость: клички разные.
    assert ep.check_llm_epithet("Мудрый собеседник-спортсмен", taken=taken)[0] == (
        "Мудрый собеседник-спортсмен"
    )


@pytest.mark.parametrize("label, names", [
    ("Путешественник Саша", ["Саша"]),
    ("Борисыч путешественник", ["Борис"]),
    ("Весёлый Борис-путешественник", ["Саша", "Борис"]),  # чужое имя
])
def test_issue_2887_rejects_person_name(label, names):
    assert ep.check_llm_epithet(label, names=names) == (None, "name")


def test_issue_2887_short_name_matches_only_whole_word():
    """«Лев» не должен резать «Левша»: короткие имена — только целиком."""
    assert ep.check_llm_epithet("Мудрый левша", names=["Лев"])[0] == "Мудрый левша"
    assert ep.check_llm_epithet("Мудрый Лев", names=["Лев"]) == (None, "name")


def test_issue_2887_prompt_asks_for_two_to_four_words_from_traits():
    prompt = ep.build_llm_prompt(["я бегаю марафоны"], fallback="Атлет", cluster="спорт")
    assert "ОДНО слово" not in prompt
    assert "2–4 слов" in prompt
    assert "Мудрый собеседник-спортсмен" in prompt
    assert "рассказал о себе" in prompt
    assert "имя" in prompt


def _all_dictionary_labels():
    labels = [e for group in ep.EPITHET_LEXICON.values() for e in group]
    return labels + list(ep.DEFAULT_POOL_NEUTRAL) + list(ep.DEFAULT_POOL_RESTLESS)


def test_issue_2886_no_dictionary_label_means_stranger():
    """Робот, акт 2b run 35903232434: Борис получил словарную кличку «Гость».

    Фильтр #2864 закрывал только LLM-слой, а словарный пул сам выдавал
    знакомому человеку метку «незнакомца». Ни одна словарная кличка не
    должна проходить через ``is_stranger_epithet``.
    """
    bad = [e for e in _all_dictionary_labels() if ep.is_stranger_epithet(e)]
    assert not bad, f"словарные клички-«незнакомцы»: {bad}"


@pytest.mark.parametrize("label", ["Гость", "Прохожий", "Визитёр", "Пришелец"])
def test_issue_2886_visitor_words_are_stranger_labels(label):
    """Прохожий, визитёр, пришелец — тот же смысл «чужой человек», что и гость."""
    assert ep.is_stranger_epithet(label) is True
    assert label not in _all_dictionary_labels()


def test_issue_2887_dictionary_layer_stays_single_word():
    """Словарный фолбек — одно слово (обоснование в docstring модуля).

    Заодно он не может совпасть с LLM-кличкой: у той 2–4 слова.
    """
    pools = list(ep.EPITHET_LEXICON.values())
    pools += [ep.DEFAULT_POOL_NEUTRAL, ep.DEFAULT_POOL_RESTLESS]
    for pool in pools:
        for label in pool:
            assert " " not in label, label
            assert ep.check_llm_epithet(label)[0] is None, label


# ── Issue #2926: отказ модели не должен проходить как кличка ────────────────


@pytest.mark.parametrize("bad", [
    "Пока нечего показать",         # живой прогон, акт 2 run 35933157262
    "Ничего не могу сказать",
    "Недостаточно данных пока",
    "Затрудняюсь дать кличку",
    "Невозможно точно сказать",
    "Неясно что сказать пока",
    "Непонятно кто это пока",
    # "Неизвестно" сюда не идёт: это уже «незнакомец» (#2864,
    # STRANGER_EPITHET_STEMS), стоит первым в порядке проверки — см.
    # test_issue_2926_unknown_word_is_stranger_not_refusal ниже.
])
def test_issue_2926_rejects_refusal_phrases(bad):
    """«Пока нечего показать» и похожие отказы — не кличка, а служебный ответ.

    По форме (2–4 слова, первое с большой буквы) такая фраза неотличима от
    кличек #2887, поэтому нужна отдельная сверка по смыслу, как у
    «незнакомца» (#2864).
    """
    assert ep.check_llm_epithet(bad) == (None, "refusal")
    assert ep.sanitize_llm_epithet(bad) is None, bad
    assert ep.is_refusal_epithet(bad) is True, bad


@pytest.mark.parametrize("good", [
    "Ничейный мастер дебюта",        # «ничья» — шахматный термин, не «ничего»
    "Показательный мастер спорта",   # «показательный» ≠ «пока»/«показать»
    "Покорительный дух странника",   # «покори…» — другой корень, не «пока»
    "Мудрый собеседник-спортсмен",
    "Ночной паяльщик моторов",
])
def test_issue_2926_refusal_check_has_no_false_positives(good):
    """Проверка по целому слову — префиксы «пока»/«ничего» не должны ловить

    похожие, но смысловые не связанные с отказом слова: «показательный»,
    «покорительный», «ничейный» (шахматная ничья).
    """
    assert ep.is_refusal_epithet(good) is False, good
    assert ep.check_llm_epithet(good)[0] == good, good


def test_issue_2926_unknown_word_is_stranger_not_refusal():
    """«Неизвестно» отклоняется раньше — по смыслу «незнакомец» (#2864).

    ``is_stranger_epithet`` проверяется в ``check_llm_epithet`` первой, а
    «неизвест…» уже входит в ``STRANGER_EPITHET_STEMS``. «неизвестно» в
    ``REFUSAL_EPITHET_WORDS`` — намеренный запасной путь на случай, если
    список кличек-незнакомцев когда-то сузят.
    """
    assert ep.check_llm_epithet("Неизвестно пока что") == (None, "stranger")


def test_issue_2926_no_dictionary_label_means_refusal():
    """Ни одна словарная кличка не должна читаться как отказ модели."""
    bad = [e for e in _all_dictionary_labels() if ep.is_refusal_epithet(e)]
    assert not bad, f"словарные клички-«отказы»: {bad}"


def test_issue_2926_prompt_no_longer_echoes_refusal_placeholder():
    """Плейсхолдер пустой истории раньше буквально был «пока нечего показать»

    — модель дословно повторяла его как кличку (issue #2926). Новый
    плейсхолдер не похож на ответ, а промпт разрешает вернуть «-», когда
    фактов о человеке нет.
    """
    prompt = ep.build_llm_prompt([], fallback="Странник")
    assert "пока нечего показать" not in prompt.lower()
    assert "-" in prompt
    assert "фактов" in prompt.lower() or "не хватает" in prompt.lower()


def test_issue_2926_check_llm_epithet_names_refusal_reason():
    label, why = ep.check_llm_epithet("Пока нечего показать")
    assert label is None
    assert why == ep.REJECT_REFUSAL == "refusal"


# ── Issue #2934: переспрос LLM-клички, когда появились факты ────────────────


def test_issue_2934_no_reask_before_enough_messages():
    """Меньше EPITHET_REASK_MIN_MESSAGES реплик — фактов ещё мало, не спрашиваем."""
    assert ep.should_reask_llm_epithet([], 0) is False
    assert (
        ep.should_reask_llm_epithet([], ep.EPITHET_REASK_MIN_MESSAGES - 1) is False
    )


def test_issue_2934_reask_once_facts_accumulated():
    """Реплик накопилось достаточно, кличка всё ещё словарная — переспрашиваем."""
    history = [{"ts": 1.0, "old": None, "new": "Странник", "reason": ep.REASON_FIRST_SEEN}]
    assert (
        ep.should_reask_llm_epithet(history, ep.EPITHET_REASK_MIN_MESSAGES) is True
    )


def test_issue_2934_empty_history_still_dictionary():
    """Профиль без единой записи в истории тоже 'ещё словарный' (#2934)."""
    assert ep.should_reask_llm_epithet([], ep.EPITHET_REASK_MIN_MESSAGES) is True


def test_issue_2934_llm_epithet_is_not_overwritten():
    """Если последний пересмотр — REASON_LLM, кличку не трогаем (#2887)."""
    history = [
        {"ts": 1.0, "old": None, "new": "Странник", "reason": ep.REASON_FIRST_SEEN},
        {"ts": 2.0, "old": "Странник", "new": "Ночной паяльщик моторов", "reason": ep.REASON_LLM},
    ]
    assert (
        ep.should_reask_llm_epithet(history, ep.EPITHET_REASK_MIN_MESSAGES * 10)
        is False
    )


def test_issue_2934_new_topic_reason_still_dictionary():
    """REASON_NEW_TOPIC — тоже словарный пересмотр (#1787), можно переспрашивать."""
    history = [
        {"ts": 1.0, "old": None, "new": "Странник", "reason": ep.REASON_FIRST_SEEN},
        {
            "ts": 2.0,
            "old": "Странник",
            "new": "Наблюдатель",
            "reason": f"{ep.REASON_NEW_TOPIC}:техно",
        },
    ]
    assert (
        ep.should_reask_llm_epithet(history, ep.EPITHET_REASK_MIN_MESSAGES) is True
    )
