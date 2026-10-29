"""#3975 — слова темпа в воротах искажённого вейка (``rob_box_dialog``) не расходятся с таблицей правок играющего
(``rob_box_music.knowledge.TRACK_EDITS``, #4029).

Пакет диалога от ``rob_box_music`` не зависит (образы бота и матрицы собирают его без музыки), поэтому ворота правки
музыки (``WAKE_SLIP_GATES``, ``under_music``) строятся из таблиц диалога, а эта сверка лежит здесь, где видны оба
пакета: добавили слово темпа в ``TRACK_EDITS`` — «роботы <слово>» под музыкой обязано будить, иначе тест красный.
Только вид ``tempo``: тембр и ``develop`` без названной музыки бывают про лицо, свет и рассказ (``TrackEdit.bare``), их
ворота не открывают — это отдельное решение."""

import pytest

from rob_box_dialog.knowledge import WAKE_SLIP_TWEAK_ENDINGS
from rob_box_dialog.words import slipped_wake_rest
from rob_box_music import knowledge as kn

TEMPO_WORDS = tuple(dict.fromkeys(
    word for edit in kn.TRACK_EDITS.values() if edit.group == "tempo" for word in edit.words))


def test_the_edit_table_has_tempo_words():
    assert len(TEMPO_WORDS) >= 8, "в таблице правок нет слов темпа - сверка ничего не проверяет"


@pytest.mark.parametrize("entry", TEMPO_WORDS)
def test_every_tempo_edit_word_wakes_after_a_slipped_wake_under_music_only(entry):
    """Запись таблицы - основа («ускор») или слово («быстрее»): хоть одна словоформа «запись + окончание» будит после
    «роботы» под музыкой, и ни одна не будит без музыки."""
    forms = [entry + ending for ending in WAKE_SLIP_TWEAK_ENDINGS]
    assert any(slipped_wake_rest(f"роботы {form}", music_playing=True) for form in forms), entry
    assert not any(slipped_wake_rest(f"роботы {form}") for form in forms), entry
