"""#3801 (решение Шифу 21.10): имя меняется фразой, один голос — одна личность, подтверждение шаблоном нормальной длины.

Поведение чистых решений: грамматика смены имени (``reply_grammar.classify_rename``) и шаги личности
(``speaker_adapter``): что сделать (переименовать / склеить / ничего) и что сказать — кодом.
"""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.reply_grammar import NAME, NO, YES, Rename, classify_rename
from rob_box_dialog.speaker_adapter import (
    VOICE_STRANGER,
    VOICE_SURE,
    VOICE_UNSURE,
    merge_step,
    question_reply,
    rename_step,
    same_voice_plan,
    same_voice_step,
    voice_certainty,
    voice_check_step,
)

EIDZHIK = {"is_known": True, "speaker_id": "e1d2", "name": "Эйджик", "confidence": 0.93}
SHORT_YES = {"is_known": False, "candidate_name": "Степан", "candidate_score": 0.673}  # живой прогон 2 #3765
STRANGER_VOICE = {"is_known": False, "candidate_name": "Саша", "candidate_score": 0.41}
PLAN = {"kind": "conflict", "new_id": "new1", "new_name": "Степан", "known_id": "old1", "known_name": "Тимофей"}


class TestRenameGrammar:
    @pytest.mark.parametrize("text, expected", [
        ("меня зовут не Эйджик, а Денчик", Rename("Денчик", "Эйджик")),
        ("Робот, меня зовут не Эйджик а Денчик", Rename("Денчик", "Эйджик")),
        ("я не Саша, а Боря", Rename("Боря", "Саша")),
        ("зови меня Боря", Rename("Боря")),
        ("Робот, называй меня Денчик", Rename("Денчик")),
        ("переименуй меня в Денчик", Rename("Денчик")),
    ])
    def test_rename_phrases(self, text, expected):
        assert classify_rename(text) == expected

    @pytest.mark.parametrize("text", [
        "меня зовут Денчик",  # представление, не смена имени
        "мы разные люди",
        "называй меня просто",  # не имя
        "я не знаю, а ты",
        "",
    ])
    def test_not_rename(self, text):
        assert classify_rename(text) is None

    def test_correction_is_an_answer_with_new_name(self):
        """«меня зовут не X, а Y» в окне ответа — имя Y, а не «Не»."""
        assert question_reply("same_voice", "меня зовут не Эйджик, а Денчик").name == "Денчик"
        assert question_reply("same_voice", "мы разные люди").kind == NO
        assert question_reply("same_voice", "да, это я").kind == YES
        assert question_reply("voice_check", "Меня зовут Денчик, и это мой голос, запомни его").kind == NAME


class TestVoiceCertainty:
    def test_name_from_biometry_is_sure(self):
        assert voice_certainty(EIDZHIK) == VOICE_SURE

    @pytest.mark.parametrize("speaker", [
        SHORT_YES,
        {"is_known": True, "speaker_id": "e1d2", "name": None, "tentative_name": "Эйджик", "confidence": 0.74},
        {"is_known": False, "inconclusive": True, "reason": "too_short"},
    ])
    def test_short_or_tentative_voice_is_unsure(self, speaker):
        assert voice_certainty(speaker) == VOICE_UNSURE

    @pytest.mark.parametrize("speaker", [STRANGER_VOICE, {"is_known": False}, {}])
    def test_other_voice_is_stranger(self, speaker):
        assert voice_certainty(speaker) == VOICE_STRANGER

    def test_threshold_lives_in_knowledge(self):
        assert kn.IDENTITY_TEMPLATE_MIN_SCORE < 0.72, "биометрический порог 0.72 не меняется — это нижняя граница «близко»"


class TestRenameStep:
    def test_sure_voice_renames_own_profile(self):
        step = rename_step(Rename("Денчик", "Эйджик"), EIDZHIK)
        assert step.rename == ("e1d2", "Денчик")
        assert step.phrase == phrases.text("identity.renamed", name="Денчик")
        assert step.frame is None

    def test_same_name_changes_nothing(self):
        step = rename_step(Rename("Эйджик"), EIDZHIK)
        assert step.rename == ()
        assert step.phrase == phrases.text("identity.name_same", name="Эйджик")

    def test_unsure_voice_asks_template(self):
        step = rename_step(Rename("Денчик"), SHORT_YES)
        assert step.rename == ()
        assert step.frame["kind"] == kn.IDENTITY_VOICE_CHECK_KIND
        assert step.frame["action"] == "rename" and step.frame["name"] == "Денчик"
        assert phrases.text("identity.template", name="Денчик") in step.phrase

    def test_stranger_or_no_request_is_ordinary_turn(self):
        assert rename_step(Rename("Денчик"), STRANGER_VOICE) is None
        assert rename_step(None, EIDZHIK) is None


class TestSameVoice:
    def test_plan_says_phrase_and_keeps_known(self):
        plan = same_voice_plan("sasha", "Саша", "Боря")
        assert plan["kind"] == kn.IDENTITY_SAME_VOICE_KIND
        assert plan["new_id"] == "" and plan["known_id"] == "sasha"
        assert plan["question"] == phrases.text("identity.same_voice", known="Саша", name="Боря")

    def test_we_are_different_gets_one_voice_phrase(self):
        plan = same_voice_plan("sasha", "Саша", "Боря")
        step = same_voice_step(plan, "мы разные люди")
        assert step.merge == () and step.rename == ()
        assert step.phrase == phrases.text("identity.one_voice", name="Боря")

    def test_its_me(self):
        step = same_voice_step(same_voice_plan("sasha", "Саша", "Боря"), "да, это я")
        assert step.phrase == phrases.text("identity.same_voice_ok", name="Саша")


class TestTemplate:
    def test_template_is_long_enough_for_biometry(self):
        words = phrases.text("identity.template", name="Денчик").replace(",", " ").split()
        assert len(words) >= kn.IDENTITY_TEMPLATE_MIN_WORDS
        assert kn.IDENTITY_TEMPLATE_MIN_WORDS / 2.5 >= kn.IDENTITY_TEMPLATE_MIN_VOICE_S

    def test_short_its_me_with_unsure_voice_asks_template(self):
        """Живой прогон 2 #3765: «да, это я» 1.17 с → кандидат 0.673 < 0.72 — вместо «Не понял ответ» просьба шаблона."""
        step = merge_step(PLAN, "да, это я", SHORT_YES)
        assert step.merge == ()
        assert step.frame["action"] == "merge"
        assert (step.frame["new_id"], step.frame["known_id"]) == ("new1", "old1")
        assert phrases.text("identity.template", name="Тимофей") in step.phrase

    def test_its_me_from_stranger_still_merges_nothing(self):
        """#2829: чужой голос чужую склейку не подтверждает и шаблон не получает."""
        step = merge_step(PLAN, "да, это я", STRANGER_VOICE)
        assert step.merge == () and step.frame is None
        assert step.phrase == phrases.text("identity.merge_unclear")

    def test_template_with_sure_voice_of_pair_merges(self):
        frame = merge_step(PLAN, "да, это я", SHORT_YES).frame
        step = voice_check_step(frame, {"is_known": True, "speaker_id": "old1", "name": "Тимофей"})
        assert step.merge == ("new1", "old1", "Тимофей")
        assert step.phrase == phrases.text("identity.merged", name="Тимофей")

    def test_template_with_voice_outside_pair_changes_nothing(self):
        frame = merge_step(PLAN, "да, это я", SHORT_YES).frame
        step = voice_check_step(frame, {"is_known": True, "speaker_id": "other", "name": "Олег"})
        assert step.merge == () and step.rename == ()
        assert step.phrase == phrases.text("identity.voice_check_failed")

    def test_template_unsure_again_changes_nothing(self):
        frame = rename_step(Rename("Денчик"), SHORT_YES).frame
        step = voice_check_step(frame, SHORT_YES)
        assert step.rename == ()
        assert step.phrase == phrases.text("identity.voice_check_failed")

    def test_template_for_rename_renames_sure_voice(self):
        frame = rename_step(Rename("Денчик"), SHORT_YES).frame
        step = voice_check_step(frame, EIDZHIK)
        assert step.rename == ("e1d2", "Денчик")
        assert step.phrase == phrases.text("identity.renamed", name="Денчик")

    def test_sure_its_me_merges_without_template(self):
        step = merge_step(PLAN, "да, это я", {"is_known": True, "speaker_id": "old1", "name": "Тимофей"})
        assert step.merge == ("new1", "old1", "Тимофей") and step.frame is None
