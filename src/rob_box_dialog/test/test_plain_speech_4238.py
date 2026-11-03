"""Голый текст речи — правило из таблицы знания (ADR-0160 §2.1, PR-1, #4238; ADR-0148).

Модель пишет в ``speech`` и ``perform.segments`` только слова и знаки :data:`knowledge.PLAIN_SPEECH_PUNCTUATION`;
паузы, ударения, кавычки и тире под голос или экран делает код. Проверяется поведение модуля: правило приходит
агенту робота в блоке правил, называет те же знаки, что константа, пауз-тегов в правилах и во фрагменте команды
нет, а пример команды во фрагменте сам пишет голым текстом (пример с тире учил бы обратному).
"""

import json
import re

from rob_box_dialog import knowledge as kn

ROBOT = kn.AGENTS[0]
#: Буквы, цифры, пробел, перевод строки и дефис внутри слова — плюс знаки из таблицы знания.
_ALLOWED = re.compile(r"[\w \n-]")


def _plain(text: str) -> bool:
    return all(_ALLOWED.fullmatch(ch) or ch in kn.PLAIN_SPEECH_PUNCTUATION for ch in text)


def _rule(key: str) -> kn.PromptRule:
    return next(rule for rule in kn.PROMPT_RULES if rule.key == key)


def test_robot_agent_gets_the_plain_speech_rule():
    rule = _rule("plain_speech")
    assert rule.agents == (ROBOT,)
    assert f"- {rule.text}" in kn.prompt_rules(ROBOT).splitlines()


def test_rule_names_exactly_the_punctuation_of_the_table():
    """Знаки правила — из константы: поменяли таблицу — поменялось правило, второго списка нет."""
    assert " ".join(kn.PLAIN_SPEECH_PUNCTUATION) in _rule("plain_speech").text
    assert set(kn.PLAIN_SPEECH_PUNCTUATION) == set(". , ! ? : ;".split())


def test_rule_speaks_of_the_fields_and_role_of_the_command_schema():
    """Правило называет поля команды, которые есть в схеме: речь, сегменты выступления и роль сегмента."""
    props = kn.command_schema("json")["properties"]
    perform = props["perform"]["properties"]
    assert "speech" in props and "segments" in perform and "role_per_segment" in perform
    text = _rule("plain_speech").text
    assert all(f"`{name}`" in text for name in ("speech", "perform.segments", "role_per_segment"))


def test_no_pause_tags_in_rules_or_command_fragment():
    blocks = [kn.prompt_rules(agent) for agent in kn.AGENTS] + [kn.prompt_fragment()]
    assert not [b for b in blocks if "<break" in b or "<speak" in b]


def test_command_fragment_example_is_plain_text():
    """Пример правки выступления во фрагменте — та же речь, что модель пишет: без тире и многоточия."""
    line = next(x for x in kn.prompt_fragment().splitlines() if x.startswith("- edit_pending"))
    segments = json.loads(re.search(r'"segments": (\[.*?\])', line).group(1))
    assert segments and all(_plain(s) for s in segments), segments


def test_plain_check_rejects_what_the_rule_forbids():
    """Самопроверка помощника теста: запретное правилом он не пропускает."""
    assert _plain("Привет, мир! Как дела?\nХорошо: всё по-старому; да.")
    for bad in ("«цитата»", '"цитата"', "— реплика", "(скобка)", "<break time=\"300ms\"/>", "**жирно**", "🤖", "…"):
        assert not _plain(bad), bad
