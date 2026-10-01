"""Issue #3271 — ml06: LLM пересказывает трек из истории, не зовёт gen_get_track_info.

Живой лог прогона 36777967495 (шаг ml06, «Робот, расскажи, что это за
трек»): tools=[], ответ «баллада про дождь за окном, который … подыгрывает
мыслям» — детали выдуманы (в истории была только «короткая грустная
баллада»). Правило: вопрос про трек → ``gen_get_track_info`` ДО ответа, говорить
по полям результата. Правило общее (не под фразу шага) и лежит и в
RULE #DISCOVERY-TOOLS, и в блоке скилла player.
"""

from __future__ import annotations

from pathlib import Path
import re

PROMPTS = Path(__file__).resolve().parents[2] / "prompts"
MASTER = (PROMPTS / "master_prompt_compact.txt").read_text(encoding="utf-8")
PLAYER = (PROMPTS / "skills" / "player.txt").read_text(encoding="utf-8")


def _discovery_block() -> str:
    m = re.search(r"🚨 \*\*RULE #DISCOVERY-TOOLS — .*?(?=🚨 \*\*RULE #)", MASTER, re.DOTALL)
    assert m, "RULE #DISCOVERY-TOOLS не найден"
    return m.group(0)


def _player_block() -> str:
    m = re.search(r"<<<SKILL-MOVE player>>>(.*?)<<<SKILL-MOVE-END>>>", MASTER, re.DOTALL)
    assert m, "блок SKILL-MOVE player не найден"
    return m.group(1)


def test_discovery_rule_requires_track_info_before_speak() -> None:
    block = _discovery_block()
    assert "gen_get_track_info" in block
    assert "что это за трек" in block
    assert "BEFORE" in block.split("gen_get_track_info", 1)[1][:300]


def test_player_block_forbids_retelling_from_history() -> None:
    block = _player_block()
    assert "gen_get_track_info" in block
    assert "по памяти" in block


def test_player_skill_file_mentions_track_info_rule() -> None:
    assert "gen_get_track_info" in PLAYER
    assert "по памяти" in " ".join(PLAYER.split())
