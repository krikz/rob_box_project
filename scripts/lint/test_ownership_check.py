"""Self-tests для scripts/lint/ownership_check.py (ADR-0150 PR-2 / #3687).

Запуск: ``python -m unittest scripts.lint.test_ownership_check -v``
(или ``pytest scripts/lint/test_ownership_check.py``).
"""

from __future__ import annotations

import copy
import unittest

from scripts.lint import ownership_check as oc
from scripts.lint import seam_without_consumer as seam


def _hit(topic: str, file: str, line: int = 10) -> seam.TopicHit:
    return seam.TopicHit(topic=topic, file=file, line=line, kind="pub")


def _ownership() -> dict:
    return {
        "nodes": {
            "stt_node": {"owner": "voice", "source": ["src/pkg/stt_node.py"]},
            "telegram_node": {"owner": "telegram", "source": ["src/tg/*.py"]},
            "tts_node": {"owner": "voice", "source": ["src/pkg/tts_node.py"]},
        },
        "topics": {
            "/voice/stt/utterance": {
                "owner": "stt_node",
                "status": "live",
                "writers_allowed": ["stt_node", "telegram_node"],
                "readers": ["tts_node"],
            },
            "/voice/speech/event": {
                "owner": "tts_node",
                "status": "planned",
                "writers_allowed": ["tts_node"],
                "readers": [],
            },
        },
    }


def _pubs() -> dict:
    return {
        "/voice/stt/utterance": [
            _hit("/voice/stt/utterance", "src/pkg/stt_node.py"),
            _hit("/voice/stt/utterance", "src/tg/telegram_node.py"),
        ]
    }


class TopicScopeTests(unittest.TestCase):
    def test_prefixes(self) -> None:
        for topic in ("/dialog/session", "/voice/speech/event", "/voice/expression/request",
                      "/voice/providers/state", "/voice/stt/utterance", "/voice/speaker/result",
                      "/voice/music/state", "/telegram/session", "/avatar/session"):
            self.assertTrue(oc.is_owned_topic(topic), topic)
        for topic in ("/voice/tts/request", "/avatar/state", "/dialogue/control", "/cmd_vel",
                      "/avatar/sessions_extra"):
            self.assertFalse(oc.is_owned_topic(topic), topic)


class StaticCheckTests(unittest.TestCase):
    def test_clean_map_passes(self) -> None:
        own = _ownership()
        self.assertEqual(oc.check_declarations(own), [])
        self.assertEqual(oc.check_static(own, _pubs()), [])

    def test_topic_without_owner_fails(self) -> None:
        pubs = _pubs()
        pubs["/voice/speaker/new_topic"] = [_hit("/voice/speaker/new_topic", "src/pkg/stt_node.py")]
        errors = oc.check_static(_ownership(), pubs)
        self.assertEqual(len(errors), 1)
        self.assertIn("нет владельца", errors[0])

    def test_extra_writer_fails(self) -> None:
        pubs = _pubs()
        pubs["/voice/stt/utterance"].append(_hit("/voice/stt/utterance", "src/pkg/tts_node.py", 7))
        errors = oc.check_static(_ownership(), pubs)
        self.assertEqual(len(errors), 1)
        self.assertIn("tts_node", errors[0])
        self.assertIn("вне writers_allowed", errors[0])

    def test_publisher_outside_any_node_fails(self) -> None:
        pubs = _pubs()
        pubs["/voice/stt/utterance"].append(_hit("/voice/stt/utterance", "src/pkg/stranger.py"))
        errors = oc.check_static(_ownership(), pubs)
        self.assertEqual(len(errors), 1)
        self.assertIn("не входит ни в один узел", errors[0])

    def test_dev_script_is_not_a_writer(self) -> None:
        pubs = _pubs()
        pubs["/voice/stt/utterance"].append(
            _hit("/voice/stt/utterance", "src/rob_box_voice/scripts/example_voice_usage.py")
        )
        self.assertEqual(oc.check_static(_ownership(), pubs), [])

    def test_live_topic_without_publisher_fails(self) -> None:
        errors = oc.check_static(_ownership(), {})
        self.assertEqual(len(errors), 1)
        self.assertIn("status live", errors[0])

    def test_planned_topic_published_by_code_fails(self) -> None:
        pubs = _pubs()
        pubs["/voice/speech/event"] = [_hit("/voice/speech/event", "src/pkg/tts_node.py")]
        errors = oc.check_static(_ownership(), pubs)
        self.assertEqual(len(errors), 1)
        self.assertIn("planned", errors[0])

    def test_topics_outside_scope_are_ignored(self) -> None:
        pubs = _pubs()
        pubs["/voice/tts/request"] = [_hit("/voice/tts/request", "src/pkg/anything.py")]
        self.assertEqual(oc.check_static(_ownership(), pubs), [])


class DeclarationTests(unittest.TestCase):
    def test_missing_fields_and_unknown_nodes_fail(self) -> None:
        own = _ownership()
        own["topics"]["/voice/stt/utterance"]["writers_allowed"] = ["stt_node", "ghost_node"]
        del own["topics"]["/voice/speech/event"]["readers"]
        errors = oc.check_declarations(own)
        self.assertTrue(any("ghost_node" in e for e in errors), errors)
        self.assertTrue(any("нет поля 'readers'" in e for e in errors), errors)

    def test_owner_must_be_a_declared_node(self) -> None:
        own = copy.deepcopy(_ownership())
        own["topics"]["/voice/stt/utterance"]["owner"] = "nobody"
        self.assertTrue(any("owner" in e for e in oc.check_declarations(own)))

    def test_entry_outside_guard_scope_is_flagged(self) -> None:
        own = _ownership()
        own["topics"]["/cmd_vel"] = copy.deepcopy(own["topics"]["/voice/speech/event"])
        self.assertTrue(any("вне зоны гарда" in e for e in oc.check_declarations(own)))


class RuntimeDebtTests(unittest.TestCase):
    def test_multiple_writers_need_debt(self) -> None:
        baseline = {"known": [{"key": "multiple_writers|/voice/tts/request|", "scope": "repo"}]}
        errors = oc.check_runtime_debts(_ownership(), baseline)
        self.assertEqual(len(errors), 1)
        self.assertIn("debt", errors[0])

    def test_debt_with_pr_passes(self) -> None:
        baseline = {"known": [{"key": "multiple_writers|/voice/tts/request|", "scope": "repo",
                               "debt": "PR-3 (#3688): один клиент речи"}]}
        self.assertEqual(oc.check_runtime_debts(_ownership(), baseline), [])

    def test_runtime_multi_writer_on_single_writer_topic_fails(self) -> None:
        baseline = {"known": [{"key": "multiple_writers|/voice/speech/event|", "scope": "repo",
                               "debt": "PR-3 (#3688): один писатель"}]}
        errors = oc.check_runtime_debts(_ownership(), baseline)
        self.assertEqual(len(errors), 1)
        self.assertIn("допускает одного", errors[0])

    def test_other_finding_kinds_are_not_checked(self) -> None:
        baseline = {"known": [{"key": "dead_input|/voice/state|", "scope": "repo"}]}
        self.assertEqual(oc.check_runtime_debts(_ownership(), baseline), [])


class RealRepositoryTests(unittest.TestCase):
    """Карта и код репозитория согласованы (то, что гоняет CI)."""

    def test_repository_ownership_is_consistent(self) -> None:
        ownership = oc.load_ownership()
        pubs = oc.scan_publishers(oc.REPO_ROOT / "src")
        import json

        baseline = json.loads(oc.RUNTIME_BASELINE_FILE.read_text(encoding="utf-8"))
        self.assertEqual(oc.run_checks(ownership, pubs, baseline), [])

    def test_typed_stt_and_speaker_topics_have_expected_writers(self) -> None:
        topics = oc.load_ownership()["topics"]
        self.assertEqual(sorted(topics["/voice/stt/utterance"]["writers_allowed"]),
                         ["stt_node", "telegram_node"])
        self.assertEqual(topics["/voice/speaker/result"]["writers_allowed"], ["speaker_id_node"])
        self.assertEqual(topics["/voice/speaker/result"]["type"], "rob_box_dialog_msgs/SpeakerResult")
        self.assertEqual(topics["/voice/stt/utterance"]["type"], "rob_box_dialog_msgs/Utterance")

    def test_removed_string_topic_is_gone_from_code(self) -> None:
        pubs = oc.scan_publishers(oc.REPO_ROOT / "src")
        self.assertNotIn("/voice/stt/result", pubs)


if __name__ == "__main__":
    unittest.main()
