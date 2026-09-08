#!/usr/bin/env python3
"""Self-tests for scripts/lint/seam_without_consumer.py (issue #2118).

``cc_budget.py`` (the sibling guard this script's conventions are copied
from) has no dedicated test suite — there's nothing to match here. This
guard gets one anyway because its resolution logic (module/class/parameter
constant chasing, the declare_parameter/get_parameter idiom, cross-module
imports) is nontrivial enough that regressions would be easy to introduce
silently, and because the whole point of the tool (issue #2118) is to never
again let a real regression hide behind "the other half is covered
elsewhere" — that standard should apply to the guard's own code too.

Two kinds of coverage:

* Unit tests for the resolution helpers in isolation (module/class/local
  constants, declare_parameter/get_parameter, cross-module imports).
* Regression tests that reconstruct the minimal shape of the three
  historical incidents (#1992, #2113, #2116) as synthetic fixtures and
  assert ``scan_files`` still flags them — so a future refactor of this
  script that accidentally stops catching one of them fails loudly here,
  not three days after the next real regression.

Run: ``python -m unittest scripts/lint/test_seam_without_consumer.py -v``
(or ``python scripts/lint/test_seam_without_consumer.py``).
"""

from __future__ import annotations

import ast
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import seam_without_consumer as swc  # noqa: E402


def _write(dir_: Path, rel: str, content: str) -> Path:
    path = dir_ / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    return path


class LiteralResolutionTests(unittest.TestCase):
    def test_plain_string_literal(self):
        node = ast.parse('"/foo/bar"', mode="eval").body
        self.assertEqual(swc._literal_str(node), "/foo/bar")

    def test_fully_literal_fstring(self):
        node = ast.parse('f"/foo/bar"', mode="eval").body
        self.assertEqual(swc._literal_str(node), "/foo/bar")

    def test_fstring_with_interpolation_is_not_a_plain_literal(self):
        node = ast.parse('f"/foo/{x}"', mode="eval").body
        self.assertIsNone(swc._literal_str(node))

    def test_non_string_constant_is_not_literal(self):
        node = ast.parse("10", mode="eval").body
        self.assertIsNone(swc._literal_str(node))


class ModuleConstsTests(unittest.TestCase):
    def test_simple_assign(self):
        tree = ast.parse('TOPIC = "/foo"\n')
        self.assertEqual(swc._module_consts(tree), {"TOPIC": "/foo"})

    def test_ann_assign(self):
        tree = ast.parse('TOPIC: str = "/foo"\n')
        self.assertEqual(swc._module_consts(tree), {"TOPIC": "/foo"})

    def test_reassignment_to_different_literal_is_ambiguous(self):
        tree = ast.parse('TOPIC = "/foo"\nTOPIC = "/bar"\n')
        self.assertIsNone(swc._module_consts(tree)["TOPIC"])

    def test_non_literal_assign_is_unresolved(self):
        tree = ast.parse("TOPIC = some_call()\n")
        self.assertIsNone(swc._module_consts(tree)["TOPIC"])


class ClassConstsTests(unittest.TestCase):
    def _class_node(self, src: str) -> ast.ClassDef:
        tree = ast.parse(src)
        (cls,) = [n for n in tree.body if isinstance(n, ast.ClassDef)]
        return cls

    def test_class_body_literal(self):
        cls = self._class_node(
            "class Foo:\n"
            "    TOPIC: str = '/foo'\n"
        )
        self.assertEqual(swc._class_consts(cls)["TOPIC"], "/foo")

    def test_self_attr_assign_in_init(self):
        cls = self._class_node(
            "class Foo:\n"
            "    def __init__(self):\n"
            "        self._topic = '/foo'\n"
        )
        self.assertEqual(swc._class_consts(cls)["_topic"], "/foo")

    def test_self_attr_ann_assign_in_init(self):
        # Regression: the actual tts_node.py shape for /tars1/text (issue
        # #2113) — an annotated self-attribute assign was silently invisible
        # to the first version of this resolver (Assign-only), which made
        # /tars1/text look one-sided (sub-only) even after it had a
        # matching publisher.
        cls = self._class_node(
            "class Foo:\n"
            "    def __init__(self):\n"
            "        self._topic: str = '/tars1/text'\n"
        )
        self.assertEqual(swc._class_consts(cls)["_topic"], "/tars1/text")

    def test_declare_then_get_parameter_idiom(self):
        # The quest_node.py / tts_node.py idiom: declare a parameter with a
        # literal default, later assign self.attr from get_parameter(...).value.
        cls = self._class_node(
            "class Foo:\n"
            "    def __init__(self):\n"
            "        self.declare_parameter('topic_param', '/avatar/tts/request')\n"
            "        self.avatar_request_topic = str(\n"
            "            self.get_parameter('topic_param').value\n"
            "        )\n"
        )
        self.assertEqual(swc._class_consts(cls)["avatar_request_topic"], "/avatar/tts/request")

    def test_declared_param_without_default_is_unresolved(self):
        cls = self._class_node(
            "class Foo:\n"
            "    def __init__(self):\n"
            "        self.declare_parameter('topic_param')\n"
            "        self.t = str(self.get_parameter('topic_param').value)\n"
        )
        self.assertIsNone(swc._class_consts(cls)["t"])

    def test_module_const_reexported_as_class_attr(self):
        # rob_box_telegram/supervisor_client.py shape: a module-level
        # constant re-exposed as a same-named class attribute so it's
        # reachable as ``self.TOPIC_STATE``.
        tree = ast.parse(
            "TOPIC_STATE = '/avatar/state'\n"
            "class Foo:\n"
            "    TOPIC_STATE = TOPIC_STATE\n"
        )
        module_consts = swc._module_consts(tree)
        (cls,) = [n for n in tree.body if isinstance(n, ast.ClassDef)]
        self.assertEqual(swc._class_consts(cls, module_consts)["TOPIC_STATE"], "/avatar/state")

    def test_ambiguous_self_attr_is_unresolved(self):
        cls = self._class_node(
            "class Foo:\n"
            "    def a(self):\n"
            "        self._topic = '/foo'\n"
            "    def b(self):\n"
            "        self._topic = '/bar'\n"
        )
        self.assertIsNone(swc._class_consts(cls)["_topic"])


class ParamDefaultTests(unittest.TestCase):
    def test_keyword_only_default(self):
        tree = ast.parse(
            "def f(self, *, panel_url_topic: str = '/avatar/tars/panel_url'):\n"
            "    pass\n"
        )
        func = tree.body[0]
        self.assertEqual(
            swc._param_defaults(func)["panel_url_topic"], "/avatar/tars/panel_url"
        )

    def test_positional_default(self):
        tree = ast.parse("def f(self, topic='/x'):\n    pass\n")
        func = tree.body[0]
        self.assertEqual(swc._param_defaults(func)["topic"], "/x")

    def test_local_consts_resolves_get_parameter_local_var(self):
        # quest_node.py shape: a plain local var (not self.attr) built from
        # declare_parameter/get_parameter, then used directly as the topic
        # arg. This is the /device/snapshot resolution path.
        tree = ast.parse(
            "class Foo:\n"
            "    def setup(self):\n"
            "        self.declare_parameter('battery_json_topic', '/device/snapshot')\n"
            "        battery_topic = str(self.get_parameter('battery_json_topic').value)\n"
        )
        (cls,) = [n for n in tree.body if isinstance(n, ast.ClassDef)]
        declared = swc._declared_params(cls)
        setup_func = cls.body[0]
        local = swc._local_consts(setup_func, declared)
        self.assertEqual(local["battery_topic"], "/device/snapshot")


class ModuleDottedNameTests(unittest.TestCase):
    def test_ament_python_layout(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            path = _write(root, "src/rob_box_core/rob_box_core/avatar_command.py", "")
            dotted = self._dotted_relative_to(root, path)
            self.assertEqual(dotted, "rob_box_core.avatar_command")

    @staticmethod
    def _dotted_relative_to(root: Path, path: Path) -> str | None:
        # _module_dotted_name uses REPO_ROOT internally via _rel(); patch it
        # for the duration of this one computation.
        original = swc.REPO_ROOT
        swc.REPO_ROOT = root
        try:
            return swc._module_dotted_name(path)
        finally:
            swc.REPO_ROOT = original

    def test_non_doubled_dir_is_unresolvable(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            path = _write(root, "src/rob_box_animations/scripts/animation_player_node.py", "")
            self.assertIsNone(self._dotted_relative_to(root, path))


class ScanFilesIntegrationTests(unittest.TestCase):
    """End-to-end scan_files() tests, including the three historical shapes."""

    def _scan(self, files: dict[str, str]) -> tuple[swc.SeamScan, Path]:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        paths = [_write(root, rel, content) for rel, content in files.items()]
        # scan_files() computes repo-relative paths (for reporting) and the
        # ament_python dotted-module heuristic (for cross-file import
        # resolution) via the real REPO_ROOT — point it at the fixture root
        # for the duration of the scan so both work against our tmpdir tree.
        original_root = swc.REPO_ROOT
        swc.REPO_ROOT = root
        try:
            return swc.scan_files(paths), root
        finally:
            swc.REPO_ROOT = original_root

    def test_issue_1992_subscriber_without_publisher(self):
        # #1992: stt_node subscribes /audio/quest_wake, nothing publishes it.
        scan, _ = self._scan(
            {
                "src/rob_box_voice/rob_box_voice/stt_node.py": (
                    "class SttNode:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(\n"
                    "            AudioData, '/audio/quest_wake', self.cb, 10\n"
                    "        )\n"
                ),
            }
        )
        self.assertIn("/audio/quest_wake", scan.subs)
        self.assertNotIn("/audio/quest_wake", scan.pubs)

    def test_issue_2113_publishers_without_subscribers(self):
        # #2113: tts_node + tars_panel publish /tars1/text and
        # /avatar/tars/panel_url; nothing in the repo subscribes to either
        # (the real subscriber, quest_node, hadn't been wired yet).
        scan, _ = self._scan(
            {
                "src/rob_box_voice/rob_box_voice/tts_node.py": (
                    "class TtsNode:\n"
                    "    def __init__(self):\n"
                    "        self._tars1_text_topic: str = '/tars1/text'\n"
                    "        self.create_publisher(String, self._tars1_text_topic, 10)\n"
                ),
                "src/rob_box_supervisor/rob_box_supervisor/tars_panel.py": (
                    "class TarsPanelDispatcher:\n"
                    "    def __init__(self, node, *, panel_url_topic='/avatar/tars/panel_url'):\n"
                    "        node.create_publisher(String, panel_url_topic, 10)\n"
                ),
            }
        )
        self.assertIn("/tars1/text", scan.pubs)
        self.assertNotIn("/tars1/text", scan.subs)
        self.assertIn("/avatar/tars/panel_url", scan.pubs)
        self.assertNotIn("/avatar/tars/panel_url", scan.subs)

    def test_issue_2113_resolved_once_subscriber_lands(self):
        # Same as above, plus the quest_node subscription the real fix
        # (832124c4) added — both topics should now balance out.
        scan, _ = self._scan(
            {
                "src/rob_box_voice/rob_box_voice/tts_node.py": (
                    "class TtsNode:\n"
                    "    def __init__(self):\n"
                    "        self._tars1_text_topic: str = '/tars1/text'\n"
                    "        self.create_publisher(String, self._tars1_text_topic, 10)\n"
                ),
                "src/rob_box_quest/rob_box_quest/quest_node.py": (
                    "class QuestNode:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(String, '/tars1/text', self._on_tars1_text, 10)\n"
                ),
            }
        )
        self.assertIn("/tars1/text", scan.pubs)
        self.assertIn("/tars1/text", scan.subs)

    def test_issue_2116_seam_covered_by_tests_only(self):
        # #2116: _publish_avatar_tts defined + called from tests, zero
        # production callers.
        scan, _ = self._scan(
            {
                "src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py": (
                    "class AvatarSupervisor:\n"
                    "    def _publish_avatar_tts(self, text):\n"
                    "        pass\n"
                    "    def _unrelated(self):\n"
                    "        pass\n"
                ),
                "src/rob_box_supervisor/test/unit/test_supervisor_avatar_tts.py": (
                    "def test_it(node):\n"
                    "    node._publish_avatar_tts('hi')\n"
                ),
            }
        )
        usage = scan.seam_usage["_publish_avatar_tts"]
        self.assertEqual(usage["prod"], 0)
        self.assertEqual(usage["test"], 1)
        qualnames = {d.qualname for d in scan.seam_defs}
        self.assertIn("AvatarSupervisor._publish_avatar_tts", qualnames)

    def test_on_callback_registration_counts_as_a_use(self):
        # _on_* passed as a bare callback reference (not called directly)
        # must NOT be flagged — this is the normal create_subscription
        # registration shape, not an orphaned seam.
        scan, _ = self._scan(
            {
                "src/rob_box_voice/rob_box_voice/some_node.py": (
                    "class SomeNode:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(String, '/x', self._on_x, 10)\n"
                    "    def _on_x(self, msg):\n"
                    "        pass\n"
                ),
            }
        )
        usage = scan.seam_usage.get("_on_x", {"prod": 0, "test": 0})
        self.assertGreaterEqual(usage["prod"], 1)

    def test_seam_with_no_usage_anywhere_is_not_flagged(self):
        # Plain dead code (zero references, not even in tests) is out of
        # scope for this guard by design (issue #2118 boundary: not a
        # general dead-code detector) — only "tests cover it, prod doesn't"
        # is the target signature.
        scan, _ = self._scan(
            {
                "src/rob_box_voice/rob_box_voice/some_node.py": (
                    "class SomeNode:\n"
                    "    def _on_never_used(self, msg):\n"
                    "        pass\n"
                ),
            }
        )
        usage = scan.seam_usage.get("_on_never_used", {"prod": 0, "test": 0})
        self.assertEqual(usage["test"], 0)

    def test_cross_module_constant_import_resolves(self):
        scan, _ = self._scan(
            {
                "src/rob_box_core/rob_box_core/avatar_command.py": (
                    "AVATAR_COMMAND_TOPIC: str = '/avatar/command'\n"
                ),
                "src/rob_box_quest/rob_box_quest/quest_node.py": (
                    "from rob_box_core.avatar_command import AVATAR_COMMAND_TOPIC\n"
                    "class QuestNode:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(String, AVATAR_COMMAND_TOPIC, self.cb, 10)\n"
                ),
                "src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py": (
                    "from rob_box_core.avatar_command import AVATAR_COMMAND_TOPIC\n"
                    "class Supervisor:\n"
                    "    def __init__(self):\n"
                    "        self.create_publisher(String, AVATAR_COMMAND_TOPIC, 10)\n"
                ),
            }
        )
        self.assertIn("/avatar/command", scan.pubs)
        self.assertIn("/avatar/command", scan.subs)

    def test_test_files_do_not_feed_the_topic_graph(self):
        # A fake node in a test file publishing/subscribing must not count
        # as a real ROS wiring — only the production graph matters.
        scan, _ = self._scan(
            {
                "src/rob_box_voice/test/unit/fake_node.py": (
                    "class FakeNode:\n"
                    "    def __init__(self):\n"
                    "        self.create_publisher(String, '/only/in/tests', 10)\n"
                ),
            }
        )
        self.assertNotIn("/only/in/tests", scan.pubs)
        self.assertNotIn("/only/in/tests", scan.subs)

    def test_unresolvable_topic_is_reported_not_silently_dropped(self):
        scan, _ = self._scan(
            {
                "src/rob_box_voice/rob_box_voice/some_node.py": (
                    "class SomeNode:\n"
                    "    def __init__(self, dynamic_topic):\n"
                    "        self.create_publisher(String, dynamic_topic, 10)\n"
                ),
            }
        )
        self.assertTrue(scan.unresolved)
        self.assertEqual(scan.unresolved[0].kind, "pub")


class TestPathClassificationTests(unittest.TestCase):
    def test_test_dir(self):
        self.assertTrue(swc._is_test_path(Path("src/rob_box_voice/test/unit/test_x.py")))

    def test_test_prefixed_file(self):
        self.assertTrue(swc._is_test_path(Path("src/rob_box_voice/rob_box_voice/test_helpers.py")))

    def test_prod_file(self):
        self.assertFalse(swc._is_test_path(Path("src/rob_box_voice/rob_box_voice/tts_node.py")))


class AllowlistTests(unittest.TestCase):
    def test_empty_reason_fails_loudly(self):
        with tempfile.TemporaryDirectory() as tmp:
            allow_path = Path(tmp) / "seam_allowlist.json"
            allow_path.write_text(
                json.dumps(
                    {
                        "publishers_without_local_subscriber": {"/x": "   "},
                        "subscribers_without_local_publisher": {},
                    }
                ),
                encoding="utf-8",
            )
            original = swc.ALLOWLIST_FILE
            swc.ALLOWLIST_FILE = allow_path
            try:
                with self.assertRaises(SystemExit) as ctx:
                    swc._load_allowlist()
                self.assertEqual(ctx.exception.code, 2)
            finally:
                swc.ALLOWLIST_FILE = original

    def test_missing_file_yields_empty_allowlist(self):
        original = swc.ALLOWLIST_FILE
        swc.ALLOWLIST_FILE = Path(tempfile.gettempdir()) / "does-not-exist-seam-allowlist.json"
        try:
            data = swc._load_allowlist()
        finally:
            swc.ALLOWLIST_FILE = original
        self.assertEqual(data["publishers_without_local_subscriber"], {})


class BaselineRoundTripTests(unittest.TestCase):
    def test_update_then_check_is_clean(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            paths = [
                _write(
                    root,
                    "src/pkg/pkg/node.py",
                    "class N:\n"
                    "    def __init__(self):\n"
                    "        self.create_publisher(String, '/orphan', 10)\n",
                )
            ]
            baseline_path = root / "seam_baseline.json"
            allow_path = root / "seam_allowlist.json"
            allow_path.write_text(
                json.dumps(
                    {"publishers_without_local_subscriber": {}, "subscribers_without_local_publisher": {}}
                ),
                encoding="utf-8",
            )
            orig_baseline, orig_allow = swc.BASELINE_FILE, swc.ALLOWLIST_FILE
            swc.BASELINE_FILE, swc.ALLOWLIST_FILE = baseline_path, allow_path
            try:
                rc = swc.cmd_update_baseline(paths, base_sha="deadbeef")
                self.assertEqual(rc, 0)
                baseline = swc._load_baseline()
                self.assertIn("/orphan", baseline["publishers_without_local_subscriber"])
                rc = swc.cmd_check(paths, baseline)
                self.assertEqual(rc, 0)  # grandfathered: no NEW violation
            finally:
                swc.BASELINE_FILE, swc.ALLOWLIST_FILE = orig_baseline, orig_allow

    def test_new_violation_not_in_baseline_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            paths = [
                _write(
                    root,
                    "src/pkg/pkg/node.py",
                    "class N:\n"
                    "    def __init__(self):\n"
                    "        self.create_publisher(String, '/brand/new/orphan', 10)\n",
                )
            ]
            allow_path = root / "seam_allowlist.json"
            allow_path.write_text(
                json.dumps(
                    {"publishers_without_local_subscriber": {}, "subscribers_without_local_publisher": {}}
                ),
                encoding="utf-8",
            )
            empty_baseline = swc._empty_baseline()
            orig_allow = swc.ALLOWLIST_FILE
            swc.ALLOWLIST_FILE = allow_path
            try:
                rc = swc.cmd_check(paths, empty_baseline)
            finally:
                swc.ALLOWLIST_FILE = orig_allow
            self.assertEqual(rc, 1)


class MsgTypeResolutionTests(unittest.TestCase):
    """Issue #2188 / voice-vr 03: ``create_publisher``/``create_subscription``
    сверяют не только имя топика, но и тип сообщения. Иначе шов
    ``create_publisher(String, "/teleop_heartbeat")`` vs
    ``create_subscription(TeleopHeartbeat, "/teleop_heartbeat")`` —
    два разных IDL-класса — проходит как «связанный», а в рантайме ROS
    не поднимает DDS-соединение.
    """

    def _scan(self, files: dict[str, str]) -> swc.SeamScan:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        paths = [_write(root, rel, content) for rel, content in files.items()]
        original_root = swc.REPO_ROOT
        swc.REPO_ROOT = root
        try:
            return swc.scan_files(paths)
        finally:
            swc.REPO_ROOT = original_root

    def test_same_msg_type_is_not_a_mismatch(self):
        scan = self._scan(
            {
                "src/rob_box_voice/rob_box_voice/tts_node.py": (
                    "from std_msgs.msg import String\n"
                    "class TtsNode:\n"
                    "    def __init__(self):\n"
                    "        self.create_publisher(String, '/tts/ready', 10)\n"
                ),
                "src/rob_box_quest/rob_box_quest/quest_node.py": (
                    "from std_msgs.msg import String\n"
                    "class QuestNode:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(String, '/tts/ready', self.cb, 10)\n"
                ),
            }
        )
        self.assertEqual(swc._collect_topic_type_mismatches(scan), [])

    def test_different_msg_types_under_same_topic_is_mismatch(self):
        # Direct shape of /teleop_heartbeat on develop (issue #2188):
        # quest_node publishes std_msgs/String, arbiter subscribes the
        # IDL TeleopHeartbeat. Same topic name, mismatched types.
        scan = self._scan(
            {
                "src/rob_box_quest/rob_box_quest/quest_node.py": (
                    "from std_msgs.msg import String\n"
                    "class QuestNode:\n"
                    "    def __init__(self):\n"
                    "        self.create_publisher(String, '/teleop_heartbeat', 10)\n"
                ),
                "src/rob_box_supervisor/rob_box_supervisor/arbiter_node.py": (
                    "from rob_box_supervisor_msgs.msg import TeleopHeartbeat\n"
                    "class AvatarArbiter:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(TeleopHeartbeat, '/teleop_heartbeat', self.cb, 10)\n"
                ),
            }
        )
        mismatches = swc._collect_topic_type_mismatches(scan)
        self.assertEqual(len(mismatches), 1)
        self.assertEqual(mismatches[0], "/teleop_heartbeat|String != TeleopHeartbeat")

    def test_unresolved_msg_type_is_skipped_not_flagged(self):
        # Calling ``self._heartbeat_msg_type`` (a method-returned IDL
        # class) cannot be resolved statically — and we do NOT want to
        # falsely flag a mismatch on it. The mismatched-by-name sibling
        # below should still be detected.
        scan = self._scan(
            {
                "src/rob_box_quest/rob_box_quest/quest_node.py": (
                    "from std_msgs.msg import String\n"
                    "class QuestNode:\n"
                    "    def __init__(self):\n"
                    "        self._heartbeat_msg_type = self._try_import()\n"
                    "        self.create_subscription(self._heartbeat_msg_type, '/x', self.cb, 10)\n"
                ),
                "src/rob_box_supervisor/rob_box_supervisor/arbiter_node.py": (
                    "from std_msgs.msg import String\n"
                    "class AvatarArbiter:\n"
                    "    def __init__(self):\n"
                    "        self.create_publisher(String, '/y', 10)\n"
                    "        self.create_subscription(String, '/x', self.cb, 10)\n"
                ),
            }
        )
        # /x has a sub but no pub in this fixture — it is NOT a type
        # mismatch candidate (no pub-type to compare against).
        # /y has only a pub — also not a mismatch candidate.
        # Both stay out of the mismatch list.
        self.assertEqual(swc._collect_topic_type_mismatches(scan), [])
        # And /x's sub is recorded as having no resolvable msg type:
        self.assertEqual(scan.unresolved_msg_types[0].expr_src, "self._heartbeat_msg_type")

    def test_import_alias_resolves_to_original_name(self):
        # ``from X import String as RosString`` followed by
        # ``create_publisher(RosString, ...)`` must normalise to ``String``
        # so the comparison works against another ``String`` import.
        scan = self._scan(
            {
                "src/rob_box_telegram/rob_box_telegram/supervisor_client.py": (
                    "from std_msgs.msg import String as RosString\n"
                    "class TelegramClient:\n"
                    "    def __init__(self):\n"
                    "        self.create_publisher(RosString, '/topic', 10)\n"
                ),
                "src/rob_box_voice/rob_box_voice/audio_node.py": (
                    "from std_msgs.msg import String\n"
                    "class AudioNode:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(String, '/topic', self.cb, 10)\n"
                ),
            }
        )
        self.assertEqual(swc._collect_topic_type_mismatches(scan), [])

    def test_dotted_msg_type_resolves_to_short_name(self):
        # ``std_msgs.msg.String`` and ``String`` should compare equal.
        scan = self._scan(
            {
                "src/rob_box_a/rob_box_a/a.py": (
                    "import std_msgs.msg\n"
                    "class A:\n"
                    "    def __init__(self):\n"
                    "        self.create_publisher(std_msgs.msg.String, '/t', 10)\n"
                ),
                "src/rob_box_b/rob_box_b/b.py": (
                    "from std_msgs.msg import String\n"
                    "class B:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(String, '/t', self.cb, 10)\n"
                ),
            }
        )
        self.assertEqual(swc._collect_topic_type_mismatches(scan), [])

    def test_lazy_import_inside_function_resolves(self):
        # rob_box_mcp_tools pattern: ``from std_msgs.msg import String``
        # happens inside ``run()``, not at module scope. Lazy-import
        # collection must catch it so the mismatch detector doesn't
        # drown MCP tools in false negatives.
        scan = self._scan(
            {
                "src/rob_box_mcp_tools/rob_box_mcp_tools/tools/say.py": (
                    "class SayTool:\n"
                    "    def run(self):\n"
                    "        from std_msgs.msg import String\n"
                    "        self.create_publisher(String, '/say', 10)\n"
                ),
                "src/rob_box_mcp_tools/rob_box_mcp_tools/tools/listener.py": (
                    "class ListenerTool:\n"
                    "    def run(self):\n"
                    "        from std_msgs.msg import String\n"
                    "        self.create_subscription(String, '/say', self.cb, 10)\n"
                ),
            }
        )
        self.assertEqual(swc._collect_topic_type_mismatches(scan), [])

    def test_baseline_round_trip_for_topic_type_mismatch(self):
        # Update baseline → mismatch recorded → check passes (no NEW
        # violation). This is the same flow cc_budget follows for its
        # categories and the contract voice-vr 03 promises Шифу.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            paths = [
                _write(
                    root,
                    "src/pkg/pkg/pub.py",
                    "from std_msgs.msg import String\n"
                    "class P:\n"
                    "    def __init__(self):\n"
                    "        self.create_publisher(String, '/x', 10)\n",
                ),
                _write(
                    root,
                    "src/pkg/pkg/sub.py",
                    "from other.msg import Bool\n"
                    "class S:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(Bool, '/x', self.cb, 10)\n",
                ),
            ]
            baseline_path = root / "seam_baseline.json"
            allow_path = root / "seam_allowlist.json"
            allow_path.write_text(
                json.dumps(
                    {"publishers_without_local_subscriber": {}, "subscribers_without_local_publisher": {}}
                ),
                encoding="utf-8",
            )
            orig_root, orig_baseline, orig_allow = (
                swc.REPO_ROOT,
                swc.BASELINE_FILE,
                swc.ALLOWLIST_FILE,
            )
            swc.REPO_ROOT, swc.BASELINE_FILE, swc.ALLOWLIST_FILE = (
                root,
                baseline_path,
                allow_path,
            )
            try:
                rc = swc.cmd_update_baseline(paths, base_sha="deadbeef")
                self.assertEqual(rc, 0)
                baseline = swc._load_baseline()
                self.assertIn("/x|String != Bool", baseline["topic_type_mismatch"])
                rc = swc.cmd_check(paths, baseline)
                self.assertEqual(rc, 0)
                # Now flip the sub to use the same type — baseline still
                # holds the old mismatch key, which is fine: it remains
                # grandfathered and doesn't generate a NEW FAIL, even
                # though it's no longer an actual mismatch on disk.
                _write(
                    root,
                    "src/pkg/pkg/sub.py",
                    "from std_msgs.msg import String\n"
                    "class S:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(String, '/x', self.cb, 10)\n",
                )
                rc = swc.cmd_check(paths, baseline)
                self.assertEqual(rc, 0)
                # And a brand-new mismatch (different topic) IS a FAIL.
                _write(
                    root,
                    "src/pkg/pkg/sub.py",
                    "from std_msgs.msg import String\n"
                    "class S:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(String, '/y', self.cb, 10)\n"
                    "        self.create_publisher(String, '/y', 10)\n",
                )
                _write(
                    root,
                    "src/pkg/pkg/sub2.py",
                    "from other.msg import Int32\n"
                    "class S2:\n"
                    "    def __init__(self):\n"
                    "        self.create_subscription(Int32, '/y', self.cb, 10)\n",
                )
                paths = [
                    Path(root / "src/pkg/pkg/pub.py"),
                    Path(root / "src/pkg/pkg/sub.py"),
                    Path(root / "src/pkg/pkg/sub2.py"),
                ]
                rc = swc.cmd_check(paths, baseline)
                self.assertEqual(rc, 1)
            finally:
                swc.REPO_ROOT, swc.BASELINE_FILE, swc.ALLOWLIST_FILE = (
                    orig_root,
                    orig_baseline,
                    orig_allow,
                )


if __name__ == "__main__":
    unittest.main()
