"""Unit tests for :mod:`rob_box_voice.core.event_profile`."""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

from rob_box_voice.core.event_profile import (
    EventProfile,
    build_event_faq_prefetch_context,
    load_event_profile,
    render_event_instructions,
    render_faq_skill_prompt,
    slugify_event_id,
)


# ---------------------------------------------------------------------------
# slugify_event_id
# ---------------------------------------------------------------------------


class TestSlugifyEventId:
    def test_simple(self) -> None:
        assert slugify_event_id("RoboConf") == "roboconf"

    def test_with_punctuation(self) -> None:
        assert slugify_event_id("RoboConf 2026!") == "roboconf-2026"

    def test_russian(self) -> None:
        # NB: regex [^a-zA-Z0-9] strips Russian chars too (they are
        # not in [a-zA-Z0-9]). This is the existing legacy behaviour
        # from DialogueNode._slugify_event_id — preserved on purpose.
        assert slugify_event_id("Фестиваль Роботов") == "faq-event"
        # Mixed script — only Latin survives.
        assert slugify_event_id("RoboConf 2026 Москва") == "roboconf-2026"

    def test_special_chars_only_become_separators(self) -> None:
        assert slugify_event_id("Foo --- Bar") == "foo-bar"
        # underscores → separators
        assert slugify_event_id("foo_bar_baz") == "foo-bar-baz"

    def test_empty_returns_default(self) -> None:
        assert slugify_event_id("") == "faq-event"
        assert slugify_event_id("!@#") == "faq-event"


# ---------------------------------------------------------------------------
# load_event_profile
# ---------------------------------------------------------------------------


def _write_yaml(tmp_path: Path, body: str) -> str:
    path = tmp_path / "event.yaml"
    path.write_text(textwrap.dedent(body).strip() + "\n", encoding="utf-8")
    return str(path)


class TestLoadEventProfile:
    def test_disabled_returns_none(self, tmp_path: Path) -> None:
        cfg = _write_yaml(tmp_path, "event:\n  name: Test\n")
        assert load_event_profile(enabled=False, config_file=cfg) is None

    def test_missing_file_returns_none(self, tmp_path: Path) -> None:
        missing = str(tmp_path / "does_not_exist.yaml")
        assert load_event_profile(enabled=True, config_file=missing) is None

    def test_empty_config_file_returns_none(self) -> None:
        assert load_event_profile(enabled=True, config_file="") is None

    def test_loads_minimal_event(self, tmp_path: Path) -> None:
        faq = tmp_path / "my-faq.json"
        faq.write_text("[]", encoding="utf-8")
        cfg = _write_yaml(
            tmp_path,
            f"""\
            event:
              name: RoboConf 2026
              faq_file: {faq}
            """,
        )
        profile = load_event_profile(enabled=True, config_file=cfg)
        assert profile is not None
        assert isinstance(profile, EventProfile)
        assert profile.name == "RoboConf 2026"
        # Slug = name + '-' + faq_path.stem (underscores become separators)
        assert profile.event_id == "roboconf-2026-my-faq"
        assert profile.faq_file == str(faq)

    def test_loads_full_event(self, tmp_path: Path) -> None:
        faq = tmp_path / "faq.json"
        faq.write_text("[]", encoding="utf-8")
        cfg = _write_yaml(
            tmp_path,
            f"""\
            event:
              name: Open Day
              faq_file: {faq}
              id: open-day-2026
              organization: Lab42
              location: Moscow
              date: 2026-08-15
              description: Robotics open doors
              robot_role: Tour Guide
              intro_identity: Я экскурсовод
            """,
        )
        profile = load_event_profile(enabled=True, config_file=cfg)
        assert profile is not None
        assert profile.event_id == "open-day-2026"
        assert profile.organization == "Lab42"
        assert profile.location == "Moscow"
        assert profile.date == "2026-08-15"
        assert profile.description == "Robotics open doors"
        assert profile.robot_role == "Tour Guide"
        assert profile.intro_identity == "Я экскурсовод"

    def test_relative_faq_file_resolved_against_config(self, tmp_path: Path) -> None:
        # faq file lives next to the config
        faq = tmp_path / "faq.json"
        faq.write_text("[]", encoding="utf-8")
        cfg = _write_yaml(
            tmp_path,
            f"""\
            event:
              name: X
              faq_file: faq.json
            """,
        )
        profile = load_event_profile(enabled=True, config_file=cfg)
        assert profile is not None
        assert profile.faq_file == str(faq.resolve())

    def test_payload_without_event_key(self, tmp_path: Path) -> None:
        # Legacy format — the entire payload IS the event dict.
        faq = tmp_path / "faq.json"
        faq.write_text("[]", encoding="utf-8")
        cfg = _write_yaml(
            tmp_path,
            f"""\
            name: Just name
            faq_file: {faq}
            """,
        )
        profile = load_event_profile(enabled=True, config_file=cfg)
        assert profile is not None
        assert profile.name == "Just name"

    def test_missing_required_fields(self, tmp_path: Path) -> None:
        cfg = _write_yaml(tmp_path, "event:\n  name: only name\n")
        # Missing faq_file → returns None
        assert load_event_profile(enabled=True, config_file=cfg) is None

    def test_malformed_yaml_returns_none(self, tmp_path: Path) -> None:
        cfg = tmp_path / "broken.yaml"
        cfg.write_text(":::bad::: yaml :::", encoding="utf-8")
        assert load_event_profile(enabled=True, config_file=str(cfg)) is None


# ---------------------------------------------------------------------------
# render_event_instructions
# ---------------------------------------------------------------------------


class TestRenderEventInstructions:
    def test_no_profile_returns_base(self) -> None:
        assert render_event_instructions(None, "BASE") == "BASE"

    def test_minimal_profile(self) -> None:
        profile = EventProfile(
            event_id="e1",
            name="Demo",
            organization="",
            location="",
            date="",
            description="",
            robot_role="",
            intro_identity="",
            faq_file="",
        )
        out = render_event_instructions(profile, "BASE")
        assert "[EVENT MODE]" in out
        assert "Мероприятие: Demo." in out
        assert "BASE" in out

    def test_full_profile_includes_all_optional_fields(self) -> None:
        profile = EventProfile(
            event_id="e1",
            name="Demo",
            organization="Lab",
            location="Moscow",
            date="2026-08-15",
            description="Desc",
            robot_role="Guide",
            intro_identity="I'm a guide",
            faq_file="/tmp/x",
        )
        out = render_event_instructions(profile, "BASE")
        for fragment in [
            "Demo",
            "Lab",
            "Moscow",
            "2026-08-15",
            "Desc",
            "Guide",
            "I'm a guide",
            "BASE",
        ]:
            assert fragment in out

    def test_faq_guidance_appended_when_store_available(self) -> None:
        profile = EventProfile(
            event_id="e1",
            name="Demo",
            organization="",
            location="",
            date="",
            description="",
            robot_role="",
            intro_identity="",
            faq_file="",
        )
        out = render_event_instructions(profile, "BASE", faq_store_available=True)
        assert "FAQ retrieval tool" in out
        assert "handle_music" in out

    def test_faq_guidance_absent_when_no_store(self) -> None:
        profile = EventProfile(
            event_id="e1",
            name="Demo",
            organization="",
            location="",
            date="",
            description="",
            robot_role="",
            intro_identity="",
            faq_file="",
        )
        out = render_event_instructions(profile, "BASE", faq_store_available=False)
        assert "FAQ retrieval tool" not in out


# ---------------------------------------------------------------------------
# render_faq_skill_prompt
# ---------------------------------------------------------------------------


class TestRenderFaqSkillPrompt:
    def test_no_profile_returns_base(self) -> None:
        assert render_faq_skill_prompt(None, "BASE") == "BASE"

    def test_includes_role_and_name(self) -> None:
        profile = EventProfile(
            event_id="e1",
            name="Demo",
            organization="Lab",
            location="Moscow",
            date="",
            description="",
            robot_role="Guide",
            intro_identity="",
            faq_file="",
        )
        out = render_faq_skill_prompt(profile, "BASE")
        assert "Demo" in out
        assert "Guide" in out
        assert "Lab" in out
        assert "Moscow" in out
        assert "BASE" in out


# ---------------------------------------------------------------------------
# build_event_faq_prefetch_context
# ---------------------------------------------------------------------------


class _FakeFAQStore:
    """Minimal stand-in for the real FAQStore — just .search(query, event_id, limit)."""

    def __init__(self, results: list[dict] | None = None, raises: Exception | None = None) -> None:
        self._results = results or []
        self._raises = raises

    def search(self, *, query: str, event_id: str, limit: int) -> list[dict]:
        if self._raises is not None:
            raise self._raises
        return self._results[:limit]


class TestBuildEventFaqPrefetchContext:
    def _profile(self) -> EventProfile:
        return EventProfile(
            event_id="e1",
            name="Demo",
            organization="",
            location="",
            date="",
            description="",
            robot_role="",
            intro_identity="",
            faq_file="",
        )

    def test_no_store_returns_none(self) -> None:
        assert (
            build_event_faq_prefetch_context(
                profile=self._profile(),
                faq_store=None,
                user_input="hello",
            )
            is None
        )

    def test_empty_results_returns_none(self) -> None:
        store = _FakeFAQStore(results=[])
        assert (
            build_event_faq_prefetch_context(
                profile=self._profile(),
                faq_store=store,
                user_input="hello",
            )
            is None
        )

    def test_results_rendered(self) -> None:
        store = _FakeFAQStore(
            results=[
                {
                    "question": "Где туалет?",
                    "answer": "За углом.",
                    "category": "facilities",
                    "source": "site_map.pdf",
                },
            ]
        )
        out = build_event_faq_prefetch_context(
            profile=self._profile(),
            faq_store=store,
            user_input="где туалет?",
            limit=3,
        )
        assert out is not None
        assert "[EVENT FAQ PREFETCH]" in out
        assert "Где туалет?" in out
        assert "За углом." in out
        assert "facilities" in out
        assert "site_map.pdf" in out

    def test_store_exception_returns_none(self) -> None:
        store = _FakeFAQStore(raises=RuntimeError("boom"))
        assert (
            build_event_faq_prefetch_context(
                profile=self._profile(),
                faq_store=store,
                user_input="hello",
            )
            is None
        )

    def test_empty_user_input_returns_none(self) -> None:
        store = _FakeFAQStore(results=[{"question": "q", "answer": "a"}])
        assert (
            build_event_faq_prefetch_context(
                profile=self._profile(),
                faq_store=store,
                user_input="   ",
            )
            is None
        )