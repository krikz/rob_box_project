"""Таблица знания: вейки совпадают с YAML робота, Tier-1, громкость, классы действий, экспрессия и схема команды
согласованы с каталогом тулов и лабораторией лица (ADR-0150 PR-1)."""

import ast
import pathlib

import pytest
import yaml

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import knowledge as kn

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
#: Сцены процедурного лица (ADR-0150 PR-3a #3689: генератор переехал из tools/led_face_lab в пакет).
FACE = REPO_ROOT / "src" / "rob_box_animations" / "rob_box_animations" / "face"
WAKE_YAML = REPO_ROOT / "docker" / "sound" / "config" / "wake_words.yaml"


def test_wake_words_yaml_matches_code():
    data = yaml.safe_load(WAKE_YAML.read_text(encoding="utf-8"))
    plain = [w for w in data["personality"] if isinstance(w, str)]
    risky = [w["word"] for w in data["personality"] if isinstance(w, dict) and w.get("risky") is True]
    assert plain == list(kn.DEFAULT_WAKE_WORDS)
    assert risky == list(kn.RISKY_WAKE_WORDS)
    assert data["personality"][-len(risky):] == [{"word": w, "risky": True} for w in risky]  # рискованные — в конце
    assert data["risky_enabled"] is False  # В18: выключены по умолчанию
    assert data["operator"] == list(kn.DEFAULT_OPERATOR_WAKE_WORDS)
    expected = (list(kn.DEFAULT_WAKE_WORDS), list(kn.DEFAULT_OPERATOR_WAKE_WORDS))
    assert kn.resolve_wake_word_namespaces(str(WAKE_YAML)) == expected


def test_risky_wake_words_switch_on_with_one_line(tmp_path):
    text = WAKE_YAML.read_text(encoding="utf-8").replace("risky_enabled: false", "risky_enabled: true")
    path = tmp_path / "wake_words.yaml"
    path.write_text(text, encoding="utf-8")
    personality, _ = kn.resolve_wake_word_namespaces(str(path))
    assert personality == list(kn.DEFAULT_WAKE_WORDS) + list(kn.RISKY_WAKE_WORDS)


def test_wake_words_loader_is_safe(tmp_path):
    broken = tmp_path / "w.yaml"
    broken.write_text("personality: [unclosed", encoding="utf-8")
    assert kn.load_wake_word_namespaces(str(broken)) == {}
    assert kn.load_wake_word_namespaces(str(tmp_path / "missing.yaml")) == {}
    assert kn.load_wake_word_namespaces("") == {}
    assert kn.resolve_wake_word_namespaces(None, ["робот"], ["тарс"]) == (["робот"], ["тарс"])


def test_wake_words_loader_keeps_file_order(tmp_path):
    custom = tmp_path / "w.yaml"
    custom.write_text("personality: [робби, робот]\noperator: [тарс]\n", encoding="utf-8")
    assert kn.load_wake_word_namespaces(str(custom)) == {"personality": ["робби", "робот"], "operator": ["тарс"]}


def test_longer_wake_variants_come_before_their_prefixes():
    words = kn.DEFAULT_WAKE_WORDS
    for i, short in enumerate(words):
        for longer in words[i + 1:]:
            assert not longer.startswith(short), f"«{longer}» стоит после своего префикса «{short}»"


def test_tier1_policy_uses_known_policies():
    for group in kn.TIER1_POLICY.values():
        assert group.policy in kn.POLICIES and group.boundary in kn.BOUNDARIES and group.words


def test_volume_table():
    assert set(kn.VOLUME_STEMS) == set(kn.VOLUME_INTENTS)
    assert set(kn.VOLUME_CHANNELS) == {"tts", "music", "sfx"}
    catalog = {t.name for t in TOOL_CATALOG}
    assert {c.tool for c in kn.VOLUME_CHANNELS.values() if c.tool} <= catalog


def _literal(path: pathlib.Path, name: str):
    """Значение присваивания ``name = <литерал>`` модуля без его импорта (тянет numpy/ROS); у dict — ключи."""
    for node in ast.parse(path.read_text(encoding="utf-8")).body:
        targets = node.targets if isinstance(node, ast.Assign) else [getattr(node, "target", None)]
        if any(isinstance(t, ast.Name) and t.id == name for t in targets):
            if isinstance(node.value, ast.Dict):
                return [ast.literal_eval(k) for k in node.value.keys]
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} не найден в {path}")


def test_every_catalog_tool_has_an_action_class():
    names = {t.name for t in TOOL_CATALOG}
    classified = set(kn.TOOL_CLASSES) | set(kn.SPEECH_CHANNEL_TOOLS)
    assert names - classified == set(), "тул каталога без класса действия"
    assert classified - names == set(), "класс у тула, которого нет в каталоге"
    assert set(kn.TOOL_CLASSES.values()) <= set(kn.ACTION_CLASSES)


def test_action_classes_follow_adr_5_1():
    for name, cls in kn.ACTION_CLASSES.items():
        assert cls.timeout_s is None or cls.timeout_s > 0, name
        assert set(cls.phrases) <= set(kn.ACTION_STATUSES), name
        assert set(cls.agents) <= set(kn.AGENTS), name
    motion = kn.ACTION_CLASSES["action.motion"]
    assert (motion.timeout_s, motion.interruptible) == (120.0, False)
    assert kn.ACTION_CLASSES["action.media"].timeout_s == kn.MUSIC_START_WAIT_S + kn.MUSIC_PREPARE_S == 10.0
    assert kn.PERFORM_BACKING_START_S == kn.MUSIC_START_WAIT_S == 6.0  # подложка ждёт старт, как раньше
    assert kn.ACTION_CLASSES["action.local"].timeout_s == 3.0
    assert (kn.ACTION_CLASSES["query"].timeout_s, kn.ACTION_CLASSES["query"].interruptible) == (15.0, True)
    assert kn.ACTION_CLASSES["action.speak_aloud"].agents == ("operator", "telegram")
    assert motion.phrases["pending"] == "motion.started"
    assert set(kn.SINGLE_ACTION_CLASSES) <= set(kn.ACTION_CLASSES)


def test_emotions_and_gestures_are_scenes_of_player_or_lab():
    known = set(kn.KNOWN_ANIMATIONS)  # словарь тулов - здесь же, один источник (#4103)
    lab = set(_literal(FACE / "states.py", "SCENES"))
    assert set(kn.LAB_SCENES) <= lab
    scenes = known | set(kn.LAB_SCENES)
    assert set(kn.EMOTIONS.values()) <= scenes
    assert set(kn.GESTURES) <= scenes


def _walk(schema):
    yield schema
    for sub in schema.get("properties", {}).values():
        yield from _walk(sub)
    if isinstance(schema.get("items"), dict):
        yield from _walk(schema["items"])


def test_command_schema_modes():
    schema = kn.command_schema("json")
    assert schema["properties"]["speech"]["maxLength"] == kn.SPEECH_MAX_CHARS
    assert schema["properties"]["actions"]["maxItems"] == kn.ACTIONS_MAX
    face = schema["properties"]["expression"]["properties"]["face"]["properties"]
    assert {k: tuple(v["enum"]) for k, v in face.items() if "enum" in v} == dict(kn.FACE_SCENE_SLOTS)
    for node in _walk(schema):
        assert node.get("type") in {"object", "array", "string", "integer", "number", "boolean"}
        if "enum" in node:
            assert node["enum"], node
    tool = kn.command_schema("tool")
    assert tool["function"]["name"] == kn.COMMAND_TOOL and tool["function"]["parameters"] == schema
    with pytest.raises(ValueError):
        kn.command_schema("xml")


def test_prompt_fragment_names_every_enum_value():
    text = kn.prompt_fragment()
    values = [*kn.EMOTIONS, *kn.GESTURES, *kn.EARCONS, *kn.PERFORM_KINDS, *kn.WHEN, *kn.ASK_EXPECTS, *kn.PASSED,
              *kn.ROLE_GENDERS, *(v for vs in kn.FACE_SCENE_SLOTS.values() for v in vs)]
    assert [v for v in values if v not in text] == []
    assert str(kn.SPEECH_MAX_CHARS) in text and str(kn.ACTIONS_MAX) in text
    # PR-7 ч.7c: об итоге действий говорит код; ответ по сути при действиях — perform; голос реплики; роли (#4084) —
    # герои и чья реплика, голоса им даёт код (поля voice_per_segment в схеме и в промпте нет)
    assert "скажет код" in text and "- voice:" in text and "voice_per_segment" not in text
    assert all(field in text for field in kn.ROLE_FIELDS) and "Голоса ролям даёт код" in text


def test_prompt_fragment_of_a_sung_turn_has_no_roles_4084():
    """Путь живого пения (песня/рэп) — схема без полей ролей, и промпт о них молчит: всё голосом робота."""
    sung = kn.with_sing(kn.command_schema("json"), "live:song")
    assert not set(kn.ROLE_FIELDS) & set(sung["properties"]["perform"]["properties"])
    text = kn.prompt_fragment(sung)
    assert "cast" not in text and "голосом робота" in text
    generate = kn.with_sing(kn.command_schema("json"), "generate:robot_song:V6")  # петь нельзя — сказка/стих с ролями
    assert set(kn.ROLE_FIELDS) <= set(generate["properties"]["perform"]["properties"])


def test_outcome_tables_name_action_tools_of_the_catalog():
    """Фразу об исходе этих тулов говорит не исполнитель (PR-7 ч.7c): тулы — действия из таблицы классов."""
    for tool in (*kn.OUTCOME_SPOKEN_ELSEWHERE, *kn.SILENT_DONE_TOOLS, *kn.TOOL_PHRASES):
        assert kn.TOOL_CLASSES[tool].startswith("action."), tool
    assert set(kn.OUTCOME_SPOKEN_ELSEWHERE.values()) <= {"tool", "music_turn"}


def test_session_budgets():
    assert 60 <= kn.TURN_CEILING_S <= 90 and kn.PROGRESS_AFTER_S == 6.0  # В31: потолок 60–90 с, фраза на ~6 с
    assert kn.SILENCE_TTL_S == 600
    assert kn.IDENTITY_TTL_S == 15
    assert kn.REPLY_WINDOW_S == 15 == kn.IDENTITY_TTL_S  # одна константа окна ответа (Identity и приветствие)
    assert set(kn.SCENE_KINDS) == {"Silence", "Persona", "DJ", "OperatorHold", "Identity", "GreetingReply",
                                   "PerformAsk", "ToolAsk"}


def test_reflexes_use_known_scenes_ring_modes_and_earcons():
    known = set(kn.KNOWN_ANIMATIONS)  # словарь тулов - здесь же, один источник (#4103)
    scenes = known | set(kn.LAB_SCENES)
    for stage, reflex in kn.REFLEXES.items():
        assert set(reflex.face) <= scenes, stage
        assert reflex.ring is None or reflex.ring in kn.RING_MODES, stage
        assert reflex.earcon is None or reflex.earcon in kn.EARCONS, stage
    for faces in kn.EXECUTING_FACE.values():
        assert set(faces) <= scenes
    assert {scene for _, scene in kn.MUSIC_SCENE_BPM} <= scenes
    assert set(kn.MUSIC_SCENES) <= scenes
    assert set(kn.TURN_STAGES) <= set(kn.REFLEXES)
    assert set(kn.EXECUTING_FACE) <= set(kn.ACTION_CLASSES)
    assert kn.REFLEXES["addressed"].earcon == "heard" and kn.REFLEXES["addressed"].deadline_ms == 300
    assert set(kn.TOOL_PHRASES) <= set(kn.TOOL_CLASSES)


def test_4010_music_scenes_have_one_table_of_knowledge():
    """Сцены музыки лица - одна таблица: рефлексы и сцена на ``executing`` берут её, а не пишут своё перечисление."""
    assert kn.MUSIC_SCENES == tuple(scene for _, scene in kn.MUSIC_SCENE_BPM) and len(set(kn.MUSIC_SCENES)) == 3
    assert kn.REFLEXES["music.playing"].face == kn.MUSIC_SCENES == kn.EXECUTING_FACE["action.media"]
    limits = [limit for limit, _ in kn.MUSIC_SCENE_BPM]
    assert limits == sorted(limits) and limits[-1] == float("inf")
    assert kn.MUSIC_SCENE_BPM[0][0] <= kn.MUSIC_DEFAULT_BPM < kn.MUSIC_SCENE_BPM[1][0]
    assert 0 < kn.MUSIC_SCENE_MIN_S <= kn.MUSIC_SCENE_HOLD_S


def test_reflex_ttl_is_set_for_every_turn_stage_and_fits_the_face_limit():
    # face_node держит сцену запроса не дольше REQUEST_MAX_S (60 с); терминальные стадии короче ожидания
    for stage in kn.TURN_STAGES:
        ttl = kn.REFLEXES[stage].ttl_ms
        assert ttl is not None and 0 < ttl <= 60_000, stage
    assert max(kn.REFLEXES[s].ttl_ms for s in ("done", "failed", "timeout")) < kn.REFLEXES["thinking"].ttl_ms


def test_calendar_themes_exist_in_rear_lights():
    themes = set(_literal(FACE / "rear.py", "THEMES"))
    for theme in kn.CALENDAR:
        assert theme.rear in themes, theme
        assert 1 <= theme.month <= 12 and 1 <= theme.day <= 31
        assert theme.scene is None or theme.scene in kn.LAB_SCENES


def test_composition_covers_every_pair_with_known_policy():
    pairs = {(n, b) for n in kn.COMPOSITION_NEW for b in kn.COMPOSITION_BUSY}
    assert set(kn.COMPOSITION) == pairs
    for pair, rule in kn.COMPOSITION.items():
        assert rule.policy in kn.POLICIES, pair
        assert rule.boundary is None or rule.boundary in kn.BOUNDARIES, pair
        assert (rule.policy == "REJECT") == (rule.ask is not None), pair


@pytest.mark.parametrize("pair, policy, boundary", [
    (("speech", "speech"), "MERGE", "segment"),        # О7: речь во время речи — на границе сегмента
    (("perform", "track"), "REJECT", None),            # О9 (б): спросить, а не вытеснить трек молча
    (("music", "perform"), "REPLACE", "bar"),          # «добавь бит во время рэпа» — речь не рвётся
    (("motion", "track"), "MERGE", None),              # «направо» во время песни — параллельно
    (("halt", "motion"), "REPLACE", "now"),            # «стой» — сразу
    (("sfx", "sfx"), "APPEND", "block"),
])
def test_composition_reference_cells(pair, policy, boundary):
    rule = kn.COMPOSITION[pair]
    assert (rule.policy, rule.boundary) == (policy, boundary)


def test_sound_stems_are_one_table_without_repeats():
    """Сверка названий звуков с ``sound_pack/sound_catalog.json`` - в ``tests/unit/dialog/test_sound_aliases.py`` (в сборке
    пакета корня репозитория нет, #3987). Здесь - только согласованность самой таблицы."""
    assert kn.TOOL_GROUPS["sound"].stems == kn.SOUND_STEMS and "звук" in kn.SOUND_STEMS
    assert len(set(kn.SOUND_STEMS)) == len(kn.SOUND_STEMS)
    assert not {t for ts, _ in kn.SOUND_ALIASES_RU for t in ts} & set(kn.SOUND_UNNAMED)


def test_ambiguous_tempo_stems_are_a_subset_of_one_table():
    """Слова про скорость - одна таблица: голос берёт их из ``VOICE_PACE_STEMS``, музыка - из ``MUSIC_AMBIGUOUS_STEMS``."""
    assert set(kn.VOICE_PACE_STEMS) <= set(kn.TOOL_GROUPS["voice"].stems)
    assert set(kn.VOICE_PACE_STEMS) <= set(kn.MUSIC_AMBIGUOUS_STEMS)
