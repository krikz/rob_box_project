"""Ручки аранжировщика в ``compose_music`` (ADR-0132, PR-4, вариант A).

Каждая ручка ядра (PR-3: ``HarmonizeOptions``/``ArrangeOptions``) стала
отдельным плоским параметром тула. Здесь проверяется граница тула:

1. параметр доходит до опций ядра (шпион на ``melody_to_compose_params`` /
   ``spec_from_flat``) и меняет сгенерированный код;
2. неверное значение — ошибка по-русски со списком допустимых, код не
   исполняется; ручка выведенной аранжировки без ``name=`` — тоже ошибка;
3. умолчания = golden байт-в-байт (и без ручек, и с явными ``auto``,
   и со старым булевым ``theme_octaves``);
4. схема: строгие enum из ядра + текст «По умолчанию …», каталог совпадает;
5. бюджет контекста: модели — один раз компактный текст партитуры, без
   структурного dict.

Вход — замороженные RTTTL-строки golden-фикстуры, а не поиск по
библиотеке (ранжирование поиска меняется и не должно ронять тесты).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock, Mock, patch

import pytest

for _mod in [
    "rclpy", "rclpy.node", "rclpy.action", "rclpy.qos", "std_msgs",
    "std_msgs.msg", "geometry_msgs", "geometry_msgs.msg", "nav2_msgs",
    "nav2_msgs.action", "action_msgs", "action_msgs.srv", "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.core import arranger as arranger_mod  # noqa: E402
from rob_box_mcp_tools.core import rtttl_compose  # noqa: E402
from rob_box_mcp_tools.core.compose_knobs import KNOB_PARAMS  # noqa: E402
from rob_box_mcp_tools.tools.music import ComposeMusicTool  # noqa: E402

_FIXTURE = Path(__file__).parent / "fixtures" / "arranger_golden.json"
_ARR = dict(lead_synth="pluck", bass_synth="bass", pad_synth="strings")

#: Темы из PR-3 (test_arrange_options): плотная, редкая, спорная тональность,
#: с выбросами, переносимая автоматикой на октаву.
DENSE, SPARSE, KEY_DISPUTED, OUTLIERS, SHIFTED = (
    "imperial", "pinkpant", "hallofth_2", "stilldre_2", "tetris",
)

_DRUMS = "X...o...X.X.o..."
_HATS = "--.---.---.---.-"


@pytest.fixture(scope="module")
def cases():
    return json.loads(_FIXTURE.read_text(encoding="utf-8"))["cases"]


@pytest.fixture(scope="module")
def themes(cases):
    out = {}
    for case in cases:
        out.setdefault(case["key"], case["rtttl"])
    return out


def _tool(mock_node, rtttl=None):
    mgr = Mock()
    mgr.execute_code = Mock(return_value={"success": True})
    tool = ComposeMusicTool(mock_node, mgr, Mock())
    if rtttl is not None:
        rec = {"name": "theme", "title": "Theme", "rtttl": rtttl}
        tool._resolve_melody = lambda _n, _v: rec  # замороженный вход
    tool._melody_alternatives = lambda _n, _t: []
    return tool, mgr


def _play(mock_node, rtttl, **kwargs):
    """Сыграть тему с ручками; вернуть (результат, код, опции гармонизации, опции сборки)."""
    tool, mgr = _tool(mock_node, rtttl)
    real_prep, real_spec = rtttl_compose.melody_to_compose_params, arranger_mod.spec_from_flat
    with patch(
        "rob_box_mcp_tools.tools.music.melody_to_compose_params", wraps=real_prep
    ) as prep_spy, patch(
        "rob_box_mcp_tools.tools.music.spec_from_flat", wraps=real_spec
    ) as spec_spy:
        result = tool.execute(name="theme", **{**_ARR, **kwargs})
    assert result.success is True, result.error
    code = mgr.execute_code.call_args[0][0]
    return (
        result, code,
        prep_spy.call_args.kwargs["options"], spec_spy.call_args.kwargs["options"],
    )


# ---------------------------------------------------------------------------
# 1. Параметр → опции ядра → другой код
# ---------------------------------------------------------------------------

#: (параметр, значение, тема, куда приходит, поле, ожидаемое значение поля)
_REACH = [
    ("key_detection", "profile", KEY_DISPUTED, "h", "key_detection", "profile"),
    ("chords", "Am|F|C|G", DENSE, "h", "chords", ("Am", "F", "C", "G")),
    ("harmonic_rhythm", "half", DENSE, "h", "harmonic_rhythm", "half"),
    ("density", "sparse", DENSE, "h", "density", "sparse"),
    ("bass_style", "root", DENSE, "h", "bass_style", "root"),
    ("bass_approach", "off", DENSE, "h", "bass_approach", "off"),
    ("pad_style", "sustain", DENSE, "h", "pad_style", "sustain"),
    ("pad_register", "low", DENSE, "h", "pad_register", "low"),
    ("pad_register", "55-67", DENSE, "h", "pad_register", (55, 67)),
    ("counter", "off", DENSE, "a", "counter", "off"),
    ("counter", "on", SPARSE, "a", "counter", "on"),
    ("theme_octaves", "off", DENSE, "a", "theme_octaves", "off"),
    ("theme_octaves", "on", SPARSE, "a", "theme_octaves", "on"),
    # issue #2962: сдвиг теперь считается от нормализованного регистра и
    # клампится в рабочий диапазон — "-1" от DENSE (нормализован 70-86)
    # укладывается (58-74), "+1" упёрся бы в потолок 88 (86+12=98).
    ("lead_octave", "-1", DENSE, "h", "lead_octave", -1),
    ("lead_octave", "keep", SHIFTED, "h", "lead_octave", "keep"),
    ("lead_outliers", "keep", OUTLIERS, "h", "lead_outliers", "keep"),
    ("levels", "bass=0.5,pad=0.8", DENSE, "a", "levels", {"bass": 0.5, "pad": 0.8}),
    ("drums", _DRUMS, DENSE, "h", "drums", _DRUMS),
    ("hats", _HATS, DENSE, "h", "hats", _HATS),
]


@pytest.mark.parametrize(
    "param,value,theme,target,field,expected", _REACH,
    ids=[f"{c[0]}={c[1]}" for c in _REACH],
)
def test_knob_reaches_core_options_and_changes_code(
    mock_node, themes, param, value, theme, target, field, expected
):
    _res, base_code, base_h, base_a = _play(mock_node, themes[theme])
    _res, code, h_opts, a_opts = _play(mock_node, themes[theme], **{param: value})
    options = h_opts if target == "h" else a_opts
    assert getattr(options, field) == expected
    # остальные поля — по умолчанию: ручка меняет только своё
    others_h = {k: v for k, v in vars(h_opts).items() if not (target == "h" and k == field)}
    others_a = {k: v for k, v in vars(a_opts).items() if not (target == "a" and k == field)}
    assert others_h == {k: v for k, v in vars(base_h).items() if k in others_h}
    assert others_a == {k: v for k, v in vars(base_a).items() if k in others_a}
    assert code != base_code, f"{param}={value!r} не изменил код на теме {theme}"


def test_bass_and_pad_off_remove_their_players(mock_node, themes):
    _r, code, _h, _a = _play(mock_node, themes[DENSE], bass_style="off", pad_style="off")
    players = {line.split(" >>")[0] for line in code.splitlines() if " >> " in line}
    assert "p1" not in players and "p3" not in players
    assert "p2" in players  # тема на месте


def test_counter_off_removes_second_voice(mock_node, themes):
    _r, base, _h, _a = _play(mock_node, themes[DENSE])
    _r, code, _h, _a = _play(mock_node, themes[DENSE], counter="off")
    assert any(line.startswith("d3 >>") for line in base.splitlines())
    assert not any(line.startswith("d3 >>") for line in code.splitlines())


def test_knob_values_show_in_score_text(mock_node, themes):
    result, _c, _h, _a = _play(mock_node, themes[DENSE], bass_style="root", counter="off")
    assert "bass_style=root" in result.data["score"]
    assert "counter=off" in result.data["score"]


def test_levels_work_without_name(mock_node):
    tool, mgr = _tool(mock_node)
    composed = dict(
        bpm=110, root="A", scale="minor", drums="X...o...", lead_synth="pluck",
        lead_notes="0, 2, 4, 7", pad_synth="warmpad", pad_notes="0, 2, 4",
        bass_synth="dub", bass_notes="0, 4",
    )
    assert tool.execute(**composed).success is True
    base = mgr.execute_code.call_args[0][0]
    result = tool.execute(levels="bass=0.5", **composed)
    assert result.success is True, result.error
    assert mgr.execute_code.call_args[0][0] != base


# ---------------------------------------------------------------------------
# 2. Ошибки: неверное значение, ручка без name=
# ---------------------------------------------------------------------------

_BAD = [
    ("key_detection", "histogram", ["key_detection", "auto, profile"]),
    ("harmonic_rhythm", "quarter", ["harmonic_rhythm", "auto, bar, half"]),
    ("density", "medium", ["density", "auto, sparse, dense"]),
    ("bass_style", "walking", ["bass_style", "root_fifth", "pedal"]),
    ("bass_approach", "maybe", ["bass_approach", "auto, on, off"]),
    ("pad_style", "pluck", ["pad_style", "stab", "sustain"]),
    ("pad_register", "huge", ["pad_register", "low (48–60)", "mid (55–67)", "high (60–72)", "55-67"]),
    ("pad_register", "60-65", ["pad_register", "не уже октавы"]),
    ("counter", "double", ["counter", "auto, on, off"]),
    ("theme_octaves", "twice", ["theme_octaves", "auto, on, off"]),
    ("lead_octave", "+3", ["lead_octave", "keep", "-2..+2"]),
    ("lead_outliers", "drop", ["lead_outliers", "fix, keep"]),
    ("levels", "bass:0.5", ["levels", "bass=0.5,pad=0.8"]),
    ("levels", "bass=3", ["levels", "0..2"]),
    ("levels", "guitar=0.5", ["levels", "Доступны", "bass"]),
    ("chords", "Am F C G", ["chords", "Am|F|C|G"]),
    ("chords", "H|F", ["chords", "Неизвестный аккорд"]),
]


@pytest.mark.parametrize("param,value,needles", _BAD, ids=[f"{c[0]}={c[1]}" for c in _BAD])
def test_unknown_knob_value_is_a_russian_error_with_valid_values(
    mock_node, themes, param, value, needles
):
    tool, mgr = _tool(mock_node, themes[DENSE])
    result = tool.execute(name="theme", **_ARR, **{param: value})
    assert result.success is False
    for needle in needles:
        assert needle in result.error, (needle, result.error)
    mgr.execute_code.assert_not_called()


def test_more_chords_than_bars_is_an_error(mock_node, themes):
    tool, mgr = _tool(mock_node, themes[DENSE])
    result = tool.execute(name="theme", chords="|".join(["C"] * 500), **_ARR)
    assert result.success is False
    assert "аранжировку" in result.error and "такт" in result.error
    mgr.execute_code.assert_not_called()


@pytest.mark.parametrize("param,value", [
    ("bass_style", "root"), ("density", "dense"), ("counter", "on"),
    ("theme_octaves", "off"), ("chords", "Am|F"), ("lead_octave", "-1"),
    ("pad_register", "high"), ("lead_outliers", "keep"),
])
def test_derived_arrangement_knob_without_name_is_an_error(mock_node, param, value):
    tool, mgr = _tool(mock_node)
    result = tool.execute(
        bpm=110, root="A", scale="minor", lead_synth="pluck", lead_notes="0, 2, 4",
        bass_synth="dub", bass_notes="0, 4", pad_synth="warmpad", pad_notes="0, 2, 4",
        **{param: value},
    )
    assert result.success is False
    assert param in result.error and "только с name=" in result.error
    mgr.execute_code.assert_not_called()


# ---------------------------------------------------------------------------
# 3. Умолчания = golden байт-в-байт
# ---------------------------------------------------------------------------

#: Все сочетания golden. До ADR-0132 PR-6 ``ambient_brass_halftime``
#: (imperialbrass) отсюда исключался: на пути тула стоял safety net, которого
#: нет в пути ядра golden. PR-6 его удалил — теперь тул обязан играть ровно
#: golden и для imperialbrass (это и есть проверка смены дефолта).
_TOOL_COMBOS = {
    "arc_blip": dict(
        drum_style="auto", form="arc", lead_synth="blip", bass_synth="moogbass",
        pad_synth="strings", repeat=False,
    ),
    "verse_pluck_march": dict(
        drum_style="march", form="verse_chorus", lead_synth="pluck", bass_synth="bass",
        pad_synth="warmpad", counter_synth="off", theme_octaves=False,
        repeat=True, drums_sample=2, hats_sample=1,
    ),
    "ambient_brass_halftime": dict(
        drum_style="halftime", form="ambient", lead_synth="imperialbrass",
        bass_synth="subbass", pad_synth="ambi", counter_synth="pianovel",
        swing=0.1, repeat=False,
    ),
}

_ALL_AUTO = dict(
    key_detection="auto", chords="auto", harmonic_rhythm="auto", density="auto",
    bass_style="auto", bass_approach="auto", pad_style="auto", pad_register="auto",
    counter="auto", lead_octave="auto", lead_outliers="fix", levels="",
)


def _golden_code(mock_node, case, **extra):
    tool, mgr = _tool(mock_node, case["rtttl"])
    result = tool.execute(name=case["key"], **{**_TOOL_COMBOS[case["combo"]], **extra})
    assert result.success is True, result.error
    return mgr.execute_code.call_args[0][0]


def test_tool_defaults_are_byte_identical_to_golden(mock_node, cases):
    """Без ручек тул играет ровно golden (все темы × все сочетания)."""
    selected = [c for c in cases if c["combo"] in _TOOL_COMBOS]
    assert len(selected) >= 120
    diverged = [
        f"{c['key']}-{c['combo']}" for c in selected if _golden_code(mock_node, c) != c["code"]
    ]
    assert not diverged


def test_explicit_auto_knobs_are_byte_identical_to_golden(mock_node, cases):
    """Явные умолчания (auto/fix/пусто) = отсутствие ручек."""
    for case in [c for c in cases if c["combo"] == "arc_blip"][:12]:
        assert _golden_code(mock_node, case, **_ALL_AUTO, theme_octaves="auto") == case["code"]


def test_theme_octaves_legacy_bool_is_backward_compatible(mock_node, cases):
    """Старый булев контракт: true = auto, false = прежний флаг (golden verse_pluck_march)."""
    arc = next(c for c in cases if c["combo"] == "arc_blip" and c["key"] == "national_2")
    verse = next(c for c in cases if c["combo"] == "verse_pluck_march" and c["key"] == "national_2")
    assert _golden_code(mock_node, arc, theme_octaves=True) == arc["code"]
    assert _golden_code(mock_node, arc, theme_octaves="true") == arc["code"]
    assert _golden_code(mock_node, verse, theme_octaves="false") == verse["code"]


def test_imperialbrass_default_plays_counter_and_octaves_and_warns(mock_node, themes):
    """ADR-0132 PR-6: тихого safety net нет — auto играет как у любого синта,
    а партитура предупреждает о долгом хвосте и называет ручки."""
    tool, mgr = _tool(mock_node, themes[DENSE])
    tool.execute(name="theme", lead_synth="imperialbrass", bass_synth="bass", pad_synth="strings")
    score = tool.last_score
    assert score["decisions"]["counter"].startswith("auto→on")
    assert score["decisions"]["theme_octaves"].startswith("auto→on")
    assert "counter" in score["parts"] and score["parts"]["lead"]["octave_doubled"]
    assert (
        "imperialbrass: долгий релиз ≈1.5 с + второй голос + октава = 3 голоса с хвостом "
        "→ counter=off или theme_octaves=off"
    ) in score["warnings"]
    assert "долгий релиз" in tool.last_score["text"]
    assert not any("safety net" in w for w in score["warnings"])
    mgr.execute_code.assert_called_once()


@pytest.mark.parametrize("knob", [dict(counter="off"), dict(theme_octaves="off"),
                                  dict(counter_synth="off")])
def test_imperialbrass_explicit_off_is_respected_and_silences_warning(mock_node, themes, knob):
    """Модель выключила один из голосов → 2 голоса, предупреждения нет."""
    tool, _mgr = _tool(mock_node, themes[DENSE])
    tool.execute(
        name="theme", lead_synth="imperialbrass", bass_synth="bass", pad_synth="strings", **knob,
    )
    score = tool.last_score
    voices = ("counter" in score["parts"]) + bool(score["parts"]["lead"]["octave_doubled"])
    assert voices == 1
    assert not any("долгий релиз" in w for w in score["warnings"])


def test_explicit_on_knobs_are_played_with_long_release_lead(mock_node, themes):
    """Явные on — выбор модели: тул играет 3 голоса и только предупреждает."""
    tool, _mgr = _tool(mock_node, themes[DENSE])
    tool.execute(
        name="theme", lead_synth="imperialbrass", bass_synth="bass", pad_synth="strings",
        counter="on", theme_octaves="on",
    )
    score = tool.last_score
    assert score["decisions"]["counter"].startswith("on")
    assert score["decisions"]["theme_octaves"].startswith("on")
    assert any("долгий релиз" in w for w in score["warnings"])


def test_short_release_lead_with_three_voices_has_no_tail_warning(mock_node, themes):
    """brass (Env.perc — короткий хвост): 3 голоса без предупреждения."""
    tool, _mgr = _tool(mock_node, themes[DENSE])
    tool.execute(name="theme", lead_synth="brass", bass_synth="bass", pad_synth="strings")
    score = tool.last_score
    assert "counter" in score["parts"] and score["parts"]["lead"]["octave_doubled"]
    assert not any("долгий релиз" in w for w in score["warnings"])


def test_heavy_brass_safety_net_is_gone():
    assert not hasattr(ComposeMusicTool, "_heavy_brass_safety_net")
    assert not hasattr(ComposeMusicTool, "HEAVY_BRASS_LEAD_SYNTHS")


# ---------------------------------------------------------------------------
# 4. Схема: enum из ядра + «По умолчанию», каталог совпадает с тулом
# ---------------------------------------------------------------------------

_ENUMS = {
    "key_detection": ["auto", "profile"],
    "harmonic_rhythm": ["auto", "bar", "half"],
    "density": ["auto", "sparse", "dense"],
    "bass_style": ["auto", "root", "root_fifth", "pedal", "off"],
    "bass_approach": ["auto", "on", "off"],
    "pad_style": ["auto", "stab", "sustain", "off"],
    "lead_outliers": ["fix", "keep"],
    "counter": ["auto", "on", "off"],
    "theme_octaves": ["auto", "on", "off"],
    "lead_octave": ["auto", "keep", "-2", "-1", "0", "+1", "+2"],
}


def _tool_params(mock_node):
    return {p.name: p for p in ComposeMusicTool(mock_node, Mock(), Mock()).parameters}


def test_schema_has_every_knob_with_enum_and_default_text(mock_node):
    params = _tool_params(mock_node)
    for knob in KNOB_PARAMS:
        assert knob in params, knob
        param = params[knob]
        assert param.type == "string" and not param.required
        assert "По умолчанию" in param.description, knob
        assert param.enum == _ENUMS.get(knob), knob
    assert "48-60" in params["pad_register"].description
    assert "55-67" in params["pad_register"].description
    assert "60-72" in params["pad_register"].description
    assert "bass=0.5,pad=0.8" in params["levels"].description
    assert "Am|F|C|G" in params["chords"].description
    assert params["lead_outliers"].to_json_schema()["default"] == "fix"
    assert params["density"].to_json_schema()["default"] == "auto"


def test_catalog_schema_matches_tool_schema(mock_node):
    from rob_box_core._tool_catalog_data import TOOL_CATALOG_DATA

    entry = next(e for e in TOOL_CATALOG_DATA if e["name"] == "compose_music")
    props = entry["parameters"]["properties"]
    params = _tool_params(mock_node)
    for knob in KNOB_PARAMS:
        assert props[knob] == params[knob].to_json_schema(), knob
        assert knob in entry["signature"]["params"]


# ---------------------------------------------------------------------------
# 5. Бюджет контекста: модели — один раз компактный текст партитуры
# ---------------------------------------------------------------------------


def _llm_visible(result) -> str:
    """Зеркало прод-пути результата тула до LLM.

    ``mcp_server`` публикует ``result.to_dict()`` JSON-ом →
    ``rob_box_harness.executors.ros_mcp._result`` берёт ``data`` (а при его
    наличии ``message`` не показывает вовсе) → ``core_adapter._result_content``
    отдаёт ``repr`` значения.
    """
    raw = json.loads(json.dumps(result.to_dict(), ensure_ascii=False))
    return repr(raw.get("data", raw.get("message")))


def test_llm_visible_payload_has_score_text_once_and_no_dict(mock_node, themes):
    tool, mgr = _tool(mock_node, themes["national_2"])
    mgr.execute_code = Mock(side_effect=lambda code, **_kw: {
        "success": True, "message": "Код выполнен успешно", "code": code,
    })
    result = tool.execute(name="national_2", lead_synth="brass", bass_synth="moogbass", pad_synth="strings")
    assert result.success is True, result.error
    score_text = result.data["score"]
    assert isinstance(score_text, str) and score_text.startswith("Партитура")
    assert len(score_text) <= 1500
    assert score_text == tool.last_score["text"]  # полный dict — вне ответа
    payload = json.dumps(result.to_dict(), ensure_ascii=False)
    assert payload.count("Партитура") == 1
    visible = _llm_visible(result)
    # Всё, кроме самого кода трека (его размер задаёт тема), — не больше 2.5 КБ;
    # до PR-4 один только dict партитуры занимал ~3.8 КБ.
    assert len(visible) - len(repr(result.data["code"])) <= 2500
