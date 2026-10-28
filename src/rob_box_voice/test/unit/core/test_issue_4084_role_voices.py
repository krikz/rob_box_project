"""#4084: голоса ролей выступления решает код - на настоящем пути хоста.

Живой случай 27.10 (dev-0d94634): «Робот, спой песню про енота» - куплеты через один голосом ``Russian_ReliableMan`` и
``Russian_HandsomeChildhoodFriend``; анекдот - пять голосов на трёх героев. Голос каждого сегмента выбирала модель.

Теперь: каталог ролей (``tts_voice_registry.role_voices_for``: русские голоса звучащего провайдера без рассказчика;
голоса каста марафона нерусские, поэтому в него не входят) с полом голоса -> ``plan_host.perform_busy`` -> валидатор команды
(``rob_box_dialog.cast``: голос героя - по полу, один на всё выступление; песня и рэп - голосом робота) ->
``PerformHost`` -> ``SpeechRequest.voice``. Правка идущего выступления видит роли и голоса хоста (``Busy.cast``).
"""

from __future__ import annotations

import os
import re
import types
from pathlib import Path
from typing import Any, Dict, List

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, Invalid, parse, validate

from rob_box_voice.core import plan_host
from rob_box_voice.core.perform_host import PerformHost
from rob_box_voice.tts_voice_registry import (
    PROVIDER_VOICES,
    ROLE_VOICE_LANGUAGE,
    VOICE_METADATA,
    default_voice_for,
    gender_of,
    language_of,
    role_voices_for,
)

M = install_dialog_msgs_stub()
TIME = type(M.SpeechEvent().stamp)
ALL_TOOLS = [t.name for t in TOOL_CATALOG]
LIB = Path(".github") / "workflows" / "scripts" / "e2e_voice_lib.sh"


def _repo_root(start: Path) -> Path:
    """Корень репо в dev и в CI (test_ws, туда копируется ``.github``), как в test_issue_3114_scsynth_block_size."""
    override = os.environ.get("ROB_BOX_REPO_ROOT")
    if override and (Path(override) / LIB).is_file():
        return Path(override).resolve()
    for parent in [start, *start.parents]:
        if (parent / LIB).is_file():
            return parent
    raise RuntimeError(f"repo root not found for {start!s}; set ROB_BOX_REPO_ROOT")


E2E_LIB = _repo_root(Path(__file__).resolve()) / LIB

TALE = {
    "kind": "tale",
    "backing": "none",
    "segments": [
        "Жил-был в лесу ёжик.",
        "Лиса, давай дружить!",
        "Давай, колючий.",
        "И пошли они вместе.",
        "Смотри, грибы!",
        "Чур, мои!",
    ],
    "cast": [{"name": "Ёж", "gender": "male"}, {"name": "Лиса", "gender": "female"}],
    "role_per_segment": ["", "Ёж", "Лиса", "", "Ёж", "Лиса"],
}
RACCOON = {
    "kind": "song",
    "backing": "none",
    "segments": [
        "Ночью енот выходит на промысел.",
        "Полосатый хвост, как тельняшка.",
        "Енот-енот, ночной пилот.",
        "Утром лужица, усы в сметане.",
    ],
    "voice_per_segment": [
        "Russian_ReliableMan",
        "Russian_HandsomeChildhoodFriend",
        "Russian_ReliableMan",
        "Russian_HandsomeChildhoodFriend",
    ],
}


class _Pub:
    def __init__(self) -> None:
        self.sent: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.sent.append(msg)


class _Node:
    """Нода v2: публикаторы речи, часы, параметр ``tts_provider`` и исполнитель тулов с хостом выступления."""

    def __init__(self, provider: str = "minimax") -> None:
        self.pubs: Dict[str, _Pub] = {}
        self.now_s = 1000.0
        self._actual_tts_provider = provider
        self._tool_executor = types.SimpleNamespace(performer=None)

    def create_publisher(self, msg_type: Any, topic: str, depth: int) -> _Pub:
        return self.pubs.setdefault(topic, _Pub())

    def get_clock(self) -> Any:
        ns = int(self.now_s * 1e9)
        stamp = TIME(sec=ns // 10**9, nanosec=ns % 10**9)
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=ns, to_msg=lambda: stamp)
        )

    def get_logger(self) -> Any:
        return types.SimpleNamespace(info=lambda _: None)

    def get_parameter(self, name: str) -> Any:
        raise KeyError(name)

    def voices_requested(self) -> List[str]:
        """Голоса пред-синтеза сегментов (``commit=false``) - по одному запросу на сегмент, в порядке звучания."""
        sent = self.pubs.get("/voice/speech/request", _Pub()).sent
        return [r.voice for r in sent if not r.commit]


def _performing_node(provider: str = "minimax") -> _Node:
    node = _Node(provider)
    node._tool_executor.performer = PerformHost(
        node,
        lambda call: None,
        lambda: None,
        lambda event: None,
        kn.SPEECH_OUTPUT_LATENCY_MS,
        clock=lambda: node.now_s,
    )
    return node


def _judge(node: _Node, raw: Dict[str, Any]) -> Command:
    res = validate(parse(raw), ALL_TOOLS, busy=plan_host.perform_busy(node))
    assert isinstance(res, Command), res
    return res


def _perform(node: _Node, perform: Dict[str, Any]) -> List[str]:
    """Команда модели -> валидатор -> хост выступления; голоса запросов речи (пред-синтез сегментов)."""
    node._tool_executor.performer.start(
        _judge(node, {"perform": perform}).perform, "t-4084"
    )
    return node.voices_requested()


# -- каталог ролей ------------------------------------------------------------------------------------------------


def _harness_people_voices() -> set:
    """Голоса людей e2e-харнесса (каст марафона): единственный источник - ``map_tts_voice`` харнесса, голос человека там
    с просодией ``id:pitch:speed`` (#3931). Отдельного списка каста в реестре нет (ревью PR #4112)."""
    text = E2E_LIB.read_text(encoding="utf-8")
    arm_start = text.index("        minimax)", text.index("map_tts_voice() {"))
    arm = text[arm_start : text.index("            ;;", arm_start)]
    return {spec.split(":")[0] for spec in re.findall(r"printf '([^'%][^':]*:[^']*)'", arm)}


def test_harness_people_voices_are_in_the_catalogue_with_a_native_language_that_is_not_a_role_language():
    """Каст не входит в роли самим фильтром языка (``role_voices_for``): голос харнесса в каталоге с нативным нерусским
    языком. Харнесс получит русский голос - тест красный раньше, чем герой сказки зазвучит «Сашей» (#4084)."""
    people = _harness_people_voices()
    assert people, "голоса людей в map_tts_voice не нашлись - изменился разбор харнесса"
    for voice in people:
        assert voice in PROVIDER_VOICES["minimax"] and f"minimax:{voice}" in VOICE_METADATA, voice
        assert language_of("minimax", voice) not in ("", ROLE_VOICE_LANGUAGE), voice


def test_harness_people_voices_never_voice_a_role_of_any_provider():
    people = {f"minimax:{v}" for v in _harness_people_voices()}
    for provider in PROVIDER_VOICES:
        roles = {f"{provider}:{v}" for v in role_voices_for(provider, default_voice_for(provider))}
        assert roles and not roles & people, provider


def test_busy_carries_the_gender_of_every_role_voice():
    busy = plan_host.perform_busy(_Node("yandex"))
    assert busy.voices and [v for v, _ in busy.voice_genders] == list(busy.voices)
    assert all(g == gender_of("yandex", v) and g for v, g in busy.voice_genders)


# -- путь хоста -----------------------------------------------------------------------------------------------------


def test_raccoon_song_reaches_the_speech_owner_in_the_robot_voice():
    assert _perform(_performing_node(), RACCOON) == [""] * len(RACCOON["segments"])


def test_tale_reaches_the_speech_owner_with_one_voice_per_role():
    voices = _perform(_performing_node(), TALE)
    narrator, hedgehog, fox = voices[0], voices[1], voices[2]
    assert voices == [narrator, hedgehog, fox, narrator, hedgehog, fox]
    assert narrator == "" and len({narrator, hedgehog, fox}) == 3
    assert (gender_of("minimax", hedgehog), gender_of("minimax", fox)) == (
        "male",
        "female",
    )
    assert hedgehog.startswith("Russian_") and fox.startswith("Russian_")


def test_an_edit_of_the_running_tale_keeps_the_heroes_voices():
    """Правка (``edit_pending``) или новое выступление поверх идущего (MERGE): героев с тем же именем не перекрашивает,
    даже если модель перечислила их в другом порядке; новый герой - свободный голос."""
    node = _performing_node()
    voices = _perform(node, TALE)
    host = node._tool_executor.performer
    assert host.cast == {"еж": voices[1], "лиса": voices[2]}
    assert plan_host.perform_busy(node).cast == tuple(host.cast.items())

    edit = {
        "kind": "tale",
        "segments": ["Пришёл волк.", "Беги, ёжик!", "Не боюсь!"],
        "cast": [
            {"name": "Волк", "gender": "male"},
            {"name": "Лиса", "gender": "female"},
            {"name": "ёж", "gender": "male"},
        ],
        "role_per_segment": ["Волк", "Лиса", "Ёж"],
    }
    res = _judge(node, {"edit_pending": True, "perform": edit})
    wolf, fox, hedgehog = res.perform.voice_per_segment
    assert (fox, hedgehog) == (voices[2], voices[1])
    assert wolf and wolf not in voices


def test_an_edit_with_more_new_heroes_than_free_voices_is_refused_not_given_the_narrators_voice():
    """Ревью PR #4112: каталог Silero - четыре голоса ролей; два держат ёж и лиса идущей сказки. Правка с тремя новыми
    героями: свободных голосов два - нарушение ``bad_cast`` (сверка шла со всем каталогом, и третий герой получил бы
    голос рассказчика). Двое новых - проходят и берут свободные голоса."""
    node = _performing_node("silero")
    voices = _perform(node, TALE)
    assert len(set(voices) - {""}) == 2 and len(plan_host.perform_busy(node).voices) == 4

    def edit(heroes):
        roles = [name for name in heroes]
        return {"edit_pending": True, "perform": {"kind": "tale", "segments": [f"Реплика {n}." for n in roles],
                                                  "cast": [{"name": n} for n in heroes], "role_per_segment": roles}}

    busy = plan_host.perform_busy(node)
    refused = validate(parse(edit(["Лиса", "Волк", "Сова", "Кот"])), ALL_TOOLS, busy=busy)
    assert isinstance(refused, Invalid) and refused.reason == "bad_cast"
    assert "свободных голосов для них 2" in refused.violations[0].detail
    accepted = _judge(node, edit(["Лиса", "Волк", "Сова"]))
    fox, wolf, owl = accepted.perform.voice_per_segment
    assert fox == voices[2] and wolf and owl and len({fox, wolf, owl}) == 3 and not {wolf, owl} & set(voices)
