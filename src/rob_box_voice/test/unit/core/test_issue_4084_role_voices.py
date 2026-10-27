"""#4084: голоса ролей выступления решает код - на настоящем пути хоста.

Живой случай 27.10 (dev-0d94634): «Робот, спой песню про енота» - куплеты через один голосом ``Russian_ReliableMan`` и
``Russian_HandsomeChildhoodFriend``; анекдот - пять голосов на трёх героев. Голос каждого сегмента выбирала модель.

Теперь: каталог ролей (``tts_voice_registry.role_voices_for``: русские голоса звучащего провайдера без рассказчика и
без голосов каста марафона ``CAST_VOICES``) с полом голоса -> ``plan_host.perform_busy`` -> валидатор команды
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
from rob_box_dialog.command import Command, parse, validate

from rob_box_voice.core import plan_host
from rob_box_voice.core.perform_host import PerformHost
from rob_box_voice.tts_voice_registry import (
    CAST_VOICES,
    PROVIDER_VOICES,
    VOICE_METADATA,
    default_voice_for,
    gender_of,
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


def test_cast_voices_never_voice_a_role_of_any_provider():
    for provider in PROVIDER_VOICES:
        roles = set(role_voices_for(provider, default_voice_for(provider)))
        assert not {f"{provider}:{v}" for v in roles} & CAST_VOICES, provider


def test_cast_voices_are_in_the_catalogue_with_metadata():
    for key in CAST_VOICES:
        provider, voice = key.split(":", 1)
        assert voice in PROVIDER_VOICES[provider] and key in VOICE_METADATA, key


def test_cast_voices_are_the_people_voices_of_the_e2e_harness():
    """Источник каста - ``map_tts_voice`` харнесса: голос человека там с просодией ``id:pitch:speed`` (#3931)."""
    text = E2E_LIB.read_text(encoding="utf-8")
    arm_start = text.index("        minimax)", text.index("map_tts_voice() {"))
    arm = text[arm_start : text.index("            ;;", arm_start)]
    people = {
        spec.split(":")[0] for spec in re.findall(r"printf '([^'%][^':]*:[^']*)'", arm)
    }
    assert people and {f"minimax:{v}" for v in people} == CAST_VOICES, sorted(people)


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
