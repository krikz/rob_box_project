"""Поставка готовых клипов Химеры (ADR-0175 шаг 1.1, #5152).

Герметично: неизвестный вызов отказывает до хоста, битый хеш или пропавший файл не объявляются, подтверждение
строится из стадии ``finished``, а не из ``started``, стоп во время клипа обрывает именно его. Синтеза нет.
"""

from __future__ import annotations

import json
import shutil
import wave
from pathlib import Path

from rob_box_dialog import phrases

from ._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools import chimera_package as package
    from rob_box_mcp_tools.tools import chimera_decide as decide
    from rob_box_mcp_tools.tools import chimera_delivery as delivery
    from rob_box_mcp_tools.tools.chimera import ChimeraHostPort, ChimeraPerformTool
_ros.fixture()

BIRD = "bird_chirps_16k_notail.wav"
BIRD_SHA = "03fc28940f71091bc60e89b21cca846b5173bac521a15f3166f3c902c369ab33"
SEAL = "seal_zap_16k_notail.wav"
SEAL_SHA = "fac1baf26afa3134d2abc8eaddc2cb794c3218c110bac96ed78b918b7ae0c2c2"


def _repo_clips() -> Path:
    """Поставка лежит в чекауте. Идём вверх до profile.json, а не считаем parents[N]."""
    here = Path(__file__).resolve()
    for ancestor in Path(__file__).resolve().parents:
        profile = ancestor / "sound_pack" / "chimera" / "profile.json"
        if profile.is_file():
            return profile.parent
    raise FileNotFoundError(f"sound_pack/chimera/profile.json не найден от {here}")


REPO_CLIPS = _repo_clips()


class _Node:
    """Нода-заглушка: издатель запоминает, что просили сыграть, и ничего не шлёт в ROS."""

    def __init__(self):
        self.published: list = []
        self.aborted: list = []

    def get_logger(self):
        return self

    def info(self, *_a, **_k):
        return None

    def create_publisher(self, *_a, **_k):
        return self

    def publish(self, msg):
        self.published.append(json.loads(msg.data))

    def abort_chimera(self, clip: str) -> bool:
        self.aborted.append(clip)
        return True


def _profile(tmp_path: Path) -> decide.ChimeraProfile:
    shutil.copytree(REPO_CLIPS, tmp_path / "chimera")
    return delivery.load_delivery(str(tmp_path / "chimera"))


def _tool(host, profile) -> ChimeraPerformTool:
    return ChimeraPerformTool(_Node(), host=host, profile=profile)


class _Host:
    def __init__(self, stage):
        self.stage = stage
        self.plans = []

    def perform(self, plan):
        self.plans.append(plan)
        return "req-1", self.stage


def test_delivery_advertises_only_the_five_verified_calls(tmp_path):
    profile = _profile(tmp_path)
    calls = delivery.advertised_calls(profile)
    assert profile.stub is False and profile.actions == ("call",)
    assert calls == (
        ("аллигатор", "рык", "alligator_growl_16k_notail.wav"),
        ("птица", "жужжание", "bird_buzz_16k_notail.wav"),
        ("птица", "щебет", BIRD),
        ("слон", "писк", "elephant_squeak_16k_notail.wav"),
        ("тюлень", "писк", SEAL),
    )
    schema = {p.name: p for p in _tool(_Host(None), profile).parameters}
    assert schema["action"].enum == ["call"]  # речь, пение и игра пакет не обещает
    assert "speak" not in schema["action"].enum
    assert set(schema["call"].enum) == {"щебет", "писк", "рык", "жужжание"}
    assert set(schema["creature"].enum) == {"птица", "тюлень", "слон", "аллигатор"}


def test_unknown_call_is_a_clear_refusal_and_never_asks_the_host(tmp_path):
    profile = _profile(tmp_path)
    host = _Host({"stage": "finished"})
    result = _tool(host, profile).execute(action="call", creature="птица", call="лай")
    assert host.plans == []
    assert result.data["status"] == "rejected" and result.data["reason"] == "unsupported_pairing"
    assert result.data["sang"] is False and result.success is False
    assert result.message == phrases.text("chimera.rejected.unsupported_pairing")
    assert "Спел" not in (result.message or "")


def test_missing_file_is_not_advertised_and_refuses(tmp_path):
    root = tmp_path / "chimera"
    shutil.copytree(REPO_CLIPS, root)
    (root / BIRD).unlink()
    profile = delivery.load_delivery(str(root))
    names = {item[2] for item in delivery.advertised_calls(profile)}
    assert BIRD not in names and len(names) == 4
    result = _tool(_Host({"stage": "finished"}), profile).execute(action="call", creature="птица", call="щебет")
    assert result.data["reason"] == "unsupported_pairing" and result.data["sang"] is False
    assert result.message == phrases.text("chimera.rejected.unsupported_pairing")


def test_broken_hash_drops_the_call(tmp_path):
    """Мутация: байт файла испорчен — вызов пропадает из профиля, тест красный, если сверка хеша снята."""
    root = tmp_path / "chimera"
    shutil.copytree(REPO_CLIPS, root)
    target = root / SEAL
    target.write_bytes(target.read_bytes()[:-1] + b"\x00")
    profile = delivery.load_delivery(str(root))
    assert SEAL not in {item[2] for item in delivery.advertised_calls(profile)}
    result = _tool(_Host(None), profile).execute(action="call", creature="тюлень", call="писк")
    assert result.data["reason"] == "unsupported_pairing" and result.success is False


def test_confirmation_waits_for_finished_and_names_the_file(tmp_path):
    profile = _profile(tmp_path)
    started = _tool(_Host({"stage": "started"}), profile).execute(action="call", creature="птица", call="щебет")
    assert started.data["status"] == "pending" and started.data["sang"] is False
    assert started.message is None  # «играет» — ещё не «сказала»
    done = _tool(_Host({"stage": "finished"}), profile).execute(action="call", creature="птица", call="щебет")
    assert done.success is True and done.data["clip"] == BIRD and done.data["sang"] is False
    assert done.message == phrases.text("chimera.done", title=f" «{BIRD}»", action="call")
    assert done.message != phrases.text("chimera.sang", title=f" «{BIRD}»", action="call")


def test_port_publishes_the_real_filename_and_stop_aborts_that_clip(tmp_path):
    profile = _profile(tmp_path)
    plan = decide.decide("call", profile, creature="птица", call_name="щебет")
    assert plan.plays and plan.clip == BIRD and plan.stub is False
    node = _Node()
    port = ChimeraHostPort(node)

    def ask(clip, trigger, event, wait_s):
        node.published.append({"sound": clip, "source": "chimera"})
        return "req-9", {"stage": "started"}

    port._ask_sound = ask  # noqa: SLF001 — проверяем, что в звук уходит имя файла, не clip:
    request_id, stage = port.perform(plan)
    assert request_id == "req-9" and stage["stage"] == "started"
    assert node.published == [{"sound": BIRD, "source": "chimera"}]
    assert port.stop("playback") is True and node.aborted == [BIRD]
    assert port.stop("playback") is False  # второй стоп уже нечего обрывать — «остановил» было бы враньём


class _SilentNode:
    """Нода без глушителя: ``abort_chimera`` нет, и «остановил» сказать нельзя."""

    def get_logger(self):
        return self

    def info(self, *_a, **_k):
        return None


def test_stop_without_a_silencer_does_not_claim_success(tmp_path):
    profile = _profile(tmp_path)
    plan = decide.decide("call", profile, creature="слон", call_name="писк")
    node = _SilentNode()
    port = ChimeraHostPort(node)
    port._ask_sound = lambda *a, **k: ("req", {"stage": "started"})  # noqa: SLF001
    port.perform(plan)
    assert port.stop("playback") is False


def test_package_pins_the_source_sha_and_the_measured_hashes():
    body = json.loads((REPO_CLIPS / delivery.PROFILE_NAME).read_text(encoding="utf-8"))
    assert body["source_sha"] == "caf270abf893bdc63f6342059d4468f754edf5c6"
    assert body["source_issue"] == 5026 and body["version"] == "0.1.0"
    rows = {row["file"]: row["sha256"] for row in body["calls"]}
    assert rows[BIRD] == BIRD_SHA and rows[SEAL] == SEAL_SHA
    for filename, expect in rows.items():
        path = REPO_CLIPS / filename
        import hashlib

        assert hashlib.sha256(path.read_bytes()).hexdigest() == expect
        with wave.open(str(path)) as handle:
            assert handle.getnchannels() == 1 and handle.getframerate() == 16000 and handle.getsampwidth() == 2


def test_missing_package_falls_back_to_the_stub(tmp_path):
    profile = delivery.load_delivery(str(tmp_path / "нет-такого"))
    assert profile.stub is True and delivery.advertised_calls(profile) == ()
    schema = {p.name: p for p in ChimeraPerformTool(_Node(), host=_Host(None), profile=profile).parameters}
    assert schema["action"].enum == list(decide.stub_profile().actions)
    assert schema["call"].enum is None  # заглушка не объявляет вызовов: файла нет


def test_delivery_enums_come_from_the_same_package_reader(tmp_path):
    """Схема тула и сверка файла — один читатель. Нода зовёт его же (voice/test/unit/sound)."""
    root = tmp_path / "chimera"
    shutil.copytree(REPO_CLIPS, root)
    assert delivery.delivery_enums(str(root))[0] == ("call",)
    assert package.verified_clip(str(root), BIRD) is True
    assert package.PACKAGE_DIR == "/ws/sound_pack/chimera"


def test_finished_phrase_comes_from_the_outcome_not_from_model_text():
    """ADR-0148: фразу строит ``sang_phrase`` из исхода. Текст модели сюда не входит — аргумента для него нет."""
    import inspect

    assert "model" not in inspect.signature(decide.sang_phrase).parameters
    plan = decide.ChimeraPlan("play", "call", route=decide.ROUTE_CLIP, clip=BIRD)
    assert decide.sang_phrase(decide.outcome_of(plan, {"stage": "started"}, "r"), phrases.text) is None
    said = decide.sang_phrase(decide.outcome_of(plan, {"stage": "finished"}, "r"), phrases.text)
    assert said == phrases.text("chimera.done", title=f" «{BIRD}»", action="call")
