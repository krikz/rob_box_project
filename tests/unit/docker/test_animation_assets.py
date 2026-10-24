"""Guards of the procedural LED face (ADR-0150 §5.5, PR-3a #3689).

Кадры лица больше не запекаются в PNG: ``face_node`` рисует сцены
``rob_box_animations.face`` в реальном времени. Этот тест держит то, что
композитор ros2leds и драйвер примут без молчаливых потерь, и то, что
лицо не разъедется после правки генератора:

* каждая сцена рисует все пять логических групп нужного размера, уровни
  арта 0..25 (драйвер на brightness 25 показывает ровно их);
* детерминизм: кадр t=1.0 и весь проход сцены при seed по умолчанию —
  байт в байт (дайджесты ниже; переход из лаборатории проверен против
  старого офлайн-рендера, raw — в PR #3689);
* бюджет тока: светящаяся часть любой сцены при brightness 25 ≤ 1 А;
* раскладка панель лица -> группа и тест-кадр сторон (A27);
* словарь v1 (``KNOWN_ANIMATIONS``, тул ``play_animation``) — всё рисуется.
"""

from __future__ import annotations

import hashlib
import importlib.util
import sys
from pathlib import Path

import pytest
import yaml

np = pytest.importorskip("numpy")

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "src" / "rob_box_animations"))

from rob_box_animations.face import states as S  # noqa: E402
from rob_box_animations.face.face import (  # noqa: E402
    GROUP_OF, GROUP_SIZE, IDLE_MA, SCENE_LIT_LIMIT_MA, frame_ma, to_wire)
from rob_box_animations.face.scene import FaceContext, Scene, clip  # noqa: E402
from rob_box_animations.face.sides import GLYPHS, MARKS, sides_frame  # noqa: E402

COMPOSITOR = REPO_ROOT / "docker" / "vision" / "config" / "led_matrix" / "led_matrix_compositor.yaml"
ORDER = ("eye_l", "eye_r", "mouth", "rear_l", "rear_r")

GOLDEN = {  # name: (sha1 frame t=1.0 seed default, sha1 whole pass)
    'idle_stare': ('4ca0b41f53e82dae6460f8db174f093043dad278', 'f0e125fa9fd2607a22d7f1805e6ae11468d873e4'),
    'idle_bored': ('9349c7be35a7a5d27e372f1cee735d3de576b49d', '2f240afb8e42e3625a335196bb1b76929689aa53'),
    'listening': ('dab1b8f50835e289351c78af3f87dcad88ef28f7', 'ccceea2f0a7d74fa6bbfb29190f3d7f11fc7260b'),
    'thinking_teeth': ('8b9134f4b62e0b20424a04bc8bf423ac65a7d666', 'f7c3395a5f934f298e06cda4d8ab6684fd30fcea'),
    'thinking_orbit': ('9f6461c49d7616716383e2ba8dde09efc55a195c', '5ea137968f65779db812c64c7c6a45fce49679dd'),
    'talk_jaw': ('079e6bd67467c51064516032040dce57c4cecc33', 'f0c13f0abb11d95354f64353fbe22543d8f077a5'),
    'talk_rubber': ('476bc4cca315688d6102ac0196fe622bc028e9b1', 'd5e6956802a74d06dbec6bb148cfe6e38f1eee4c'),
    'talk_voicebox': ('80fa360fd73a99696013d2bb702485f8d1773559', 'ef26cd265dc08ae006991a7338dedc1d33a11c19'),
    'talk_happy': ('106a2cede8027aca0af9276205644ccc0f146f2b', '6616dad7e5e68d2300751679574edab66ba5eb73'),
    'happy': ('103053768b23c2db53ad08f3fbabdecf266157b1', '3c40700f82c58a5cf0fee1b173ea1455232bc436'),
    'wink': ('30a6b40a4eaa88daa9c22425587cea66958e040e', 'c24ea806ed3050a0bf10412596082cdab2bcff10'),
    'sad': ('4a04b077a93b5ebc9942d08c0239f382d6747e7b', '20a556ce761fd92b5c6d5cd0b7fa2d5d68947e46'),
    'angry': ('697c52b28e6bd01dab22768ce3929b4220d3a8cd', '825fdda540f8607645f13de145ea982615606770'),
    'surprised': ('9e638315df9e0b9e39c80b3f76140b7c5fa3500b', '3ff702a3b9a14437b8ef7fd03f4822567a0f06e8'),
    'victory': ('30d17f82dbf6adc0714faef6ecf9d5ec1a4f242a', '080fba8fba1f6023595a2b11cadeec97e3e80547'),
    'music_groove': ('3aa32fa4296689e454269e38727aa8c5788916b8', '672ca39793b73ebb94a5de72a28111d7267ecf74'),
    'music_shades': ('a65c62f71bca77e1a85af49787a71c8b73551531', '8066c3ed801990dc4c18bca1a1c543721bcabfbf'),
    'music_vibe': ('c83b0a202ac9d5a50804c8d06a5215b6fb24198c', '5a6ce47ac680e992b1439123c619d85303f71fb0'),
    'police_lights': ('3e6046b5a889747af6510364fbd4cd4b6ca0a960', '22b9c74076c308773350f6a6a6180c85d1646d73'),
    'fire_truck': ('4b36411d0a5cd5a01ef1f9868a89ad28c5ff4bad', 'd828c5f1d3f02987b4169100d57b1eba2bd31d92'),
    'ambulance': ('d847d21f25104f5c2f99728c47a94fd20d2cf03b', '1c441742bb1936ea3edacd63fef1c248dbf104f3'),
    'road_service': ('f1603814ef4b64ea5f33c76eacfe492f63c718bb', '661f311ab1c4072dcf5f5fa37fe24e949c12e2c0'),
    'turn_left': ('1fa7178ac33fff144683089b321ae40e79fc07cc', '2738d36054942f7954383e49b0fa10c516e8d76f'),
    'turn_right': ('015636addef9ec9a119f2914103528509b152e17', '49f7a0a9245e46fc2f59d61572bcc7da86b28810'),
    'brake': ('7165a09f211e01449dfe90c917584f5db0e049d9', '4b0858ef73c41fa02b79fa85c8a024f86d71668b'),
    'accelerating': ('8a68299a65fc426cf19122a9e9f95c809798c81d', '61350f2d32db1f25bf46e6afed539371eaa943f3'),
    'taped_mouth': ('a6a030cac301dd4c0d117182ae9aba2cbd0cd19e', 'df92f7f404a787ba343dc5367a469510d13818d9'),
    'sleep': ('10fc1d17e5bc4f5473a1b4f1544db759054b8ab4', '5cb48db335b5296810d29fefd478f328f1e5aa60'),
    'wakeup': ('ce8359cea53adf2b0c61b461314ba83405659537', 'a01f95f3488203c353ad6fd04513510d641ccdb4'),
    'error': ('08f16a95ae838e418eeafcce366436438f60dfc6', '0501b21857de645ebf523fd67802ca1a7e58926a'),
    'low_battery': ('28eebea26d4e7c05e135f2bd3ac0d3e55cebec4b', 'b416e079017d1d98e6ec7d9d844c58bc7ea2f3f6'),
    'charging': ('c8d8ac3d4afe7e625555f25a70431a6d485fa768', '572c5fd992f37b46171457d206c6ead960982e43'),
    'new_year_eve': ('8e7b3786c7496e317493622a7f444f01b992f990', '39af475db2c30bae45106880ca6c1f4fc82187a2'),
}


def wire_bytes(f) -> bytes:
    p = f.panels()
    return b"".join(to_wire(p[k]).tobytes() for k in ORDER)


def test_every_scene_has_a_golden():
    assert set(GOLDEN) == set(S.SCENES)


@pytest.mark.parametrize("name", sorted(S.SCENES))
def test_scene_draws_every_group_in_size_and_levels(name):
    scene = Scene(S.SCENES[name])
    for i in range(0, int(S.SCENES[name].period * 25), 5):
        f = scene.frame(i / 25, FaceContext(mouth_level=0.7, gaze_deg=-30.0, bpm=128.0, beat_phase=i / 10))
        for panel, arr in f.panels().items():
            w, h = GROUP_SIZE[GROUP_OF[panel]]
            assert arr.shape == (h, w, 3), (name, panel, arr.shape)
            assert float(arr.min()) >= 0.0, (name, panel)
            # на ленте — уровни 0..25 (арт пожарной во рту доходит до 25.3, to_wire режет)
            assert int((to_wire(arr).astype(int) * 25 // 255).max()) <= 25, (name, panel)


@pytest.mark.parametrize("name", sorted(S.SCENES))
def test_scene_is_deterministic_byte_for_byte(name):
    t1, whole = GOLDEN[name]
    scene = Scene(S.SCENES[name])
    for i in range(26):
        f = scene.frame(i / 25)
    assert hashlib.sha1(wire_bytes(f)).hexdigest() == t1
    assert hashlib.sha1(b"".join(wire_bytes(f) for f in clip(S.SCENES[name]))).hexdigest() == whole


@pytest.mark.parametrize("name", sorted(S.SCENES))
def test_scene_lit_current_stays_under_one_amp(name):
    scene = Scene(S.SCENES[name])
    lit = max(frame_ma(scene.frame(i / 25)) - IDLE_MA for i in range(int(S.SCENES[name].period * 25)))
    assert lit <= SCENE_LIT_LIMIT_MA, f"{name}: {lit:.0f} mA lit at brightness 25"


def test_wire_level_survives_driver_brightness():
    levels = np.arange(26, dtype=float)
    assert ((to_wire(levels).astype(int) * 25 // 255) == levels).all()


def test_layout_table_is_pinned_and_matches_the_compositor():
    assert GROUP_OF == {
        "eye_l": "wheel_front_right",   # левая фара робота у зрителя справа
        "eye_r": "wheel_front_left",
        "mouth": "main_display",
        "rear_l": "wheel_rear_left",
        "rear_r": "wheel_rear_right",
    }
    cfg = yaml.safe_load(COMPOSITOR.read_text(encoding="utf-8"))["led_matrix_compositor"]["ros__parameters"]
    panels = cfg["physical_panels"]
    for g in cfg["logical_groups"]:
        cols, rows = g["arrangement"]
        idx = g["physical_indices"]
        w, h = panels[idx[0]]["width"] * cols, panels[idx[0]]["height"] * rows
        assert GROUP_SIZE[g["name"]] == (w, h), g["name"]


def test_side_frame_marks_every_board():
    f = sides_frame()
    panel_of = {group: panel for panel, group in GROUP_OF.items()}
    for group, (side, num) in MARKS.items():
        arr = getattr(f, panel_of[group])
        lit = arr.sum(axis=2) > 0
        want = np.zeros((8, 8), bool)
        for x0, glyph in ((0, GLYPHS[side]), (4, GLYPHS[num])):
            for y, row in enumerate(glyph):
                for x, ch in enumerate(row):
                    want[1 + y, x0 + x] = ch == "#"
        assert (lit == want).all(), group
    # стрелка во рту смотрит вправо (к левой фаре робота): остриё у правого края
    mouth = f.mouth.sum(axis=2) > 0
    assert mouth[2, 22] and not mouth[2, 24] and mouth[0, 19] and not mouth[0, 0]


def _known_animations():
    path = REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "animations.py"
    spec = importlib.util.spec_from_file_location("_animations_vocab", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.KNOWN_ANIMATIONS


def test_v1_vocabulary_resolves_to_scenes():
    """Каждое имя, которое тул ``play_animation`` предлагает LLM, лицо рисует."""
    missing = [n for n in _known_animations() if S.resolve(n) is None]
    assert not missing, missing
    assert S.resolve("talking", mouth="voicebox") == "talk_voicebox"
    assert S.resolve("thinking", thinking="thinking_orbit") == "thinking_orbit"
    assert S.resolve("no_such_scene") is None
