"""Issue #2968 — search_samples находит FX белого списка pack 1 по тегу.

По умолчанию поиск ходит по ``0_foxdot_default`` и по имени файла — эту
часть покрывает test_sample_search.py (rob_box_voice). Здесь — то, что
добавляет ``SearchSamplesTool`` СВЕРХУ для пака 1 (issue #2968, наказ Шифу:
жанр → теги каталога → поиск, а не зашитое сопоставление «жанр → сэмпл»):

* найденный по имени файл, который есть в белом списке FX, получает
  рабочий ``play_code`` вместо сломанного ``spack=1`` (#2841: no-op в этой
  сборке Renardo);
* найденный по имени файл, которого в белом списке нет, получает честную
  пометку, что ``spack=`` его не сыграет;
* запрос, совпавший с ЖАНРОВЫМ ТЕГОМ каталога (не с именем файла),
  добавляет кандидатов в ``data['fx_by_tag']`` — это и есть путь
  «жанр → поиск», а не хардкод.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path
from unittest.mock import MagicMock

# Mock ROS 2 modules before importing anything from rob_box_mcp_tools — тот
# же приём, что test_music.py (tools/__init__.py тянет rclpy через другие
# инструменты пакета, которых на CI/локально нет).
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
    "std_msgs",
    "std_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "action_msgs",
    "action_msgs.srv",
    "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.music import SearchSamplesTool  # noqa: E402


def _touch(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("")


def _make_pack1_tree(root: Path) -> None:
    """Минимальное дерево пака 1 с ОДНИМ реальным файлом из белого списка
    (``u/upper/005_Snare_AltGunShot_Kaonaya.wav``, см. data/sample_fx.json)
    плюс не-белым файлом рядом."""
    (root / "0_foxdot_default").mkdir()
    pack_dir = root / "1_pitchglitch_samples"
    upper = pack_dir / "u" / "upper"
    upper.mkdir(parents=True)
    for i in range(5):
        _touch(upper / f"{i:03d}_Snare_Filler_{i}.wav")
    _touch(upper / "005_Snare_AltGunShot_Kaonaya.wav")


def test_filename_match_replaces_broken_spack_play_code(mock_node):
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        _make_pack1_tree(root)
        tool = SearchSamplesTool(mock_node, samples_path=str(root))

        result = tool.execute(query="AltGunShot", pack="1_pitchglitch_samples", case="upper")

    assert result.success
    results = result.data["results"]
    assert len(results) == 1
    hit = results[0]
    assert hit["fx_whitelisted"] is True
    assert hit["fx_name"] == "gunshot_1"
    # Флаг выключен в тестовом окружении -> честно об этом сказано, а не
    # выдан play_code, который на самом деле не сыграет.
    assert "spack=1" not in hit["play_code"]
    assert "ROB_BOX_PACK1_LOOPS" in hit["play_code"] or "compose_music" in hit["play_code"]


def test_non_whitelisted_filename_match_gets_honest_note(mock_node):
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        _make_pack1_tree(root)
        tool = SearchSamplesTool(mock_node, samples_path=str(root))

        result = tool.execute(query="Filler", pack="1_pitchglitch_samples", case="upper")

    assert result.success
    results = result.data["results"]
    assert results
    for hit in results:
        assert "note" in hit
        assert "spack" in hit["note"]
        assert "#2841" in hit["note"]


def test_genre_tag_query_surfaces_fx_by_tag_even_without_filename_match(mock_node):
    """Наказ Шифу к #2968: «gangsta» не встречается ни в одном имени файла
    пака 1, но должен найти gunshot_1/siren_1 через теги каталога."""
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        _make_pack1_tree(root)
        tool = SearchSamplesTool(mock_node, samples_path=str(root))

        result = tool.execute(query="gangsta", pack="1_pitchglitch_samples")

    assert result.success
    assert result.data.get("found", 0) == 0  # ничего по имени файла
    fx_by_tag = result.data.get("fx_by_tag")
    assert fx_by_tag
    names = {c["name"] for c in fx_by_tag}
    assert "gunshot_1" in names
    assert "siren_1" in names
    assert "ROB_BOX_PACK1_LOOPS" not in result.message  # честное сообщение, не мусор
    assert "gunshot_1" in result.message or "тегу" in result.message


def test_default_pack_search_is_not_annotated(mock_node):
    """Пак 0 не имеет отношения к белому списку FX — аннотаций быть не должно."""
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        pack_dir = root / "0_foxdot_default"
        _touch(pack_dir / "a" / "lower" / "kick1.wav")
        tool = SearchSamplesTool(mock_node, samples_path=str(root))

        result = tool.execute(query="kick", pack="0_foxdot_default")

    assert result.success
    assert "fx_by_tag" not in (result.data or {})
    for hit in result.data.get("results", []):
        assert "fx_name" not in hit
        assert "note" not in hit
