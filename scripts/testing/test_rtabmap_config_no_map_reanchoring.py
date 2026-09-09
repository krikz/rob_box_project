from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RTABMAP_YAML = REPO_ROOT / "docker/main/config/rtabmap/rtabmap.yaml"
RTABMAP_INI = REPO_ROOT / "docker/main/config/rtabmap/rtabmap.ini"


def test_rtabmap_configs_disable_optimize_from_graph_end():
    yaml_text = RTABMAP_YAML.read_text(encoding="utf-8")
    ini_text = RTABMAP_INI.read_text(encoding="utf-8")

    assert 'RGBD/OptimizeFromGraphEnd: "false"' in yaml_text
    assert 'RGBD/OptimizeFromGraphEnd: "true"' not in yaml_text
    assert 'OptimizeFromGraphEnd=false' in ini_text
    assert 'OptimizeFromGraphEnd=true' not in ini_text