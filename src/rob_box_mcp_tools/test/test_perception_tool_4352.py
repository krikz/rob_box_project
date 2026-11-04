"""Issue #4352: get_perception_context не выдаёт старый снимок за «сейчас»."""

import pytest

from ._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools.perception import CONTEXT_STALE_S, GetPerceptionContextTool
_ros_stubs = _ros.fixture()

pytestmark = pytest.mark.unit

_LIVE = {
    'vision_context': 'в кадре: лицо «Литвин» ~1.0 м; метки AprilTag: 21',
    'faces': [{'name': 'Литвин', 'distance_m': 1.0, 'age_s': 0.4}],
    'apriltag_ids': [21],
    'is_moving': False,
}


class _Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def _tool():
    clock = _Clock()
    return GetPerceptionContextTool(node=None, clock=clock), clock


def test_fresh_context_is_returned_as_is():
    tool, _ = _tool()
    tool.log_info = lambda *_a, **_k: None
    tool.update_context(dict(_LIVE))
    result = tool.execute()
    assert result.success
    assert result.data['faces'][0]['name'] == 'Литвин'
    assert result.data['apriltag_ids'] == [21]


def test_stale_context_says_no_data_instead_of_old_faces():
    tool, clock = _tool()
    tool.log_info = lambda *_a, **_k: None
    tool.update_context(dict(_LIVE))
    clock.t += CONTEXT_STALE_S + 30
    result = tool.execute()
    assert result.success
    assert result.data['faces'] == []
    assert result.data['apriltag_ids'] == []
    assert result.data['vision_context'].startswith('камера: нет данных')
    assert result.data['perception_age_s'] > CONTEXT_STALE_S


def test_no_context_at_all_is_failure():
    tool, _ = _tool()
    tool.log_info = lambda *_a, **_k: None
    assert not tool.execute().success
