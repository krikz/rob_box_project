"""Issue #4352 (доработка): классы детектора в строке «в кадре» - по-русски."""

import pytest

from rob_box_mcp_tools.perception_projection import COCO_RU, describe_view, ru_class

pytestmark = pytest.mark.unit


def _obj(cls):
    return {'label': cls, 'kind': 'object', 'name': None, 'confidence': 0.6,
            'distance_m': None, 'age_s': 0.5}


def test_suitcase_in_russian():
    assert describe_view([_obj('suitcase')], [21]) == 'в кадре: чемодан; метки AprilTag: 21'


@pytest.mark.parametrize('cls,ru', [
    ('handbag', 'сумка'), ('backpack', 'рюкзак'), ('chair', 'стул'), ('bottle', 'бутылка'),
    ('cup', 'чашка'), ('laptop', 'ноутбук'), ('cell phone', 'телефон'),
])
def test_common_classes(cls, ru):
    assert ru_class(cls) == ru


def test_unknown_class_stays_english():
    assert ru_class('flux capacitor') == 'flux capacitor'
    assert describe_view([_obj('flux capacitor')], []) == 'в кадре: flux capacitor'


def test_table_covers_coco_except_person():
    try:
        from rob_box_perception.vision_hailo_loader import COCO_CLASS_NAMES
    except ImportError:
        pytest.skip('rob_box_perception не в PYTHONPATH')
    missing = [c for c in COCO_CLASS_NAMES if c != 'person' and c not in COCO_RU]
    assert not missing
