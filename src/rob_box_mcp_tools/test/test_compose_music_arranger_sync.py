"""Третья граница: ``ComposeMusicTool.execute()`` ↔ ``arranger.spec_from_flat``.

``test_tool_catalog_sync.py`` уже сверяет две границы контракта
``compose_music``: «схема каталога ↔ ``execute()``» (обе стороны знают
друг о друге по построению — ``entry.signature`` берётся AST-разбором
самого ``execute``). ``test_skill_prompt_contract.py``
(``rob_box_voice``) сверяет ещё одну — «инструмент упомянут в тексте
скилла» — но только по именам инструментов, до параметров не
опускается.

Ни один из них не видит третью границу: «``execute()`` ↔
``spec_from_flat``». Именно на ней стоят ``counter_synth`` и
``theme_octaves`` из issue #2463 — оба параметра существовали в
``spec_from_flat`` (реальная звуковая фича — контрмелодия и удвоение
октав темы), но не были ни в схеме, ни в ``execute()``, ни в
``composer.txt``. Оба уже проверенных теста молчали: у них симметрично
согласованные концы (обе стороны их не знают), и расхождения на СВОЕЙ
границе они не видят.

Этот тест — тот самый гипотетический тест из issue #2464 («Предлагаемая
проверка»), выбран полный вариант (а не «Альтернатива попроще» —
замена ручного кортежа в ``test_known_melody_library_1810.py`` без
похода в ``spec_from_flat``): альтернатива устраняет хрупкость
ручного списка, но НЕ ловит класс #2463, потому что пропасть на уровень
ниже её сравнения. Полная защита требует именно сверки со
``spec_from_flat`` — issue прямо говорит: «третьей границы не избежать,
если цель — именно этот класс регрессии». Механика взята из issue
дословно: два ``inspect.signature()`` вызова, разность множеств, никакого
AST и никакого ROS2 — дешевле в поддержке, чем ``test_tool_catalog_sync.py``.

См. https://github.com/krikz/rob_box_project/issues/2464.
"""

from __future__ import annotations

import inspect
import sys
from unittest.mock import MagicMock

import pytest

# ``rob_box_mcp_tools.tools.music`` импортирует ``rclpy`` на уровне модуля
# (через базовый класс) — мокаем ROS2 так же, как это уже делает
# test_tools/test_music.py:20-35, чтобы не тянуть рантайм.
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

from rob_box_mcp_tools.core.arranger import spec_from_flat  # noqa: E402
from rob_box_mcp_tools.tools.music import ComposeMusicTool  # noqa: E402

#: Параметры ``spec_from_flat``, которые ``ComposeMusicTool.execute()``
#: заполняет САМ (из RTTTL-резолвинга по ``name=``), а не получает от
#: модели напрямую — и поэтому легитимно отсутствуют среди аргументов
#: ``execute()``. Каждый — с обоснованием, почему это не дыра:
_INTERNALLY_DERIVED: frozenset[str] = frozenset(
    {
        # Аккомпанемент и рисунки ударных, выведенные из мелодии через
        # core.harmonize.Harmonization — модель отдаёт только name=/
        # variants=, конкретные ноты аккомпанемента ей не нужны и были
        # бы избыточны (arranger.py:1353-1379, music.py:2521-2526,
        # 2561).
        "harmony",
        # Абсолютный MIDI темы из RTTTL-библиотеки для точного
        # воспроизведения — тоже выводится из name=/variants=, наружу
        # уходит только в лог и для обратной совместимости
        # (music.py:2521-2526, 2577).
        "lead_midi",
    }
)

#: Параметры ``spec_from_flat``, которые УЖЕ есть в ядре, но до модели
#: доходят следующим PR по плану миграции (AF/ADR-0013: мелкие PR). Это не
#: «выводится сам», а временная, названная дыра со сроком: каждая запись
#: указывает PR, который её закрывает, и тот PR обязан удалить запись
#: (иначе ``test_pending_exposure_is_not_yet_model_facing`` упадёт).
_PENDING_EXPOSURE: dict[str, str] = {
    # ADR-0132 PR-3: ручки сборки (counter / theme_octaves / levels) —
    # arranger.ArrangeOptions в ядре. Отдельными параметрами compose_music
    # (вариант A ADR-0132) их выводит PR-4.
    "options": "ADR-0132 PR-4",
}


def test_every_spec_from_flat_param_reaches_the_model_or_is_derived() -> None:
    """Параметр ``spec_from_flat``, влияющий на звук, не может быть немым.

    У параметра ровно два легитимных пути наружу: либо модель задаёт его
    сама через ``execute()`` (и, соответственно, через схему —
    ``test_tool_catalog_sync.py`` уже гарантирует, что всё, что принимает
    ``execute()``, объявлено в схеме), либо он выводится внутри
    ``execute()`` автоматически и явно перечислен в
    ``_INTERNALLY_DERIVED`` с обоснованием. Третьего не дано: параметр,
    который есть в ``spec_from_flat``, но не проходит ни по одному из
    путей, — мёртвая фича, которую модель не может включить, и никто
    этого не заметит (ровно класс регрессии #2463: ``counter_synth`` и
    ``theme_octaves`` появились в ``spec_from_flat``, но не были выведены
    ни в ``execute()``, ни в схему, ни в промпт).
    """
    spec_params = set(inspect.signature(spec_from_flat).parameters) - {"self"}
    exec_params = set(inspect.signature(ComposeMusicTool.execute).parameters) - {"self"}

    orphaned = spec_params - _INTERNALLY_DERIVED - set(_PENDING_EXPOSURE) - exec_params
    assert not orphaned, (
        f"spec_from_flat принимает {sorted(orphaned)}, но ComposeMusicTool.execute() "
        f"их не запрашивает у модели и не выводит сам (не в _INTERNALLY_DERIVED) — "
        f"фича мертва: модель не может её включить, и ни один существующий тест "
        f"этого не ловит (класс регрессии #2463, issue #2464)"
    )


def test_internally_derived_params_are_still_accepted_by_spec_from_flat() -> None:
    """Обратная сторона: список исключений не должен протухнуть сам.

    Если ``spec_from_flat`` когда-нибудь перестанет принимать ``harmony``
    или ``lead_midi`` (переименование, удаление), запись в
    ``_INTERNALLY_DERIVED`` осталась бы мёртвым обоснованием несуществующего
    параметра — эта проверка не даёт списку исключений отстать от кода.
    """
    spec_params = set(inspect.signature(spec_from_flat).parameters) - {"self"}
    stale = _INTERNALLY_DERIVED - spec_params
    assert not stale, (
        f"_INTERNALLY_DERIVED называет {sorted(stale)}, но spec_from_flat "
        f"больше не принимает такие параметры — запись устарела, обоснование "
        f"нужно удалить или обновить"
    )


def test_internally_derived_params_are_not_also_model_facing() -> None:
    """``_INTERNALLY_DERIVED`` — это ИЛИ модель, ИЛИ вывод, не оба сразу.

    Если параметр из списка исключений вдруг тоже появится среди
    аргументов ``execute()`` (например, кто-то решил дать модели прямой
    доступ к ``harmony``), обоснование «выводится сам» перестаёт быть
    полным — стоит либо убрать параметр из списка, либо решить, откуда
    в реальности берётся значение при одновременной подаче с обеих
    сторон.
    """
    exec_params = set(inspect.signature(ComposeMusicTool.execute).parameters) - {"self"}
    double_facing = _INTERNALLY_DERIVED & exec_params
    assert not double_facing, (
        f"{sorted(double_facing)} числятся в _INTERNALLY_DERIVED как выводимые "
        f"автоматически, но ComposeMusicTool.execute() их тоже принимает от "
        f"модели напрямую — обоснование в _INTERNALLY_DERIVED больше не точно"
    )


def test_pending_exposure_is_not_yet_model_facing() -> None:
    """``_PENDING_EXPOSURE`` не протухает: запись снимается вместе с дырой.

    Как только ``execute()`` начнёт принимать параметр напрямую (PR-4
    ADR-0132 для ``options``), запись обязана уйти из списка; и наоборот —
    параметр, которого ``spec_from_flat`` больше не принимает, в списке
    держать нельзя.
    """
    spec_params = set(inspect.signature(spec_from_flat).parameters) - {"self"}
    exec_params = set(inspect.signature(ComposeMusicTool.execute).parameters) - {"self"}
    pending = set(_PENDING_EXPOSURE)
    assert not pending - spec_params, f"устарели: {sorted(pending - spec_params)}"
    assert not pending & exec_params, (
        f"{sorted(pending & exec_params)} уже доходят до модели — убери из _PENDING_EXPOSURE"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
