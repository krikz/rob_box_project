"""
test_audio_node_no_bare_except.py — Guard-тест: в audio_node.py не должно
оставаться голых `except` (без типа исключения).

История: issue #829 / FA-1 (tech-debt). В shutdown-обработчике
audio_node.py было три блока с голым `except` и пустым телом
(строки 325, 331, 337 на момент создания issue):

    try:
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
    except:  # noqa — пример старого кода из issue
        pass

Голый `except` ловит BaseException (KeyboardInterrupt, SystemExit),
маскирует реальные ошибки и не даёт диагностики. Фикс (PR #xxxx):
заменено на `except Exception as exc:` + `self.get_logger().warning(...)`
— shutdown остаётся безаварийным, но ошибки видны в логах.

Этот тест — страховка от регрессии: сканируем исходник audio_node.py
и убеждаемся, что голый `except` нигде не встречается.

Не импортируем rob_box_voice (нет ROS mocks для audio_node) — только
чтение исходника, как в test_audio_node_bytecode.py.
"""

from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
AUDIO_NODE_PY = REPO_ROOT / 'src' / 'rob_box_voice' / 'rob_box_voice' / 'audio_node.py'

# Голый except: `except:` с любым количеством пробелов между ключевым
# словом и двоеточием. Комментарий после двоеточия (`except:  # reason`)
# тоже считается голым — except должен иметь тип.
_BARE_EXCEPT_RE = re.compile(r'except\s*:')


def test_audio_node_source_exists() -> None:
    """Аудио-исходник должен существовать по ожидаемому пути."""
    assert AUDIO_NODE_PY.is_file(), f'audio_node.py not found at {AUDIO_NODE_PY}'


def test_audio_node_shutdown_has_no_bare_except() -> None:
    """Shutdown-обработчик не должен содержать голых `except`.

    Регрессия #829/FA-1: все три блока должны быть заменены
    на `except Exception as exc:` с логированием.
    """
    text = AUDIO_NODE_PY.read_text(encoding='utf-8')

    # Выделяем тело shutdown()
    m = re.search(
        r'def\s+shutdown\s*\([^)]*\)\s*:(.*?)(?=^    def\s|\Z)',
        text,
        re.MULTILINE | re.DOTALL,
    )
    assert m, 'could not find shutdown() body in audio_node.py'
    body = m.group(1)

    matches = list(_BARE_EXCEPT_RE.finditer(body))
    assert not matches, (
        'AudioNode.shutdown() содержит голый `except:` — '
        'issue #829/FA-1 требует `except Exception as exc:` с '
        'логированием. Найдено: '
        + ', '.join(
            repr(body[max(0, mt.start() - 30):mt.end() + 10]) for mt in matches
        )
    )


def test_audio_node_whole_file_has_no_bare_except() -> None:
    """Голый `except` не должен встречаться нигде в audio_node.py.

    Более сильный guard, чем проверка только shutdown(): защищает
    от регрессий во всём файле (issue #829/FA-1).
    """
    text = AUDIO_NODE_PY.read_text(encoding='utf-8')
    matches = list(_BARE_EXCEPT_RE.finditer(text))
    assert not matches, (
        'audio_node.py содержит голый `except:` — замените на '
        '`except Exception as exc:` с логированием (issue #829/FA-1). '
        'Найдено: '
        + ', '.join(
            repr(text[max(0, mt.start() - 40):mt.end() + 10]) for mt in matches
        )
    )
