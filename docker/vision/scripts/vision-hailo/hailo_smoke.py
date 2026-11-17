#!/usr/bin/env python3
"""hailo_smoke.py — smoke-test для AI HAT+ (ADR-0089 Phase 1).

Запуск на Vision Pi после установки AI HAT+:
    python3 /scripts/hailo_smoke.py

Что проверяем:
  1. hailortcli доступен в PATH.
  2. hailortcli scan возвращает устройство.
  3. HailoRT Python binding импортируется (опционально).

Exit code:
  0 — HAT виден, smoke пройден.
  1 — HAT не виден (драйвер / питание / PCIe).
  2 — hailortcli не установлен (контейнер сломан).
  3 — Python binding недоступен (warning, не fatal).
"""

import shutil
import subprocess
import sys


def main() -> int:
    print('=== AI HAT+ smoke test (ADR-0089 Phase 1) ===')

    # 1. hailortcli в PATH.
    hailortcli = shutil.which('hailortcli')
    if hailortcli is None:
        print('ERROR: hailortcli не найден в PATH.', file=sys.stderr)
        print('  Установите HailoRT: https://github.com/hailo-ai/hailort', file=sys.stderr)
        return 2

    print(f'[1/3] hailortcli: {hailortcli}')

    # 2. hailortcli scan.
    try:
        result = subprocess.run(
            ['hailortcli', 'scan'],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except subprocess.TimeoutExpired:
        print('ERROR: hailortcli scan timeout (>10s).', file=sys.stderr)
        return 1

    print('[2/3] hailortcli scan output:')
    print('---')
    print(result.stdout.strip())
    if result.stderr.strip():
        print('(stderr):', result.stderr.strip())
    print('---')

    if result.returncode != 0:
        print(f'ERROR: hailortcli scan exit={result.returncode}.', file=sys.stderr)
        return 1
    if 'hailo' not in result.stdout.lower():
        print('ERROR: hailortcli output не содержит "hailo" device.', file=sys.stderr)
        return 1

    # 3. Python binding (warning).
    try:
        import hailo_platform  # noqa: F401
        print('[3/3] Python binding: OK (hailo_platform importable)')
    except ImportError:
        print('[3/3] WARN: Python binding (hailo_platform) недоступен — '
              'Phase 1.5 inference отложен.')

    print()
    print('=== Smoke PASSED. Phase 1 acceptance met. ===')
    return 0


if __name__ == '__main__':
    sys.exit(main())
