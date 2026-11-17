"""Параллельный прогон тяжёлых чистых вычислений тестов (#3504): ``pmap(fn, items)`` = ``[fn(x) for x in items]``.

Результат и порядок те же, что у последовательного цикла (функции чистые, от процесса не зависят), поэтому смысл
тестов не меняется. Если процессов нет (одно ядро, ``ROB_BOX_TEST_SERIAL=1``) или пул не поднялся, считаем
последовательно: тест не должен падать из-за окружения."""

from __future__ import annotations

import multiprocessing
import os
from concurrent.futures import ProcessPoolExecutor


def _workers(n_items: int) -> int:
    if os.environ.get("ROB_BOX_TEST_SERIAL") == "1":
        return 1
    return max(1, min(n_items, os.cpu_count() or 1, 8))


def pmap(fn, items, chunksize: int = 1) -> list:
    """``fn`` — функция верхнего уровня модуля (picklable); ``items`` — picklable."""
    items = list(items)
    workers = _workers(len(items))
    if workers == 1:
        return [fn(x) for x in items]
    methods = multiprocessing.get_all_start_methods()
    ctx = multiprocessing.get_context("fork" if "fork" in methods else "spawn")
    try:
        with ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as pool:
            return list(pool.map(fn, items, chunksize=chunksize))
    except (OSError, ImportError, PermissionError):  # пул не поднялся; ошибки самой fn пробрасываются как есть
        return [fn(x) for x in items]
