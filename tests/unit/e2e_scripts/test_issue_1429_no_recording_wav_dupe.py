"""Regression guard для issue #1429: recording.wav НЕ должен дублироваться
между ``e2e-voice-recording-<run_id>`` и ``e2e-voice-artifacts-<run_id>``.

Сценарий
--------
В run #32165836336 оба артефакта содержали одинаковый ``recording.wav``
(1.98 МБ каждый, md5 совпадают) — итого 4 МБ избыточного трафика в
GitHub storage. Корневая причина: и ``Upload recording``, и ``Upload e2e
artifacts archive`` тянули из ``/tmp/e2e_artifacts_<run_id>/``, поэтому
``recording.wav`` оказывался в обоих.

Фикс: в ``Upload e2e artifacts archive`` добавлен ``exclude: **/*.wav`` —
wav'ы живут только в ``e2e-voice-recording-<run_id>``.

Тест валидирует YAML-структуру workflow:
  - ``Upload recording`` существует и его path — это wav-glob из ART_DIR;
  - ``Upload e2e artifacts archive`` существует И в нём есть
    ``exclude: **/*.wav``.

Если кто-то когда-нибудь удалит exclude — тест упадёт, и баг #1429
вернётся (что и требовалось предотвратить).
"""
from __future__ import annotations

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
WORKFLOW_PATH = REPO_ROOT / ".github/workflows/L-E2E Voice Test.yml"


def _load_workflow_steps():
    """Загрузить YAML и вернуть список steps job'a ``e2e-voice``.

    PyYAML нужен для структурного assert'а; если его нет в тестовом
    окружении — скипаем (тест не должен ломать CI из-за отсутствующей
    опциональной зависимости).
    """
    yaml = pytest.importorskip("yaml")
    data = yaml.safe_load(WORKFLOW_PATH.read_text())
    # jobs.<name>.steps
    jobs = data.get("jobs", {})
    # В workflow одна job с разными названиями — берём первую попавшуюся.
    for job_name, job in jobs.items():
        if "steps" in job:
            return job["steps"]
    raise AssertionError("Workflow has no job with steps")


def _find_step(steps, name: str):
    for step in steps:
        if step.get("name") == name:
            return step
    return None


class TestIssue1429NoRecordingWavDupe:
    """Структурный guard для issue #1429."""

    def test_upload_recording_step_exists(self):
        steps = _load_workflow_steps()
        step = _find_step(steps, "Upload recording")
        assert step is not None, "Step 'Upload recording' не найден"
        with_block = step["with"]
        assert with_block["name"].startswith("e2e-voice-recording-"), (
            f"Upload recording: ожидалось имя e2e-voice-recording-*, "
            f"получили {with_block['name']!r}"
        )
        # path указывает на wav-glob ART_DIR
        assert "**/*.wav" in with_block["path"], (
            f"Upload recording: path должен матчить .wav, "
            f"получили {with_block['path']!r}"
        )

    def test_upload_e2e_artifacts_archive_excludes_wav(self):
        """Главная проверка: 'Upload e2e artifacts archive' должен
        исключать **/*.wav, иначе recording.wav дублируется (#1429).
        """
        steps = _load_workflow_steps()
        step = _find_step(steps, "Upload e2e artifacts archive (full debug bundle)")
        assert step is not None, (
            "Step 'Upload e2e artifacts archive (full debug bundle)' не найден"
        )
        with_block = step["with"]
        assert with_block["name"] == "e2e-voice-artifacts-${{ github.run_id }}", (
            f"Upload e2e artifacts archive: ожидалось имя "
            f"e2e-voice-artifacts-${{ github.run_id }}, "
            f"получили {with_block['name']!r}"
        )
        # exclude должен быть, и должен матчить .wav (issue #1429)
        exclude = with_block.get("exclude")
        assert exclude is not None, (
            "Issue #1429: 'Upload e2e artifacts archive' должен иметь "
            "'exclude: **/*.wav', иначе recording.wav дублируется "
            "между e2e-voice-recording-* и e2e-voice-artifacts-*"
        )
        # exclude может быть строкой или списком строк
        if isinstance(exclude, list):
            joined = "\n".join(exclude)
        else:
            joined = str(exclude)
        assert "**/*.wav" in joined, (
            f"Issue #1429: 'Upload e2e artifacts archive' exclude должен "
            f"содержать '**/*.wav', получили:\n{joined}"
        )