"""Regression guard для issue #1429: аудио НЕ должно дублироваться между
артефактами e2e-прогона.

История
-------
В run #32165836336 ``recording.wav`` лежал и в ``e2e-voice-recording-<rid>``,
и в ``e2e-voice-artifacts-<rid>`` (md5 совпадали) — лишний трафик в GitHub
storage. Тогда фикс сформулировали как ``exclude: **/*.wav``.

Почему тест переписан
---------------------
``upload-artifact@v7`` параметр ``exclude`` **не поддерживает** — он отвечает
warning'ом "Unexpected input(s) 'exclude'". Реальный фикс в workflow другой:
``Upload e2e artifacts archive`` перечисляет include-паттерны явно
(``*.log``/``*.txt``/``*.json``/…), и wav туда просто не попадает. Старый тест
требовал несуществующий ключ и поэтому падал — то есть guard не работал, пока
дубль возвращался через другую дверь.

Так и вышло: в прогоне 34928781542 ``recording.wav`` (25.5 МБ) уехал разом в
``e2e-voice-recording-<rid>`` (его path — ``**/*.wav``) и в
``e2e-voice-harness-artifacts-<rid>`` (его path — каталог целиком). Шаг
harness-artifacts убран, а тест теперь проверяет ИНВАРИАНТ, а не конкретный
ключ: **wav матчит ровно один upload-шаг**.
"""

from __future__ import annotations

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
WORKFLOW_PATH = REPO_ROOT / ".github/workflows/L-E2E Voice Test.yml"


def _load_workflow_steps():
    """Загрузить YAML и вернуть список steps job'a ``e2e-voice``."""
    yaml = pytest.importorskip("yaml")
    # encoding обязателен: в workflow кириллица в комментариях, а дефолтная
    # локаль на Windows — cp1252, и read_text() падал UnicodeDecodeError
    # ещё до единого assert'а (тест «падал», ничего при этом не проверив).
    data = yaml.safe_load(WORKFLOW_PATH.read_text(encoding="utf-8"))
    jobs = data.get("jobs", {})
    for _job_name, job in jobs.items():
        if "steps" in job:
            return job["steps"]
    raise AssertionError("Workflow has no job with steps")


def _find_step(steps, name: str):
    for step in steps:
        if step.get("name") == name:
            return step
    return None


def _upload_steps(steps):
    return [s for s in steps if str(s.get("uses", "")).startswith("actions/upload-artifact")]


def _paths_of(step) -> list[str]:
    raw = step.get("with", {}).get("path", "")
    if isinstance(raw, list):
        lines = raw
    else:
        lines = str(raw).splitlines()
    return [p.strip() for p in lines if p.strip()]


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
        assert "**/*.wav" in with_block["path"], (
            f"Upload recording: path должен матчить .wav, "
            f"получили {with_block['path']!r}"
        )

    def test_only_one_step_uploads_wav(self):
        """Главная проверка #1429: аудио живёт РОВНО в одном артефакте.

        Проверяем инвариант, а не наличие ключа ``exclude``: шаг может
        исключать wav и явным списком include-паттернов, и это нормально.
        Ловится и старый сценарий (archive без exclude), и новый
        (harness-artifacts, который тянул каталог целиком).
        """
        steps = _load_workflow_steps()
        wav_uploaders = []
        for step in _upload_steps(steps):
            paths = _paths_of(step)
            # Путь без расширения = каталог целиком = заберёт и wav.
            matches_wav = any(
                p.endswith(".wav") or p.endswith("*") or "." not in Path(p).name
                for p in paths
            )
            if matches_wav:
                wav_uploaders.append(step["with"]["name"])
        assert len(wav_uploaders) == 1, (
            "Issue #1429: wav должен уезжать ровно одним артефактом, "
            f"а его забирают: {wav_uploaders}"
        )
        assert wav_uploaders[0].startswith("e2e-voice-recording-"), wav_uploaders

    def test_artifacts_archive_has_no_wav_pattern(self):
        """Полный debug-бандл собирается по явным include-паттернам без wav."""
        steps = _load_workflow_steps()
        step = _find_step(steps, "Upload e2e artifacts archive (full debug bundle)")
        assert step is not None, (
            "Step 'Upload e2e artifacts archive (full debug bundle)' не найден"
        )
        assert step["with"]["name"] == "e2e-voice-artifacts-${{ github.run_id }}"
        paths = _paths_of(step)
        assert paths, "у архива должны быть явные include-паттерны"
        assert all(not p.endswith(".wav") for p in paths), paths
        # Именно явный список — если кто-то заменит его на каталог, wav
        # вернётся в архив молча.
        assert all("*." in Path(p).name for p in paths), paths


class TestNoDuplicateArtifacts:
    """Один и тот же файл не должен уезжать двумя артефактами.

    В прогоне 34928781542 четыре артефакта (transcripts / audio-metrics /
    baseline-diff / acceptance) были побайтовыми копиями файлов, уже лежащих
    внутри ``e2e-voice-artifacts-<rid>``, причём их path шёл через
    ``env.LOCAL_ART_DIR``, который выставляет шаг без ``if:`` — на любом FAIL
    он skipped, и артефакты приезжали пустыми.
    """

    RETIRED = {
        "e2e-voice-transcripts",
        "e2e-voice-audio-metrics",
        "e2e-voice-baseline-diff",
        "e2e-voice-acceptance",
        "e2e-voice-harness-artifacts",
    }

    def test_retired_duplicate_artifacts_are_gone(self):
        steps = _load_workflow_steps()
        names = {
            str(s["with"]["name"]).split("-${{")[0] for s in _upload_steps(steps)
        }
        clash = names & self.RETIRED
        assert not clash, (
            f"эти артефакты дублировали содержимое e2e-voice-artifacts: {clash}"
        )

    def test_artifact_names_are_unique(self):
        steps = _load_workflow_steps()
        names = [str(s["with"]["name"]) for s in _upload_steps(steps)]
        assert len(names) == len(set(names)), names
