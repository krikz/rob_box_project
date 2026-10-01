"""Тесты хук-фетчера сэмплов DJ_Dave (issue #3219). Без сети, на моках.

Фетчер: ``docker/vision/scripts/resource_pack/fetch_dj_dave_samples.py``,
эталоны — ``dj_dave_samples.lock.json`` рядом. Образец —
``test_fetch_renardo_samples.py``.
"""

import hashlib
import importlib.util
import io
import json
import pathlib
import re
from urllib.error import URLError


REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
PACK_DIR = REPO_ROOT / "docker" / "vision" / "scripts" / "resource_pack"
MODULE_PATH = PACK_DIR / "fetch_dj_dave_samples.py"
LOCK_PATH = PACK_DIR / "dj_dave_samples.lock.json"
MANIFEST_PATH = PACK_DIR / "manifest.yaml"
APPLY_PATH = PACK_DIR / "apply_resource_pack.sh"
SPILLTAB_COMMIT = "f1f83076bde52f7d285abeab1af7ecee47904c45"


def load_module():
    spec = importlib.util.spec_from_file_location("fetch_dj_dave_samples", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class FakeResponse(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def make_entry(dest: str, payload: bytes, **over):
    entry = {
        "repo": "owner/repo",
        "commit": "a" * 40,
        "src": f"snd/{dest}",
        "dest": dest,
        "size": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }
    entry.update(over)
    return entry


def opener_for(payloads: dict, calls: list):
    def opener(url: str):
        calls.append(url)
        for key, data in payloads.items():
            if url.endswith(key):
                return FakeResponse(data)
        raise URLError("404")

    return opener


# ---------------------------------------------------------------------------
# lock-файл: то, что попадает в git вместо самих сэмплов
# ---------------------------------------------------------------------------


def test_lock_entries_are_pinned_and_hashed() -> None:
    files = load_module().load_lock(LOCK_PATH)
    assert len(files) > 100
    for entry in files:
        assert re.fullmatch(r"[0-9a-f]{40}", entry["commit"]), entry
        assert re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]), entry
        assert entry["size"] > 100, f"пустышка failsafe не берём: {entry}"
        assert not entry["dest"].startswith("/") and ".." not in entry["dest"]


def test_lock_dests_are_unique() -> None:
    dests = [e["dest"] for e in load_module().load_lock(LOCK_PATH)]
    assert len(dests) == len(set(dests))


def test_spilltab_comes_from_pinned_commit_not_head() -> None:
    (entry,) = [e for e in load_module().load_lock(LOCK_PATH) if e["dest"] == "algorave/spilltab/spilltab.wav"]
    assert entry["commit"] == SPILLTAB_COMMIT
    assert entry["repo"] == "algorave-dave/samples"


def test_lock_takes_only_needed_dirt_folders_and_no_failsafe_stubs() -> None:
    files = load_module().load_lock(LOCK_PATH)
    dirt = {e["dest"].split("/")[1] for e in files if e["dest"].startswith("dirt/")}
    assert dirt == {"tech", "psr", "hh", "cp"}
    failsafe = {e["dest"] for e in files if "failsafe" in e["dest"]}
    assert failsafe == {"algorave/failsafe/CHOPS.wav", "algorave/failsafe/FX.wav"}


def test_lock_keeps_mp3_as_source_hash_without_conversion() -> None:
    mp3 = [e for e in load_module().load_lock(LOCK_PATH) if e["dest"].endswith(".mp3")]
    assert len(mp3) == 46
    assert all(e["repo"] == "lil-data/dj_dave-array_remix" for e in mp3)


def test_raw_url_is_quoted_and_pinned() -> None:
    module = load_module()
    url = module.raw_url(make_entry("x.wav", b"1", src="machines/Hat Closed.wav", commit="b" * 40))
    assert url == f"https://raw.githubusercontent.com/owner/repo/{'b' * 40}/machines/Hat%20Closed.wav"


# ---------------------------------------------------------------------------
# скачивание
# ---------------------------------------------------------------------------


def test_fetch_all_downloads_and_verifies(tmp_path) -> None:
    module = load_module()
    payload = b"RIFF-fake-wav"
    entry = make_entry("algorave/a.wav", payload)
    calls: list = []

    problems = module.fetch_all([entry], tmp_path, opener_for({"a.wav": payload}, calls), sleep=lambda s: None)

    assert problems == []
    assert (tmp_path / "algorave" / "a.wav").read_bytes() == payload
    assert not list(tmp_path.rglob("*.part"))
    assert len(calls) == 1


def test_fetch_all_is_idempotent_without_network(tmp_path) -> None:
    module = load_module()
    payload = b"data"
    entry = make_entry("array/k.mp3", payload)
    module.fetch_all([entry], tmp_path, opener_for({"k.mp3": payload}, []), sleep=lambda s: None)

    calls: list = []
    problems = module.fetch_all([entry], tmp_path, opener_for({}, calls), sleep=lambda s: None)

    assert problems == []
    assert calls == []


def test_sha_mismatch_is_rejected_and_not_installed(tmp_path) -> None:
    module = load_module()
    entry = make_entry("a.wav", b"expected")
    calls: list = []

    problems = module.fetch_all([entry], tmp_path, opener_for({"a.wav": b"SUBSTITUTED"}, calls), sleep=lambda s: None)

    assert len(problems) == 1 and "sha256 не совпал" in problems[0]
    assert not (tmp_path / "a.wav").exists()
    assert not list(tmp_path.rglob("*.part"))
    assert len(calls) == 1, "подмена контента не лечится ретраем"


def test_corrupt_existing_file_is_redownloaded(tmp_path) -> None:
    module = load_module()
    payload = b"good"
    entry = make_entry("a.wav", payload)
    (tmp_path / "a.wav").write_bytes(b"bad!")

    problems = module.fetch_all([entry], tmp_path, opener_for({"a.wav": payload}, []), sleep=lambda s: None)

    assert problems == []
    assert (tmp_path / "a.wav").read_bytes() == payload


def test_network_errors_are_retried_then_reported(tmp_path) -> None:
    module = load_module()
    entry = make_entry("a.wav", b"x")
    calls: list = []

    problems = module.fetch_all([entry], tmp_path, opener_for({}, calls), sleep=lambda s: None)

    assert len(calls) == module.RETRIES
    assert len(problems) == 1 and "сеть" in problems[0]
    assert not (tmp_path / "a.wav").exists()


# ---------------------------------------------------------------------------
# main: маркер только после полного успеха
# ---------------------------------------------------------------------------


def _write_lock(path: pathlib.Path, entries: list) -> None:
    path.write_text(json.dumps({"files": entries}), encoding="utf-8")


def test_main_writes_marker_only_on_full_success(tmp_path, monkeypatch) -> None:
    module = load_module()
    payload = b"ok"
    lock = tmp_path / "lock.json"
    _write_lock(lock, [make_entry("a.wav", payload)])
    target = tmp_path / "dj_dave"
    monkeypatch.setattr(module, "urlopen", lambda req, timeout=None: FakeResponse(payload))

    assert module.main(["--target", str(target), "--lock", str(lock)]) == 0
    assert (target / module.MARKER_FILENAME).exists()


def test_main_removes_marker_and_fails_on_bad_download(tmp_path, monkeypatch) -> None:
    module = load_module()
    lock = tmp_path / "lock.json"
    _write_lock(lock, [make_entry("a.wav", b"expected")])
    target = tmp_path / "dj_dave"
    target.mkdir()
    (target / module.MARKER_FILENAME).write_text("stale", encoding="utf-8")
    monkeypatch.setattr(module, "urlopen", lambda req, timeout=None: FakeResponse(b"SUBSTITUTED"))

    assert module.main(["--target", str(target), "--lock", str(lock)]) == 1
    assert not (target / module.MARKER_FILENAME).exists()


# ---------------------------------------------------------------------------
# проводка: манифест и apply_resource_pack.sh
# ---------------------------------------------------------------------------


def test_manifest_entry_is_soft_hook_with_marker() -> None:
    text = MANIFEST_PATH.read_text(encoding="utf-8")
    block = text[text.index("- name: dj-dave-samples"):]
    block = block.split("\n  - name:", 1)[0].split("\r\n  - name:", 1)[0]
    assert "fetch_hook: fetch_dj_dave_samples" in block
    assert "required: soft" in block
    assert "target: /opt/rob_box/samples/dj_dave" in block
    assert "verify_file: .complete" in block
    assert re.search(r'sha256:\s*""', block), "у записи с fetch_hook sha256 обязан быть пуст"


def test_apply_script_resolves_the_hook() -> None:
    assert "fetch_dj_dave_samples)" in APPLY_PATH.read_text(encoding="utf-8")


def test_hook_marker_matches_fetcher() -> None:
    assert load_module().MARKER_FILENAME == ".complete"


def test_no_sample_files_ship_in_git() -> None:
    assert not list(PACK_DIR.rglob("*.wav")) and not list(PACK_DIR.rglob("*.mp3"))
