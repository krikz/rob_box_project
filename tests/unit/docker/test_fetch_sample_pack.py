"""Тесты хук-фетчера сэмпл-паков (DJ_Dave #3219, Sonic Pi, MuldjordKit). Без сети, на моках.

Фетчер: ``docker/vision/scripts/resource_pack/fetch_sample_pack.py`` (один на все паки),
эталоны — ``<пак>.lock.json`` рядом. Образец — ``test_fetch_renardo_samples.py``.
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
MODULE_PATH = PACK_DIR / "fetch_sample_pack.py"
LOCK_PATH = PACK_DIR / "dj_dave_samples.lock.json"
SONICPI_LOCK_PATH = PACK_DIR / "sonicpi_samples.lock.json"
MULDJORD_LOCK_PATH = PACK_DIR / "muldjord_kit.lock.json"
MANIFEST_PATH = PACK_DIR / "manifest.yaml"
APPLY_PATH = PACK_DIR / "apply_resource_pack.sh"
SPILLTAB_COMMIT = "f1f83076bde52f7d285abeab1af7ecee47904c45"


def load_module():
    spec = importlib.util.spec_from_file_location("fetch_sample_pack", MODULE_PATH)
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
    files = load_module().load_lock(LOCK_PATH)[1]
    assert len(files) > 100
    for entry in files:
        assert re.fullmatch(r"[0-9a-f]{40}", entry["commit"]), entry
        assert re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]), entry
        assert entry["size"] > 100, f"пустышка failsafe не берём: {entry}"
        assert not entry["dest"].startswith("/") and ".." not in entry["dest"]


def test_lock_dests_are_unique() -> None:
    dests = [e["dest"] for e in load_module().load_lock(LOCK_PATH)[1]]
    assert len(dests) == len(set(dests))


def test_spilltab_comes_from_pinned_commit_not_head() -> None:
    (entry,) = [e for e in load_module().load_lock(LOCK_PATH)[1] if e["dest"] == "algorave/spilltab/spilltab.wav"]
    assert entry["commit"] == SPILLTAB_COMMIT
    assert entry["repo"] == "algorave-dave/samples"


def test_lock_takes_only_needed_dirt_folders_and_no_failsafe_stubs() -> None:
    files = load_module().load_lock(LOCK_PATH)[1]
    dirt = {e["dest"].split("/")[1] for e in files if e["dest"].startswith("dirt/")}
    assert dirt == {"tech", "psr", "hh", "cp"}
    failsafe = {e["dest"] for e in files if "failsafe" in e["dest"]}
    assert failsafe == {"algorave/failsafe/CHOPS.wav", "algorave/failsafe/FX.wav"}


def test_lock_keeps_mp3_as_source_hash_without_conversion() -> None:
    mp3 = [e for e in load_module().load_lock(LOCK_PATH)[1] if e["dest"].endswith(".mp3")]
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
    assert "fetch_dj_dave_samples|" in APPLY_PATH.read_text(encoding="utf-8")


def test_hook_marker_matches_fetcher() -> None:
    assert load_module().MARKER_FILENAME == ".complete"


def test_no_sample_files_ship_in_git() -> None:
    assert not list(PACK_DIR.rglob("*.wav")) and not list(PACK_DIR.rglob("*.mp3"))


# ---------------------------------------------------------------------------
# новые паки: Sonic Pi (CC0) и MuldjordKit (CC BY 4.0) — тот же фетчер, свои lock-файлы
# ---------------------------------------------------------------------------

NEW_PACKS = {
    # имя записи манифеста: (хук, lock, target, число файлов, repo)
    "sonicpi-samples": ("fetch_sonicpi_samples", SONICPI_LOCK_PATH, "/opt/rob_box/samples/sonicpi", 207,
                        "sonic-pi-net/sonic-pi"),
    "muldjord-kit": ("fetch_muldjord_kit", MULDJORD_LOCK_PATH, "/opt/rob_box/samples/muldjord", 32,
                     "sfzinstruments/DrumGizmo.MuldjordKit"),
}


def _manifest_block(name: str) -> str:
    text = MANIFEST_PATH.read_text(encoding="utf-8")
    block = text[text.index(f"- name: {name}"):]
    return re.split(r"\r?\n  - name:", block, maxsplit=1)[0]


def test_new_locks_are_pinned_hashed_and_unique() -> None:
    module = load_module()
    for name, (_hook, lock, _target, count, repo) in NEW_PACKS.items():
        title, files = module.load_lock(lock)
        assert title and len(files) == count, name
        dests = [e["dest"] for e in files]
        assert len(dests) == len(set(dests)), name
        for entry in files:
            assert entry["repo"] == repo, entry
            assert re.fullmatch(r"[0-9a-f]{40}", entry["commit"]), entry
            assert re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]), entry
            assert entry["size"] > 100, entry
            assert not entry["dest"].startswith("/") and ".." not in entry["dest"], entry


def test_sonicpi_lock_has_the_expected_core_files() -> None:
    dests = {e["dest"] for e in load_module().load_lock(SONICPI_LOCK_PATH)[1]}
    assert {"loop_amen.flac", "loop_amen_full.flac", "loop_breakbeat.flac", "bd_tek.flac", "README.md"} <= dests
    assert sum(1 for d in dests if d.endswith(".flac")) == 206


def test_muldjord_lock_is_one_close_mic_per_instrument_two_hits() -> None:
    files = load_module().load_lock(MULDJORD_LOCK_PATH)[1]
    per_instrument: dict = {}
    for entry in files:
        assert entry["dest"].endswith(".flac")
        per_instrument.setdefault(entry["dest"].split("/")[0], []).append(entry["dest"])
    assert len(per_instrument) == 16
    assert all(len(v) == 2 for v in per_instrument.values()), per_instrument
    assert sum(e["size"] for e in files) < 10 * 1024 * 1024, "16 кГц выход: пак должен остаться лёгким"


def test_new_manifest_entries_are_soft_hooks_with_marker() -> None:
    for name, (hook, _lock, target, _count, _repo) in NEW_PACKS.items():
        block = _manifest_block(name)
        assert f"fetch_hook: {hook}" in block
        assert "required: soft" in block
        assert f"target: {target}" in block
        assert "verify_file: .complete" in block
        assert re.search(r'sha256:\s*""', block), name


def test_manifest_size_matches_lock_sum() -> None:
    module = load_module()
    for name, (_hook, lock, _target, _count, _repo) in NEW_PACKS.items():
        total = sum(e["size"] for e in module.load_lock(lock)[1])
        assert f"size_bytes: {total}" in _manifest_block(name), name


def test_apply_script_maps_every_hook_to_its_lock() -> None:
    text = APPLY_PATH.read_text(encoding="utf-8")
    assert "fetch_dj_dave_samples|fetch_sonicpi_samples|fetch_muldjord_kit)" in text
    for hook, lock in (("fetch_dj_dave_samples", "dj_dave_samples.lock.json"),
                       ("fetch_sonicpi_samples", "sonicpi_samples.lock.json"),
                       ("fetch_muldjord_kit", "muldjord_kit.lock.json")):
        assert f'{hook}) lock="{lock}"' in text
        assert (PACK_DIR / lock).is_file()
    assert "fetch_sample_pack.py" in text and "fetch_dj_dave_samples.py" not in text


def test_deploy_installs_new_packs() -> None:
    workflow = (REPO_ROOT / ".github" / "workflows" / "L-Deploy and Verify.yml").read_text(encoding="utf-8")
    assert "--only renardo-samples,dj-dave-samples,sonicpi-samples,muldjord-kit" in workflow


def test_lock_title_goes_to_the_log(tmp_path, monkeypatch, capsys) -> None:
    module = load_module()
    payload = b"ok"
    lock = tmp_path / "lock.json"
    lock.write_text(json.dumps({"title": "Тестовый пак", "files": [make_entry("a.flac", payload)]}), encoding="utf-8")
    monkeypatch.setattr(module, "urlopen", lambda req, timeout=None: FakeResponse(payload))

    assert module.main(["--target", str(tmp_path / "pak"), "--lock", str(lock)]) == 0
    out = capsys.readouterr().out
    assert "Тестовый пак target:" in out and "Тестовый пак: done" in out
