"""Тесты Ресурсного пака: manifest.yaml + apply_resource_pack.sh.

План: ``docs/plans/2026-09-15-resource-pack.md`` §7 Этап 1 п.3 — «юнит/smoke-тест
скрипта на фикстуре-манифесте (без реальной сети)». До этого файла тестов на
доставку бинарных ресурсов в репозитории не было вообще: ни одного
``test_download*`` рядом с ``download_yolov8n_hef.sh`` / ``download_retinaface_hef.sh``.

Что здесь проверяется:

* **Манифест** (реальный, тот что поедет на робота) — валидный YAML, у записей
  есть обязательные поля, sha256 записан в том виде, который скрипт умеет
  сравнивать.
* **Скрипт** — на фикстуре с локальным HTTP-сервером (``127.0.0.1``, временный
  каталог). Настоящие URL Hailo/Vosk/Silero НЕ дёргаются: тест обязан быть
  зелёным на машине без интернета, иначе он проверяет сеть, а не код.

Ключевые свойства контракта (план §2.3), под каждое — отдельный тест:

* идемпотентность (второй прогон — no-op без скачивания);
* sha256 mismatch → ничего не установлено, exit 1 (а не «тихо скачал не то»);
* отсутствие ``required: hard`` → exit 1; ``required: soft`` → exit 0 + явная
  объявленная деградация в stderr (ADR-0018);
* ``gated`` — скрипт не делает вид, что проверил, если не сказали, на каком
  хосте он работает.
"""
from __future__ import annotations

import functools
import hashlib
import http.server
import os
import shutil
import socketserver
import subprocess
import threading
import zipfile
from pathlib import Path

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
PACK_DIR = REPO_ROOT / "docker" / "vision" / "scripts" / "resource_pack"
SCRIPT = PACK_DIR / "apply_resource_pack.sh"
MANIFEST = PACK_DIR / "manifest.yaml"

BASH = shutil.which("bash")

pytestmark = pytest.mark.skipif(
    BASH is None,
    reason="bash недоступен (скрипт host-side, проверять нечем)",
)


# ===========================================================================
# Инфраструктура фикстур
# ===========================================================================
@pytest.fixture(scope="module")
def http_root(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Каталог, который отдаёт локальный HTTP-сервер (роль «интернета»)."""
    return tmp_path_factory.mktemp("resource_pack_http")


@pytest.fixture(scope="module")
def http_base(http_root: Path):
    """Локальный HTTP-сервер вместо реального hailo-model-zoo.s3.

    Тест обязан проходить без интернета: настоящие URL из манифеста здесь
    не используются никогда.
    """
    handler = functools.partial(
        http.server.SimpleHTTPRequestHandler, directory=str(http_root)
    )

    class QuietServer(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

        def handle_error(self, request, client_address):  # noqa: D102
            pass  # не шумим в stderr при закрытом соединении

    with QuietServer(("127.0.0.1", 0), handler) as httpd:
        thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        thread.start()
        try:
            yield f"http://127.0.0.1:{httpd.server_address[1]}"
        finally:
            httpd.shutdown()


def sha256_of(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def write_manifest(path: Path, resources: list[dict]) -> Path:
    """Пишет фикстуру-манифест в том же формате, что и боевой."""
    path.write_text(
        yaml.safe_dump(
            {"version": 1, "resources": resources},
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return path


def run_pack(manifest: Path, root: Path, *args: str, env_extra: dict | None = None):
    """Запуск apply_resource_pack.sh на фикстуре."""
    env = os.environ.copy()
    env.update(env_extra or {})
    return subprocess.run(
        [
            BASH,
            str(SCRIPT),
            "--manifest",
            str(manifest),
            "--root",
            str(root),
            *args,
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=120,
        env=env,
    )


def hef_entry(name: str, url: str, sha: str, required: str = "hard") -> dict:
    return {
        "name": name,
        "type": "open",
        "kind": "hef",
        "url": url,
        "sha256": sha,
        "unpack": "none",
        "target": f"/opt/rob_box/models/{name}.hef",
        "required": required,
        "on_missing": "fail-deploy" if required == "hard" else "degrade",
        "degrade_note": "фикстура: потребитель ушёл бы в stub",
    }


# ===========================================================================
# 1. Боевой манифест
# ===========================================================================
def test_manifest_parses_as_yaml() -> None:
    data = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    assert data["version"] == 1
    assert isinstance(data["resources"], list) and data["resources"]


def test_manifest_names_are_unique() -> None:
    data = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    names = [r["name"] for r in data["resources"]]
    assert len(names) == len(set(names)), f"дубли имён в манифесте: {names}"


def test_open_entries_have_contract_fields() -> None:
    """Каждая open-запись обязана уметь ответить: откуда, куда, что если нет."""
    data = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    for res in data["resources"]:
        if res.get("type") != "open":
            continue
        for key in ("url", "target", "required", "on_missing"):
            assert key in res, f"{res['name']}: нет обязательного поля {key}"
        assert res["required"] in ("hard", "soft"), res["name"]
        assert res["target"].startswith("/opt/rob_box/"), (
            f"{res['name']}: target={res['target']} вне канонических host-каталогов "
            f"(план §5: третьего каталога не заводим)"
        )
        assert res.get("unpack", "none") in ("none", "zip"), res["name"]


def test_gated_entries_are_declared_placeholders() -> None:
    """gated — заготовки Этапа 4: sha256 пуст, но поля-обязательства на месте.

    sha256 считает человек на машине с артефактом (план §7 Этап 4 п.1), и это
    честно записано пустой строкой, а не выдуманным значением.
    """
    data = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    gated = [r for r in data["resources"] if r.get("type") == "gated"]
    assert gated, "gated-записи пропали из манифеста"
    for res in gated:
        assert res["target_host"] in ("vision-pi", "katana"), res["name"]
        assert "vendor_file" in res and "version" in res, res["name"]
        assert res["required"] in ("hard", "soft"), res["name"]


def test_sha256_fields_are_lowercase_hex_or_empty() -> None:
    """Регистр важен исторически: на заглавном хексе уже горел retinaface (#2599)."""
    data = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    for res in data["resources"]:
        sha = res.get("sha256", "")
        if sha == "":
            continue
        assert len(sha) == 64 and all(c in "0123456789abcdef" for c in sha), (
            f"{res['name']}: sha256='{sha}' — ожидается 64 hex-символа в нижнем регистре"
        )


def test_retinaface_sha_matches_legacy_script() -> None:
    """Единственный уже зафиксированный эталон не должен потеряться при переезде."""
    legacy = (
        REPO_ROOT
        / "docker" / "vision" / "scripts" / "vision-hailo" / "download_retinaface_hef.sh"
    ).read_text(encoding="utf-8")
    data = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    entry = next(r for r in data["resources"] if r["name"] == "retinaface-hef")
    assert entry["sha256"].lower() in legacy.lower(), (
        "sha256 retinaface в манифесте разошёлся с download_retinaface_hef.sh"
    )


def test_crlf_manifest_is_parsed_in_full(tmp_path: Path, http_base: str) -> None:
    """Манифест с CRLF обязан разбираться целиком, а не частично.

    Поймано 21.09.2026 на живой Vision Pi: файл, приехавший с Windows-машины
    с \r на конце строк, awk-парсер разобрал как ОДНУ запись из десяти и
    молча продолжил. Молчаливо неполный манифест хуже явной ошибки — деплой
    разложил бы часть ресурсов и отчитался об успехе.
    """
    lf = write_manifest(
        tmp_path / "lf.yaml",
        [
            hef_entry("one", f"{http_base}/one.hef", "", required="soft"),
            hef_entry("two", f"{http_base}/two.hef", "", required="soft"),
            hef_entry("three", f"{http_base}/three.hef", "", required="soft"),
        ],
    )
    crlf = tmp_path / "crlf.yaml"
    crlf.write_bytes(lf.read_bytes().replace(b"\n", b"\r\n"))

    result = run_pack(crlf, tmp_path / "root", "--dry-run")

    assert result.returncode == 0, result.stdout + result.stderr
    assert "записей:   3" in result.stdout, result.stdout
    for name in ("one", "two", "three"):
        assert name in result.stdout, result.stdout


def test_script_is_executable_bash_and_syntactically_valid() -> None:
    assert SCRIPT.read_text(encoding="utf-8").startswith("#!/usr/bin/env bash")
    result = subprocess.run([BASH, "-n", str(SCRIPT)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_real_manifest_dry_run_is_clean(tmp_path: Path) -> None:
    """`--dry-run` на боевом манифесте: ни сети, ни записи, exit 0 (план §10 п.5)."""
    root = tmp_path / "opt_rob_box"
    result = run_pack(MANIFEST, root, "--dry-run")
    assert result.returncode == 0, result.stdout + result.stderr
    assert not root.exists(), "DRY-RUN создал каталоги на диске"
    assert "DRY-RUN" in result.stdout


# ===========================================================================
# 2. Скрипт на фикстуре
# ===========================================================================
def test_downloads_and_installs_open_resource(
    tmp_path: Path, http_root: Path, http_base: str
) -> None:
    payload = b"FAKE-HEF-PAYLOAD-yolo\n"
    (http_root / "good.hef").write_bytes(payload)
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [hef_entry("good", f"{http_base}/good.hef", sha256_of(payload))],
    )
    root = tmp_path / "opt"

    result = run_pack(manifest, root, "--only", "good")

    assert result.returncode == 0, result.stdout + result.stderr
    installed = root / "models" / "good.hef"
    assert installed.read_bytes() == payload
    assert "sha256 OK" in result.stdout
    # временных файлов после себя не оставляем
    assert [p.name for p in (root / "models").iterdir()] == ["good.hef"]


def test_second_run_is_noop(tmp_path: Path, http_root: Path, http_base: str) -> None:
    """Идемпотентность (план §2.3 п.1): повтор — no-op, файл не переписан."""
    payload = b"FAKE-HEF-IDEMPOTENT\n"
    (http_root / "idem.hef").write_bytes(payload)
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [hef_entry("idem", f"{http_base}/idem.hef", sha256_of(payload))],
    )
    root = tmp_path / "opt"

    first = run_pack(manifest, root, "--only", "idem")
    assert first.returncode == 0, first.stdout + first.stderr
    installed = root / "models" / "idem.hef"
    mtime_before = installed.stat().st_mtime_ns

    second = run_pack(manifest, root, "--only", "idem")
    assert second.returncode == 0, second.stdout + second.stderr
    assert "no-op" in second.stdout
    assert "скачиваю" not in second.stdout, "второй прогон полез в сеть"
    assert installed.stat().st_mtime_ns == mtime_before


def test_sha_mismatch_on_download_installs_nothing(
    tmp_path: Path, http_root: Path, http_base: str
) -> None:
    """«Тихо скачал не то» запрещено: exit 1 и пустой target."""
    (http_root / "wrong.hef").write_bytes(b"NOT-WHAT-MANIFEST-EXPECTS\n")
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [hef_entry("wrong", f"{http_base}/wrong.hef", sha256_of(b"something-else"))],
    )
    root = tmp_path / "opt"

    result = run_pack(manifest, root, "--only", "wrong")

    assert result.returncode == 1, result.stdout + result.stderr
    assert not (root / "models" / "wrong.hef").exists()
    assert "НЕ СОВПАЛ" in result.stderr
    # и никакого мусора из mktemp
    leftovers = list((root / "models").iterdir()) if (root / "models").exists() else []
    assert leftovers == [], f"остались временные файлы: {leftovers}"


def test_sha_mismatch_is_hard_even_for_soft_resource(
    tmp_path: Path, http_root: Path, http_base: str
) -> None:
    """Отсутствие — деградация, подмена содержимого — нет (см. шапку манифеста)."""
    (http_root / "softbad.hef").write_bytes(b"BAD\n")
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [
            hef_entry(
                "softbad", f"{http_base}/softbad.hef", sha256_of(b"good"), required="soft"
            )
        ],
    )
    result = run_pack(manifest, tmp_path / "opt", "--only", "softbad")
    assert result.returncode == 1, result.stdout + result.stderr


def test_existing_file_with_wrong_sha_is_not_touched(
    tmp_path: Path, http_root: Path, http_base: str
) -> None:
    """Файл на месте, но не тот → внятный fail, и скрипт его НЕ трогает."""
    payload = b"EXPECTED\n"
    (http_root / "pre.hef").write_bytes(payload)
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [hef_entry("pre", f"{http_base}/pre.hef", sha256_of(payload))],
    )
    root = tmp_path / "opt"
    target = root / "models" / "pre.hef"
    target.parent.mkdir(parents=True)
    target.write_bytes(b"STALE-VERSION\n")

    result = run_pack(manifest, root, "--only", "pre")

    assert result.returncode == 1, result.stdout + result.stderr
    assert target.read_bytes() == b"STALE-VERSION\n", "скрипт молча перезаписал файл"
    assert "sha256 НЕ СОВПАЛ" in result.stderr
    assert "--force" in result.stderr, "не подсказал, чем это чинится"


def test_force_redownloads_mismatched_file(
    tmp_path: Path, http_root: Path, http_base: str
) -> None:
    payload = b"FRESH\n"
    (http_root / "forced.hef").write_bytes(payload)
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [hef_entry("forced", f"{http_base}/forced.hef", sha256_of(payload))],
    )
    root = tmp_path / "opt"
    target = root / "models" / "forced.hef"
    target.parent.mkdir(parents=True)
    target.write_bytes(b"STALE\n")

    result = run_pack(manifest, root, "--only", "forced", "--force")

    assert result.returncode == 0, result.stdout + result.stderr
    assert target.read_bytes() == payload


def test_missing_hard_resource_fails_deploy(tmp_path: Path, http_base: str) -> None:
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [hef_entry("gone", f"{http_base}/does-not-exist.hef", "", required="hard")],
    )
    result = run_pack(manifest, tmp_path / "opt", "--only", "gone")
    assert result.returncode == 1, result.stdout + result.stderr
    assert "HARD FAIL" in result.stderr
    assert "required: hard" in result.stderr


def test_missing_soft_resource_degrades_loudly(tmp_path: Path, http_base: str) -> None:
    """soft: деплой жив, но деградация названа вслух (ADR-0018)."""
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [hef_entry("softgone", f"{http_base}/nope.hef", "", required="soft")],
    )
    result = run_pack(manifest, tmp_path / "opt", "--only", "softgone")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "ДЕГРАДАЦИЯ" in result.stderr
    assert "stub" in result.stderr, "не сказал, что именно деградировало"
    assert "объявленная деградация" in result.stderr


def test_empty_sha_is_reported_as_unverified(
    tmp_path: Path, http_root: Path, http_base: str
) -> None:
    """Пустой sha256 — не «проверено», а честное «не проверено»."""
    (http_root / "nosha.hef").write_bytes(b"whatever\n")
    manifest = write_manifest(
        tmp_path / "m.yaml", [hef_entry("nosha", f"{http_base}/nosha.hef", "")]
    )
    result = run_pack(manifest, tmp_path / "opt", "--only", "nosha")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "НЕ проверено" in result.stderr


@pytest.mark.skipif(
    shutil.which("unzip") is None and shutil.which("python3") is None,
    reason="нечем распаковать zip (ни unzip, ни python3)",
)
def test_zip_resource_unpacked_and_verified(
    tmp_path: Path, http_root: Path, http_base: str
) -> None:
    """model-zip: распаковка + проверка sentinel-файла (как test -f .../README)."""
    archive = http_root / "model.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("vosk-fixture/README", "fixture model\n")
        zf.writestr("vosk-fixture/am/final.mdl", "weights\n")
    payload = archive.read_bytes()

    manifest = write_manifest(
        tmp_path / "m.yaml",
        [
            {
                "name": "vosk-fixture",
                "type": "open",
                "kind": "model-zip",
                "url": f"{http_base}/model.zip",
                "sha256": sha256_of(payload),
                "unpack": "zip",
                "verify_file": "README",
                "target": "/opt/rob_box/models/vosk-fixture",
                "required": "hard",
                "on_missing": "fail-deploy",
            }
        ],
    )
    root = tmp_path / "opt"

    result = run_pack(manifest, root, "--only", "vosk-fixture")

    assert result.returncode == 0, result.stdout + result.stderr
    assert (root / "models" / "vosk-fixture" / "README").exists()
    assert (root / "models" / "vosk-fixture" / "am" / "final.mdl").exists()
    assert not list((root / "models").glob(".vosk-fixture*")), "остался temp-архив"

    again = run_pack(manifest, root, "--only", "vosk-fixture")
    assert again.returncode == 0
    assert "no-op" in again.stdout


@pytest.mark.skipif(
    shutil.which("unzip") is None and shutil.which("python3") is None,
    reason="нечем распаковать zip (ни unzip, ни python3)",
)
def test_zip_without_expected_sentinel_fails(
    tmp_path: Path, http_root: Path, http_base: str
) -> None:
    """Распаковался не тот архив → hard-ресурс роняет прогон, а не «ну и ладно»."""
    archive = http_root / "bad-model.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("vosk-bad/other.txt", "not a model\n")

    manifest = write_manifest(
        tmp_path / "m.yaml",
        [
            {
                "name": "vosk-bad",
                "type": "open",
                "kind": "model-zip",
                "url": f"{http_base}/bad-model.zip",
                "sha256": "",
                "unpack": "zip",
                "verify_file": "README",
                "target": "/opt/rob_box/models/vosk-bad",
                "required": "hard",
                "on_missing": "fail-deploy",
            }
        ],
    )
    result = run_pack(manifest, tmp_path / "opt", "--only", "vosk-bad")
    assert result.returncode == 1, result.stdout + result.stderr
    assert "README" in result.stderr


def test_gated_without_host_is_skipped_not_faked(tmp_path: Path) -> None:
    """Скрипт не имеет права «проверить» vendor-файл, не зная, на каком он хосте."""
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [
            {
                "name": "vendor-fixture",
                "type": "gated",
                "kind": "wheel",
                "vendor_file": "fixture.whl",
                "version": "4.24.0",
                "sha256": "",
                "target_host": "katana",
                "target": "/opt/rob_box/vendor/fixture.whl",
                "required": "soft",
                "on_missing": "degrade-build",
            }
        ],
    )
    result = run_pack(manifest, tmp_path / "opt")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "пропущено" in result.stdout
    assert "не задан" in result.stdout


def test_gated_missing_on_declared_host_explains_what_to_do(tmp_path: Path) -> None:
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [
            {
                "name": "vendor-fixture",
                "type": "gated",
                "kind": "wheel",
                "vendor_file": "fixture.whl",
                "version": "4.24.0",
                "sha256": "",
                "target_host": "katana",
                "target": "/opt/rob_box/vendor/fixture.whl",
                "required": "soft",
                "on_missing": "degrade-build",
                "degrade_note": "сборка vision-hailo без биндинга",
            }
        ],
    )
    result = run_pack(manifest, tmp_path / "opt", "--host", "katana")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Developer Zone" in result.stderr
    assert "hailo-vendor-artifacts.md" in result.stderr


def test_gated_present_without_sha_says_it_is_not_a_check(tmp_path: Path) -> None:
    root = tmp_path / "opt"
    (root / "vendor").mkdir(parents=True)
    (root / "vendor" / "fixture.whl").write_bytes(b"binary\n")
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [
            {
                "name": "vendor-fixture",
                "type": "gated",
                "kind": "wheel",
                "vendor_file": "fixture.whl",
                "version": "4.24.0",
                "sha256": "",
                "target_host": "vision-pi",
                "target": "/opt/rob_box/vendor/fixture.whl",
                "required": "soft",
                "on_missing": "degrade",
            }
        ],
    )
    result = run_pack(manifest, root, "--host", "vision-pi")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "НЕ проверка" in result.stderr


def test_unknown_only_name_is_usage_error(tmp_path: Path, http_base: str) -> None:
    """Опечатка в workflow не должна выглядеть успешным прогоном."""
    manifest = write_manifest(
        tmp_path / "m.yaml", [hef_entry("real", f"{http_base}/real.hef", "")]
    )
    result = run_pack(manifest, tmp_path / "opt", "--only", "raal")
    assert result.returncode == 2, result.stdout + result.stderr
    assert "такой записи в манифесте нет" in result.stderr


def test_broken_manifest_is_usage_error(tmp_path: Path) -> None:
    manifest = tmp_path / "broken.yaml"
    manifest.write_text("version: 1\nnothing_here: true\n", encoding="utf-8")
    result = run_pack(manifest, tmp_path / "opt")
    assert result.returncode == 2, result.stdout + result.stderr
    assert "resources" in result.stderr


def test_missing_manifest_is_usage_error(tmp_path: Path) -> None:
    result = run_pack(tmp_path / "absent.yaml", tmp_path / "opt")
    assert result.returncode == 2
    assert "манифест не найден" in result.stderr


def test_dry_run_does_not_download(tmp_path: Path, http_root: Path, http_base: str) -> None:
    (http_root / "dry.hef").write_bytes(b"payload\n")
    manifest = write_manifest(
        tmp_path / "m.yaml", [hef_entry("dry", f"{http_base}/dry.hef", "")]
    )
    root = tmp_path / "opt"
    result = run_pack(manifest, root, "--only", "dry", "--dry-run")
    assert result.returncode == 0, result.stdout + result.stderr
    assert not (root / "models" / "dry.hef").exists()
    assert "DRY-RUN" in result.stdout


def test_registry_only_types_are_skipped(tmp_path: Path) -> None:
    """build-time/git записи — реестр, а не доставка (план §6.3/§6.4)."""
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [
            {"name": "renardo", "type": "build-time", "note": "ADR-0111"},
            {"name": "sounds", "type": "git", "note": "git reset --hard"},
        ],
    )
    result = run_pack(manifest, tmp_path / "opt")
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.count("доставляется не этим швом") == 2


def test_manifest_covers_stt_tts_models() -> None:
    """Этап 3: Vosk/Silero обязаны быть в манифесте, иначе их никто не положит.

    До Этапа 3 их качала ступень сборки voice-base. Ступень удалена — если
    запись исчезнет и отсюда, модели не появятся вообще нигде, а узнаем мы об
    этом по «Folder does not contain model files» на роботе.
    """
    data = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    by_name = {r["name"]: r for r in data["resources"]}

    vosk = by_name["vosk-ru-small"]
    assert vosk["target"] == "/opt/rob_box/models/vosk-model-small-ru-0.22"
    assert vosk["unpack"] == "zip" and vosk["verify_file"] == "README"
    assert vosk["required"] == "hard", "без Vosk stt_node не стартует — это не soft"

    silero = by_name["silero-tts-v5-ru"]
    assert silero["target"] == "/opt/rob_box/models/silero/v5_ru.pt"
    assert silero["required"] == "hard"


def test_container_paths_did_not_move() -> None:
    """Переезд «образ → host bind-mount» не должен менять путь ВНУТРИ контейнера.

    Весь смысл §6.1 плана: ни один Python-файл не правится, потому что
    /models/... остаётся /models/... — меняется только, чем он наполнен.
    """
    compose = yaml.safe_load(
        (REPO_ROOT / "docker" / "vision" / "docker-compose.yaml").read_text(
            encoding="utf-8"
        )
    )
    volumes = compose["services"]["voice-assistant"]["volumes"]
    assert "/opt/rob_box/models:/models:ro" in volumes, (
        "voice-assistant не монтирует host-каталог моделей: после удаления "
        "ступени скачивания из voice_base/Dockerfile контейнер останется "
        "без Vosk и Silero"
    )

    stt_cfg = (
        REPO_ROOT / "docker" / "vision" / "config" / "voice_assistant" / "stt_node.yaml"
    ).read_text(encoding="utf-8")
    assert "/models/vosk-model-small-ru-0.22" in stt_cfg


def test_voice_base_no_longer_downloads_models() -> None:
    """Ступень скачивания моделей ушла из образа (deletion test, план §8)."""
    dockerfile = (
        REPO_ROOT / "docker" / "vision" / "voice_base" / "Dockerfile"
    ).read_text(encoding="utf-8")
    active = [
        line
        for line in dockerfile.splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]
    body = "\n".join(active)
    for needle in (
        "alphacephei.com/vosk/models",
        "models.silero.ai",
        "torch.hub.load",
    ):
        assert needle not in body, (
            f"voice_base/Dockerfile снова качает {needle} на этапе сборки — "
            f"это ровно то, что Ресурсный пак (Этап 3) убрал: сборка не должна "
            f"зависеть от сети, а модели приходят с хоста"
        )


def test_env_vars_are_honoured(tmp_path: Path, http_root: Path, http_base: str) -> None:
    """RESOURCE_PACK_MANIFEST / _ROOT / _FORCE — интерфейс из плана §2.2."""
    payload = b"ENVPATH\n"
    (http_root / "env.hef").write_bytes(payload)
    manifest = write_manifest(
        tmp_path / "m.yaml",
        [hef_entry("env", f"{http_base}/env.hef", sha256_of(payload))],
    )
    root = tmp_path / "opt"
    result = subprocess.run(
        [BASH, str(SCRIPT), "--only", "env"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=120,
        env={
            **os.environ,
            "RESOURCE_PACK_MANIFEST": str(manifest),
            "RESOURCE_PACK_ROOT": str(root),
        },
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert (root / "models" / "env.hef").read_bytes() == payload
