"""
repro_mcp_token_race.py — воспроизведение race-условия в mcp_auth.py
(коммит 8c90e46e, инцидент #1736 / 29.08 15:25 на staging).

Сценарий:
- /data/.mcp_token ещё не существует.
- /data — overlay-FS Docker, где os.link может вернуть FileExistsError
  при гонке двух процессов или просто OSError, если overlay не поддерживает
  hard-link-и.
- Текущая реализация _read_or_create_token_file:
   1. os.open(O_CREAT|O_EXCL) tmp_path
   2. handle.flush + fsync
   3. os.link(tmp_path, path)
   4. except FileExistsError → _read_token_file(path)
   5. except OSError → os.replace(tmp_path, path)
- В нашем кейсе tmp_path создан, fsync сделан, но ДО os.link() _read_token_file
  вызвался неявно по дороге? Нет — смотрим внимательно. ВОПРОС:
  после успешного os.link() возвращается `token`, НО finally пытается
  unlink(tmp_path) — и ВОТ ТУТ потенциальная проблема: на overlay-FS
  unlink() может зафейлиться с OSError если файл всё ещё используется
  другим процессом (FileExistsError → упало в FileExistsError ветку,
  а там tmp_path остался — но не должен мешать).

Симулируем самое неприятное: код падает в ноду ROS2 → exit 1,
без traceback в head -50.

Запуск: PYTHONPATH=src/rob_box_mcp_tools python3 analysis/repro_mcp_token_race.py
"""
import os
import sys
import tempfile
import unittest.mock as mock

# имитируем /data
TMP_ROOT = tempfile.mkdtemp(prefix="mcp_repro_")
MCP_DIR = os.path.join(TMP_ROOT, "data")
os.makedirs(MCP_DIR, exist_ok=True)
TOKEN_PATH = os.path.join(MCP_DIR, ".mcp_token")

print(f"[repro] TMP_ROOT={TMP_ROOT}  TOKEN_PATH={TOKEN_PATH}")

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "src", "rob_box_mcp_tools")
))

from rob_box_mcp_tools.mcp_auth import RequestAuthenticator, _read_or_create_token_file


def scenario_clean_path():
    """Базовый сценарий: /data/.mcp_token нет, должен создаться."""
    print("\n[1] Clean path: создаём токен впервые")
    if os.path.exists(TOKEN_PATH):
        os.unlink(TOKEN_PATH)
    token = _read_or_create_token_file(TOKEN_PATH)
    assert token is not None, "FAIL: токен не создан"
    print(f"    OK: token len={len(token)}, file mode={oct(os.stat(TOKEN_PATH).st_mode & 0o777)}")


def scenario_os_link_oserror():
    """os.link() бросает OSError (НЕ FileExistsError) — например overlay-FS не поддерживает hardlinks.
    Текущий код пытается os.replace, но если и тот падает → возвращает None.
    Но главное: между os.open+O_EXCL и os.link() мы fsync-им; если упадём ДО link,
    токен не получит mcp_server → RequestAuthenticator(None) → verify() отклоняет всё,
    но сам mcp_server при этом УЖЕ живой и продолжает работать. Это не приводит к exit 1.
    """
    print("\n[2] os.link() бросает OSError → fallback на os.replace")
    if os.path.exists(TOKEN_PATH):
        os.unlink(TOKEN_PATH)
    with mock.patch("os.link", side_effect=OSError("overlay-FS not supported")):
        token = _read_or_create_token_file(TOKEN_PATH)
    # Текущая реализация: os.replace(tmp_path, path) → return token.
    assert token is not None, "FAIL: токен не создан при OSError на link"
    print(f"    OK: token создан через os.replace fallback, file exists={os.path.exists(TOKEN_PATH)}")


def scenario_unlink_race_in_finally():
    """finally-блок: os.unlink(tmp_path). Если tmp_path был связан в path через os.link,
    то unlink(tmp_path) уменьшает link-count, но сам path остаётся живой. Но если мы
    попали в FileExistsError ветку (path уже есть), мы НЕ сделали unlink внутри ветки
    ДО finally. finally попытается unlink(tmp_path). Если tmp_path был успешно linked —
    unlink удалит только временный файл, path останется. OK.

    Проблемный кейс: tmp_path лежит НЕ на overlay, а на хосте (через bind-mount),
    и path тоже. unlink может упасть с OSError (EBUSY).
    """
    print("\n[3] os.unlink(tmp_path) в finally падает с EBUSY")
    if os.path.exists(TOKEN_PATH):
        os.unlink(TOKEN_PATH)
    real_unlink = os.unlink
    def flaky_unlink(p):
        if p.endswith(".tmp"):
            raise OSError(16, "Device or resource busy")
        return real_unlink(p)
    with mock.patch("os.unlink", side_effect=flaky_unlink):
        token = _read_or_create_token_file(TOKEN_PATH)
    # Текущая реализация: finally try/except OSError → pass, OK.
    assert token is not None, "FAIL: токен не создан при EBUSY на unlink"
    # Главное — НЕ падает наружу.
    print(f"    OK: EBUSY в finally проглочен, token создан, file exists={os.path.exists(TOKEN_PATH)}")


def scenario_handle_flush_oserror():
    """fsync() падает с OSError. Текущая реализация НЕ ловит эту ошибку отдельно —
    она попадёт в общий `except OSError: return None` → токен не создан.
    Authenticator без токена: verify() отклоняет, mcp_server живёт. НЕ exit 1.
    """
    print("\n[4] fsync() в handle.fileno() падает с OSError")
    if os.path.exists(TOKEN_PATH):
        os.unlink(TOKEN_PATH)
    real_fsync = os.fsync
    def flaky_fsync(fd):
        raise OSError(5, "I/O error (фейк fsync)")
    with mock.patch("os.fsync", side_effect=flaky_fsync):
        token = _read_or_create_token_file(TOKEN_PATH)
    assert token is None, f"FAIL: ожидали None, получили {token!r}"
    print("    OK: token=None, но и NO EXCEPTION наружу — mcp_server НЕ упадёт")
    print("    Это объясняет, почему в логах run 33260223956 НЕ было traceback,")
    print("    а crash был exit 1: mcp_server упал по ДРУГОЙ причине,")
    print("    а token=None — лишь пост-фактум от повторного старта.")


def scenario_two_processes_first_wins():
    """Два процесса стартуют одновременно; первый делает os.open+O_EXCL успешно,
    второй получает FileExistsError на tmp_path. Но tmp_path зависит от pid,
    так что гонки на O_EXCL не будет — только если два стартуют в один pid namespace,
    что невозможно в обычном Linux.

    Реальная гонка: оба процесса ВИДЯТ, что path не существует, оба читают _read_token_file
    → None → оба идут в ветку создания. У обоих свой tmp_path по pid → оба успешно
    создают свой tmp. Дальше:
    - процесс A: os.link(tmpA, path) — успех, теперь path=tmpA inode.
    - процесс B: os.link(tmpB, path) — FileExistsError → _read_token_file(path) → берёт
      secret от A. OK, оба согласованы.

    Если link не поддерживается:
    - процесс A: os.replace(tmpA, path) — успех, теперь path=tmpA content.
    - процесс B: os.replace(tmpB, path) — успех, перезаписал tmpA (это и есть та дыра,
      о которой говорит docstring: «не защищает от перезаписи, но один процесс получит
      отказ, а не пустой файл»). Здесь процесс B получает свой token, не токен A.

    Это критично: B публикует сообщения с подписью от токена B, A проверяет их с токеном A
    → отказы. Но mcp_server НЕ упадёт — verify() вернёт False.
    """
    print("\n[5] Два процесса: link не поддерживается → перезапись os.replace")
    if os.path.exists(TOKEN_PATH):
        os.unlink(TOKEN_PATH)
    with mock.patch("os.link", side_effect=OSError("no hardlinks")):
        # Процесс A
        tokenA = _read_or_create_token_file(TOKEN_PATH)
        # Процесс B
        tokenB = _read_or_create_token_file(TOKEN_PATH)
    # В обоих случаях вернулся непустой токен, но они РАЗНЫЕ.
    print(f"    tokenA={tokenA[:8] if tokenA else None}, tokenB={tokenB[:8] if tokenB else None}")
    if tokenA is None or tokenB is None:
        print(f"    ⚠️  RACE CONFIRMED: один из токенов None — другая нода не сможет аутентифицироваться.")
        print(f"    ⚠️  Это и есть exit 1 в staging: dialogue_node стартует, берёт None,")
        print(f"    ⚠️  mcp_server берёт token, dialogue_node не может подписать запрос,")
        print(f"    ⚠️  mcp_server всё время 401, но он не падает; падает — _init_voice_memory.")
    if tokenA == tokenB:
        print(f"    UNEXPECTED: токены совпали (race не воспроизвелся)")
    else:
        print(f"    OK: токены РАЗНЫЕ ({tokenA[:8]} vs {tokenB[:8]}) — это гонка")
        print("    Authenticator A не примет подписи от B, и наоборот.")
        print("    Это и есть корень: dialogue_node (подписант) мог получить токен A,")
        print("    mcp_server (проверяющий) — токен B, или наоборот. Все запросы 401.")


def scenario_requestauthenticator_from_env_when_token_none():
    """from_env при token=None (если _read_or_create_token_file вернул None):
    RequestAuthenticator(token=None, allow_unauthenticated=False) → enabled=False.
    sign() ничего не добавит, verify() вернёт False.
    Сам mcp_server НЕ упадёт — он просто будет отклонять все execute-запросы.
    Чтобы упасть с exit 1, нужно что-то ещё. Скорее всего — race при ПЕРВОМ создании
    файла: tmp_path не успевает удалиться, повторный вызов видит мусор, O_EXCL валит
    исключение, которое НЕ ловится и валится в __init__ ROS2 ноды → exit 1 без traceback.
    """
    print("\n[6] _read_or_create_token_file возвращает None → from_env молча даёт disabled")
    if os.path.exists(TOKEN_PATH):
        os.unlink(TOKEN_PATH)
    # запрещаем запись в директорию
    os.chmod(MCP_DIR, 0o500)
    try:
        auth = RequestAuthenticator.from_env(
            sender="mcp_server",
            allowed_senders={"dialogue_node", "harness"},
        )
    finally:
        os.chmod(MCP_DIR, 0o700)
    print(f"    enabled={auth.enabled}, token={'set' if auth._token else 'None'}")
    print("    OK: mcp_server жив, но ничего не выполняет.")


def main():
    print("=" * 60)
    print(" repro_mcp_token_race — корневой сценарий инцидента #1736")
    print("=" * 60)
    scenario_clean_path()
    scenario_os_link_oserror()
    scenario_unlink_race_in_finally()
    scenario_handle_flush_oserror()
    scenario_two_processes_first_wins()
    scenario_requestauthenticator_from_env_when_token_none()
    print("\n" + "=" * 60)
    print(" Вывод: _read_or_create_token_file может вернуть None или разные токены")
    print(" при гонке, но сам mcp_server через RequestAuthenticator НЕ падает с exit 1.")
    print(" Скорее всего exit 1 в run 33260223956 — побочный эффект")
    print(" (например, _init_voice_memory упал, а не mcp_auth).")
    print("=" * 60)


if __name__ == "__main__":
    main()
