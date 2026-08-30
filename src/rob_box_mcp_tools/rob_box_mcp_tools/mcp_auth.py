#!/usr/bin/env python3
"""mcp_auth.py — аутентификация отправителя для топика ``/mcp/execute``.

Почему это нужно
----------------
``/mcp/execute`` — это ``std_msgs/String`` с JSON-ом, и до этого модуля
единственной проверкой перед ``registry.execute(...)`` был FSM-гард режима
картографирования. То есть **любой** ROS2/Zenoh-пир на графе (скомпро-
метированный соседний нод, неправильно настроенный discovery-пир, или
процесс, уже получивший исполнение через другую дыру) мог опубликовать
tool-call напрямую и выполнить его — в обход LLM, в обход confirmation
gate, в обход всего слоя подтверждений.

ROS2 на транспортном уровне не даёт identity отправителя, поэтому
подлинность подтверждается HMAC-подписью на общем секрете:

* Секрет **никогда не ходит по топику** — только подпись. Это важно
  именно здесь: подписаться на ``/mcp/execute`` может кто угодно, так что
  простой токен в сообщении подслушивался бы и переигрывался.
* В подпись входит ``request_id`` и метка времени, поэтому чужое
  сообщение нельзя пересобрать с другими параметрами, а своё — переиграть
  позже (окно ``clock_skew_s`` + кэш уже виденных ``request_id``).
* ``sender`` тоже подписан: он не даёт криптографической гарантии сверх
  владения секретом (все легитимные ноды делят один), но позволяет
  явно перечислить, кому вообще можно звать тулы, и писать это в логи.

Секрет по умолчанию
-------------------
``dialogue_node`` и ``mcp_server`` живут в одном контейнере (см.
``docker/vision/docker-compose.yaml``), поэтому общий секрет берётся из
файла на локальной ФС: первый стартовавший процесс создаёт его атомарно,
остальные читают. Ничего не нужно добавлять в ``.env.secrets``, а пир на
другом хосте прочитать этот файл не может. Явный ``ROB_BOX_MCP_TOKEN``
в окружении переопределяет файл — для развёрток, где ноды разъезжаются
по разным контейнерам.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import logging
import os
import secrets
import tempfile
import threading
import time
from collections import OrderedDict
from typing import Any, Dict, Optional, Tuple

#: Явный секрет в окружении. Переопределяет файловый.
ENV_TOKEN = "ROB_BOX_MCP_TOKEN"

#: Путь к файлу с общим секретом (создаётся при первом обращении).
ENV_TOKEN_FILE = "ROB_BOX_MCP_TOKEN_FILE"

#: Аварийный выключатель. ``1`` — принимать неподписанные запросы.
#: Существует только чтобы можно было откатиться без пересборки образа;
#: включённым он возвращает ровно ту дыру, которую закрывает этот модуль,
#: поэтому включение логируется как ошибка на каждом запуске.
ENV_ALLOW_UNAUTH = "ROB_BOX_MCP_ALLOW_UNAUTHENTICATED"

#: ``/data`` — persistent-вольюм голосового контейнера (см. compose).
DEFAULT_TOKEN_PATH = "/data/.mcp_token"

#: Допустимый разбег часов между отправителем и mcp_server, секунды.
DEFAULT_CLOCK_SKEW_S = 30.0

#: Сколько ``request_id`` помним для защиты от повтора. С запасом
#: перекрывает окно ``clock_skew_s`` при любом реальном темпе tool-call-ов.
_REPLAY_CACHE_SIZE = 1024

#: Кто имеет право звать тулы. Совпадает с набором нодов, которые
#: действительно публикуют в ``/mcp/execute`` (см. llm_adapter.py,
#: async_executor.py).
DEFAULT_ALLOWED_SENDERS = frozenset({"dialogue_node", "harness"})


class RequestAuthenticator:
    """Подписывает и проверяет запросы ``/mcp/execute``.

    Потокобезопасен: ``mcp_server`` крутится на ``MultiThreadedExecutor``,
    так что ``verify()`` может вызываться из нескольких потоков сразу.

    Attributes:
        enabled: ``False`` если аутентификация выключена (нет секрета и
            выставлен аварийный выключатель) — тогда ``verify()``
            пропускает всё.
    """

    def __init__(
        self,
        token: Optional[str],
        *,
        sender: str = "unknown",
        allowed_senders=DEFAULT_ALLOWED_SENDERS,
        clock_skew_s: float = DEFAULT_CLOCK_SKEW_S,
        allow_unauthenticated: bool = False,
    ) -> None:
        """
        Args:
            token: Общий секрет. ``None`` — секрет недоступен.
            sender: Имя, которым подписываемся в :meth:`sign`.
            allowed_senders: Кого принимает :meth:`verify`.
            clock_skew_s: Допустимый возраст запроса, секунды.
            allow_unauthenticated: Пропускать неподписанные запросы,
                когда секрета нет. Без этого флага отсутствие секрета
                означает, что не проходит ничего.
        """
        self._token = token
        self._sender = sender
        self._allowed_senders = frozenset(allowed_senders)
        self._clock_skew_s = float(clock_skew_s)
        self._allow_unauthenticated = bool(allow_unauthenticated)
        self._seen: "OrderedDict[str, float]" = OrderedDict()
        self._lock = threading.Lock()

    @property
    def enabled(self) -> bool:
        """``True`` если запросы реально проверяются."""
        return self._token is not None

    @property
    def can_sign(self) -> bool:
        """``True`` если :meth:`sign` добавит подпись."""
        return self._token is not None

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    @classmethod
    def from_env(
        cls,
        *,
        sender: str = "unknown",
        allowed_senders=DEFAULT_ALLOWED_SENDERS,
        logger=None,
    ) -> "RequestAuthenticator":
        """Собрать аутентификатор из окружения.

        Порядок поиска секрета: ``ROB_BOX_MCP_TOKEN`` → файл
        ``ROB_BOX_MCP_TOKEN_FILE`` (по умолчанию :data:`DEFAULT_TOKEN_PATH`,
        создаётся при первом обращении).

        Args:
            sender: Имя ноды для :meth:`sign`.
            allowed_senders: Кого принимает :meth:`verify`.
            logger: Опциональный ROS-логгер для диагностики.

        Returns:
            Готовый :class:`RequestAuthenticator`.
        """
        allow_unauth = os.getenv(ENV_ALLOW_UNAUTH, "").strip() in ("1", "true", "True")

        token = os.getenv(ENV_TOKEN, "").strip() or None
        source = "env"
        if token is None:
            path = os.getenv(ENV_TOKEN_FILE, "").strip() or DEFAULT_TOKEN_PATH
            token = _read_or_create_token_file(path, logger=logger)
            source = f"file {path}"

        if logger is not None:
            if token is not None:
                logger.info(f"🔐 /mcp/execute: аутентификация включена (секрет: {source})")
            elif allow_unauth:
                logger.error(
                    f"🔓 /mcp/execute: аутентификация ОТКЛЮЧЕНА ({ENV_ALLOW_UNAUTH}=1) — "
                    "любой пир на ROS2/Zenoh-графе может выполнять инструменты"
                )
            else:
                logger.error(
                    "🔐 /mcp/execute: секрет недоступен — запросы будут отклоняться. "
                    f"Задай {ENV_TOKEN} или дай доступ на запись к "
                    f"{os.getenv(ENV_TOKEN_FILE) or DEFAULT_TOKEN_PATH}"
                )

        return cls(
            token,
            sender=sender,
            allowed_senders=allowed_senders,
            allow_unauthenticated=allow_unauth,
        )

    # ------------------------------------------------------------------
    # Signing / verification
    # ------------------------------------------------------------------

    def sign(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """Добавить к запросу блок ``auth``.

        Мутирует и возвращает тот же dict — вызывающий код формирует
        запрос прямо перед ``json.dumps``.

        Args:
            request: Запрос с ``tool_name`` / ``parameters`` / ``request_id``.

        Returns:
            Тот же dict; с ``auth``, если секрет доступен.
        """
        if self._token is None:
            return request
        ts = time.time()
        request["auth"] = {
            "sender": self._sender,
            "ts": ts,
            "sig": self._signature(request, sender=self._sender, ts=ts),
        }
        return request

    def verify(self, request: Dict[str, Any]) -> Tuple[bool, str]:
        """Проверить подпись входящего запроса.

        Args:
            request: Распарсенный JSON из ``/mcp/execute``.

        Returns:
            ``(is_authentic, error_message)`` — ``(True, "")`` если запрос
            принят.
        """
        if self._token is None:
            if self._allow_unauthenticated:
                return True, ""
            return False, "аутентификация недоступна: общий секрет не сконфигурирован"

        auth = request.get("auth")
        if not isinstance(auth, dict):
            return False, "запрос без подписи"

        sender = auth.get("sender")
        if sender not in self._allowed_senders:
            return False, f"отправитель '{sender}' не в списке разрешённых"

        ts = auth.get("ts")
        if not isinstance(ts, (int, float)) or isinstance(ts, bool):
            return False, "некорректная метка времени в подписи"
        age = time.time() - float(ts)
        if abs(age) > self._clock_skew_s:
            return False, f"запрос устарел или из будущего ({age:.1f}s)"

        sig = auth.get("sig")
        if not isinstance(sig, str):
            return False, "некорректная подпись"
        expected = self._signature(request, sender=sender, ts=float(ts))
        if not hmac.compare_digest(sig, expected):
            return False, "подпись не совпадает"

        # Подпись валидна — но точно то же сообщение можно переиграть,
        # пока не истекло окно clock_skew. request_id уникален на запрос,
        # так что повтор ловится по нему.
        request_id = request.get("request_id") or ""
        if request_id:
            with self._lock:
                if request_id in self._seen:
                    return False, f"повторный request_id '{request_id[:8]}'"
                self._seen[request_id] = time.time()
                while len(self._seen) > _REPLAY_CACHE_SIZE:
                    self._seen.popitem(last=False)

        return True, ""

    def _signature(self, request: Dict[str, Any], *, sender: Any, ts: float) -> str:
        """HMAC-SHA256 над канонической формой запроса.

        Args:
            request: Запрос (читаются ``tool_name`` / ``parameters`` /
                ``request_id``; сам блок ``auth`` в подпись не входит).
            sender: Имя отправителя.
            ts: Метка времени.

        Returns:
            Подпись в hex.
        """
        payload = json.dumps(
            {
                "tool_name": request.get("tool_name"),
                "parameters": request.get("parameters", {}),
                "request_id": request.get("request_id", ""),
                "sender": sender,
                "ts": ts,
            },
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            default=str,
        )
        return hmac.new(
            self._token.encode("utf-8"), payload.encode("utf-8"), hashlib.sha256
        ).hexdigest()


# ---------------------------------------------------------------------------
# Shared-secret file
# ---------------------------------------------------------------------------


def _read_or_create_token_file(
    path: str,
    *,
    logger: Optional[Any] = None,
) -> Optional[str]:
    """Прочитать общий секрет, создав его при первом обращении.

    Создание атомарно. Используется ``tempfile.NamedTemporaryFile`` в
    той же директории, что и целевой файл (это гарантирует, что
    ``os.replace`` будет атомарен — на одной ФС), и ``os.fsync`` ПЕРЕД
    переименованием, чтобы содержимое действительно попало на диск до
    того, как новый процесс увидит ``path``. ``O_EXCL`` + ``NamedTemporaryFile``
    гарантирует, что параллельные процессы не перетрут чужой секрет.

    Issue #1736 (29.08.2026, staging deploy run 33260223956): на overlay-FS
    Docker ``os.link`` мог упасть с OSError, fallback ``os.replace``
    при гонке двух процессов возвращал ``None`` второму — оба теряли
    общий секрет, и dialogue_node/mcp_server расходились по токенам.
    Теперь:
      - каждый шаг логируется (если передан ``logger``),
      - на любую ``OSError`` кроме уже-существующего-файла возвращается
        ``None`` И логируется — mcp_server должен это видеть, а не молча
        стартовать с пустым секретом,
      - при гонке второй процесс перечитывает созданный первым секрет,
        не перезаписывая его.

    Args:
        path: Путь к файлу секрета.
        logger: Опциональный ROS-логгер (или любой объект с методами
            ``info``/``warning``/``error``). Если ``None``, логирование
            идёт в stdlib ``logging``.

    Returns:
        Секрет, либо ``None`` если файл недоступен и создать его нельзя.
    """
    log = logger or logging.getLogger(__name__)
    existing = _read_token_file(path, logger=log)
    if existing is not None:
        log.info(f"🔐 mcp_auth: секрет прочитан из {path}")
        _check_token_file_mode(path, logger=log)
        return existing

    token = secrets.token_hex(32)
    parent = os.path.dirname(path) or "."
    try:
        os.makedirs(parent, exist_ok=True)
    except OSError as exc:
        log.error(f"❌ mcp_auth: не удалось создать директорию {parent}: {exc}")
        return None

    try:
        # ``NamedTemporaryFile(delete=False)`` → мы сами управляем жизненным
        # циклом tmp-файла и удаляем его в finally. ``dir=parent`` гарантирует,
        # что ``os.replace`` будет атомарен (на той же ФС).
        with tempfile.NamedTemporaryFile(
            mode="w",
            dir=parent,
            prefix=".mcp_token.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            tmp_path = handle.name
            try:
                handle.write(token)
                handle.flush()
                os.fsync(handle.fileno())
            except BaseException:
                _safe_unlink(tmp_path, logger=log)
                raise
        log.info(f"🔐 mcp_auth: секрет сгенерирован, tmp={tmp_path}")
    except OSError as exc:
        log.error(f"❌ mcp_auth: не удалось создать tmp-файл секрета: {exc}")
        return None

    try:
        # os.replace атомарен в пределах одной ФС; NamedTemporaryFile
        # создаёт файл в parent, так что tmp_path и path на одной ФС.
        # Если параллельный процесс уже создал path — FileExistsError
        # для os.replace НЕ возникает (replace перезаписывает), поэтому
        # сначала проверим существование, и только потом replace.
        if os.path.exists(path):
            # Гонку выиграл другой процесс — берём его секрет.
            log.warning(
                f"⚠️ mcp_auth: {path} появился во время записи — "
                "берём чужой секрет вместо своего"
            )
            other = _read_token_file(path, logger=log)
            if other is not None:
                token = other
            # tmp_path оставляем на finally — там удалим.
        else:
            try:
                os.replace(tmp_path, path)
                log.info(f"🔐 mcp_auth: секрет атомарно записан в {path} (mode 0600)")
            except OSError as exc:
                log.error(f"❌ mcp_auth: os.replace упал на {path}: {exc}")
                return None
    finally:
        _safe_unlink(tmp_path, logger=log)

    # Проверим права (0600), потому что mcp_server в проде запускается от root,
    # а мы хотим исключить случай, когда файл унаследовал маску 0644.
    _check_token_file_mode(path, logger=log)

    return token


def _check_token_file_mode(path: str, *, logger: Optional[Any] = None) -> None:
    """Предупредить, если права на файл шире, чем 0600.

    mcp_server в проде запускается от root, но если ``umask`` при создании
    был сбит (например, ``umask 022``), файл может оказаться ``-rw-r--r--``
    и тогда любой пользователь в системе прочитает общий секрет.

    Args:
        path: Путь к файлу секрета.
        logger: Опциональный ROS-логгер (или любой объект с методами
            ``info``/``warning``/``error``).
    """
    try:
        mode = stat_S_IMODE(os.stat(path).st_mode)
    except OSError as exc:
        if logger is not None:
            logger.warning(f"⚠️ mcp_auth: stat {path} упал: {exc}")
        return
    if mode & 0o077:
        log = logger or logging.getLogger(__name__)
        log.warning(
            f"⚠️ mcp_auth: {path} имеет mode={oct(mode)} — права шире, "
            "чем 0600. Секрет readable для группы/прочих."
        )


def _safe_unlink(path: str, *, logger: Optional[Any] = None) -> None:
    """Удалить файл, проглатывая ``FileNotFoundError`` и логируя ``OSError``.

    Отдельная функция, чтобы ``_read_or_create_token_file`` оставался
    читаемым: все четыре ветки финализации (``write failed``, ``replace
    won by other``, ``replace failed``, ``success``) проходят через неё.
    """
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass
    except OSError as exc:
        if logger is not None:
            logger.warning(f"⚠️ mcp_auth: не удалось удалить tmp {path}: {exc}")


def stat_S_IMODE(mode: int) -> int:
    """``stat.S_IMODE`` shim для Python 3.11+ где он всё ещё есть,
    но не помешает подстраховаться, если stdlib уберёт."""
    import stat as _stat
    return _stat.S_IMODE(mode)


def _read_token_file(
    path: str,
    *,
    logger: Optional[Any] = None,
) -> Optional[str]:
    """Прочитать секрет из файла.

    Args:
        path: Путь к файлу секрета.

    Returns:
        Секрет, либо ``None`` если файла нет / он пуст / нечитаем.
    """
    try:
        with open(path, "r", encoding="utf-8") as handle:
            token = handle.read().strip()
    except FileNotFoundError:
        return None
    except OSError as exc:
        if logger is not None:
            logger.error(f"❌ mcp_auth: чтение {path} упало: {exc}")
        return None
    return token or None
