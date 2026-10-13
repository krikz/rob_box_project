"""Клиент KIE (Suno) — реализация ``MusicGenProvider`` (ADR-0158 §2.1, §3.5).

Факты API (замеры 13.10, ADR-0158 §1.3):
  * ``POST /api/v1/jobs/createTask`` ``{"model": ..., "input": {...}}``;
    ``GET /api/v1/jobs/recordInfo?taskId=`` — состояния waiting/queuing/
    generating/success/fail; ``GET /api/v1/chat/credit`` — баланс;
    загрузка файла — ``POST https://kieai.redpandaai.co/api/file-stream-upload``
    (файл живёт 24 ч).
  * **Ошибки KIE приходят как HTTP 200 + ``code`` в теле** — смотрим ``code``.
  * CDN ``audiostream.kie.ai`` отдаёт 403 на User-Agent ``Python-urllib`` —
    всегда ставим свой UA.
  * Медиа результата живут 14 дней — ``download`` вызывать сразу.

Ключ — ``KIE_API`` из окружения процесса (не пишется в лог и в ``repr``).
Транспорт инъектируется (тесты — на фейковом HTTP, без сети и без денег).
"""

from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
import uuid
from typing import Any, Callable, Dict, Mapping, Optional, Tuple

from rob_box_music.knowledge import GEN_MODELS

from .provider import (
    DEFAULT_MODEL,
    GenJob,
    GenStatus,
    GenTicket,
    GenTrack,
    GenUnavailable,
)

API_BASE = "https://api.kie.ai"
UPLOAD_URL = "https://kieai.redpandaai.co/api/file-stream-upload"
KEY_ENV = "KIE_API"
USER_AGENT = "rob-box-music/1.0 (+https://github.com/krikz/rob_box_project)"

#: kind → модель KIE (поле ``model`` верхнего уровня createTask).
_KIND_MODEL = {
    "instrumental": "ai-music-api/generate",
    "robot_song": "ai-music-api/add-instrumental",
    "cover": "ai-music-api/upload-and-cover-audio",
}

RATE_LIMIT_RETRY_S = 10.0

# (status, body) — заголовки ответа нам не нужны.
Transport = Callable[[str, str, Mapping[str, str], Optional[bytes], float], Tuple[int, bytes]]


def urllib_transport(
    method: str, url: str, headers: Mapping[str, str], body: Optional[bytes], timeout: float,
) -> Tuple[int, bytes]:
    """Боевой транспорт. HTTP-ошибки возвращает как (status, body); сеть — ``GenUnavailable``."""
    req = urllib.request.Request(url, data=body, method=method, headers=dict(headers))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:  # noqa: S310 — https, фиксированные хосты
            return resp.status, resp.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise GenUnavailable("provider_down", f"сеть: {type(exc).__name__}") from exc


def reason_for_code(code: int) -> str:
    """Код KIE → причина (ADR-0158 §3.5)."""
    if code == 401:
        return "no_key"
    if code == 402:
        return "no_credits"
    if code == 429:
        return "rate_limited"
    if code in (400, 422):
        return "rejected"
    if code == 455 or code >= 500:
        return "provider_down"
    return "rejected" if 400 <= code < 500 else "provider_down"


def _result_items(rec: dict) -> list:
    """Список треков из ``resultJson`` (строка JSON) или из ``response``."""
    result = rec.get("resultJson")
    if isinstance(result, str) and result.strip():
        try:
            result = json.loads(result)
        except ValueError:
            result = {}
    if isinstance(result, dict) and result.get("data"):
        return result["data"]
    response = rec.get("response")
    return (response.get("data") if isinstance(response, dict) else None) or []


def _track_from_item(variant: int, t: dict) -> GenTrack:
    return GenTrack(
        variant=variant, track_id=str(t.get("id") or ""),
        audio_url=str(t.get("audio_url") or ""), stream_url=str(t.get("stream_audio_url") or ""),
        duration_s=float(t.get("duration") or 0.0), title=str(t.get("title") or ""),
        tags=str(t.get("tags") or ""),
    )


def tracks_from_record(rec: dict) -> Tuple[GenTrack, ...]:
    """Треки из ``data`` ответа recordInfo (``resultJson`` или ``response``)."""
    return tuple(
        _track_from_item(i, t) for i, t in enumerate(_result_items(rec)) if isinstance(t, dict)
    )


class KieSunoProvider:
    name = "kie_suno"

    def __init__(
        self,
        api_key: Optional[str] = None,
        *,
        transport: Transport = urllib_transport,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.time,
        api_base: str = API_BASE,
        request_timeout_s: float = 60.0,
    ) -> None:
        self._key = api_key if api_key is not None else os.environ.get(KEY_ENV, "")
        self._transport = transport
        self._sleep = sleep
        self._clock = clock
        self._base = api_base.rstrip("/")
        self._timeout = request_timeout_s

    @property
    def configured(self) -> bool:
        """Ключ задан (сам ключ наружу не отдаётся)."""
        return bool(self._key.strip())

    def __repr__(self) -> str:  # ключ никогда не попадает в лог
        return f"KieSunoProvider(configured={bool(self._key.strip())})"

    # ── HTTP ──────────────────────────────────────────────────────────────

    def _headers(self, extra: Optional[Mapping[str, str]] = None) -> Dict[str, str]:
        headers = {"Authorization": f"Bearer {self._key.strip()}", "User-Agent": USER_AGENT}
        headers.update(extra or {})
        return headers

    def _require_key(self) -> None:
        if not self._key.strip():
            raise GenUnavailable("no_key", f"{KEY_ENV} не задан")

    def _api(self, method: str, url: str, payload: Optional[dict] = None) -> dict:
        """Запрос к KIE. ``code`` ≠ 200 (в теле или HTTP-статусом) → ``GenUnavailable``."""
        self._require_key()
        body = json.dumps(payload).encode("utf-8") if payload is not None else None
        headers = self._headers({"Content-Type": "application/json"} if payload is not None else None)
        for attempt in (0, 1):
            status, raw = self._transport(method, url, headers, body, self._timeout)
            data = self._parse(raw)
            code = self._code(data, status)
            if code == 200:
                return data
            reason = reason_for_code(code)
            if reason == "rate_limited" and attempt == 0:
                self._sleep(RATE_LIMIT_RETRY_S)  # 429: один повтор через 10 с
                continue
            msg = str(data.get("msg", "")) if isinstance(data, dict) else ""
            raise GenUnavailable(reason, f"KIE code={code} {msg}".strip())
        raise AssertionError("unreachable")  # pragma: no cover

    @staticmethod
    def _code(data: Any, status: int) -> int:
        """``code`` из тела (KIE: HTTP всегда 200), иначе HTTP-статус."""
        if isinstance(data, dict) and "code" in data:
            try:
                return int(data["code"])
            except (TypeError, ValueError):
                return 500
        return status

    @staticmethod
    def _parse(raw: bytes) -> Any:
        try:
            return json.loads(raw.decode("utf-8", "replace"))
        except ValueError:
            return {}

    # ── MusicGenProvider ──────────────────────────────────────────────────

    def cost(self, job: GenJob) -> float:
        info = GEN_MODELS.get(job.model) or GEN_MODELS[DEFAULT_MODEL]
        return info.credits_per_task

    def balance(self) -> Optional[float]:
        """Остаток кредитов; ``None`` — неизвестно (нет ключа, сеть, формат)."""
        try:
            data = self._api("GET", f"{self._base}/api/v1/chat/credit")
        except GenUnavailable:
            return None
        value = data.get("data")
        if isinstance(value, dict):
            value = value.get("credits", value.get("balance"))
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def submit(self, job: GenJob) -> GenTicket:
        self._require_key()
        model = _KIND_MODEL.get(job.kind)
        if model is None:
            raise GenUnavailable("rejected", f"kind {job.kind!r} не поддержан (песня — ADR-0158 PR-7)")
        if job.model not in GEN_MODELS:
            raise GenUnavailable("rejected", f"неизвестная модель {job.model!r}")
        upload_url = ""
        if job.kind in ("robot_song", "cover"):
            if not job.audio_path:
                raise GenUnavailable("rejected", f"{job.kind}: нужен audio_path")
            upload_url = self.upload(job.audio_path)
        payload = {"model": model, "input": self._input_for(job, upload_url)}
        data = self._api("POST", f"{self._base}/api/v1/jobs/createTask", payload)
        task_id = str((data.get("data") or {}).get("taskId") or "")
        if not task_id:
            raise GenUnavailable("provider_down", "createTask без taskId")
        return GenTicket(task_id, self.name, job, self._clock(), self.cost(job))

    @staticmethod
    def _input_for(job: GenJob, upload_url: str) -> dict:
        if job.kind == "instrumental":
            return {
                "custom_mode": True, "instrumental": True, "model": job.model,
                "title": job.title, "style": job.style_tags, "duration": int(job.duration_s),
                **({"negative_tags": job.negative_tags} if job.negative_tags else {}),
            }
        if job.kind == "robot_song":
            body = {
                "upload_url": upload_url, "title": job.title, "model": job.model,
                "tags": job.style_tags, "lyrics": job.lyrics,
            }
            if job.negative_tags:
                body["negative_tags"] = job.negative_tags
            if job.vocal_gender:
                body["vocal_gender"] = job.vocal_gender
            if job.audio_weight is not None:
                body["audio_weight"] = job.audio_weight
            return body
        # cover (форма проверена живой пробой 13.10, b7.py)
        body = {
            "upload_url": upload_url, "instrumental": True, "model": job.model,
            "title": job.title, "duration": int(job.duration_s), "style": job.style_tags,
        }
        if job.negative_tags:
            body["negative_tags"] = job.negative_tags
        if job.audio_weight is not None:
            body["audio_weight"] = job.audio_weight
        return body

    def poll(self, ticket: GenTicket) -> GenStatus:
        data = self._api("GET", f"{self._base}/api/v1/jobs/recordInfo?taskId={ticket.task_id}")
        rec = data.get("data") or {}
        state = str(rec.get("state") or "").lower()
        tracks = tracks_from_record(rec)
        credits = rec.get("creditsConsumed")
        credits_f = float(credits) if isinstance(credits, (int, float)) else None
        cost_time = rec.get("costTime")
        base = dict(
            credits=credits_f,
            cost_time_s=float(cost_time) if isinstance(cost_time, (int, float)) else None,
            raw=rec,
        )
        if state == "success":
            if not tracks:
                raise GenUnavailable("provider_down", "success без треков", ticket)
            return GenStatus("complete", tracks, **base)
        if state == "fail":
            return GenStatus("failed", (), fail_msg=str(rec.get("failMsg") or rec.get("failCode") or ""), **base)
        if any(t.audio_url for t in tracks):
            return GenStatus("first", tracks, **base)
        if any(t.stream_url for t in tracks):
            return GenStatus("text", tracks, **base)
        return GenStatus("queued", (), **base)

    def download(self, url: str) -> bytes:
        status, raw = self._transport("GET", url, {"User-Agent": USER_AGENT}, None, self._timeout * 2)
        if status != 200 or not raw:
            raise GenUnavailable("provider_down", f"скачивание: HTTP {status}, {len(raw)} байт")
        return raw

    def upload(self, path: str) -> str:
        """Загрузить файл в KIE (живёт 24 ч), вернуть ``downloadUrl``."""
        self._require_key()
        with open(path, "rb") as fh:
            blob = fh.read()
        name = os.path.basename(path)
        boundary = uuid.uuid4().hex
        ctype = "audio/wav" if name.lower().endswith(".wav") else "audio/mpeg"
        parts = [
            f'--{boundary}\r\nContent-Disposition: form-data; name="uploadPath"\r\n\r\nrobbox/gen\r\n'.encode(),
            (f'--{boundary}\r\nContent-Disposition: form-data; name="file"; filename="{name}"\r\n'
             f"Content-Type: {ctype}\r\n\r\n").encode(),
            blob,
            f"\r\n--{boundary}--\r\n".encode(),
        ]
        headers = self._headers({"Content-Type": f"multipart/form-data; boundary={boundary}"})
        status, raw = self._transport("POST", UPLOAD_URL, headers, b"".join(parts), self._timeout * 3)
        data = self._parse(raw)
        code = self._code(data, status)
        if code != 200:
            raise GenUnavailable(reason_for_code(code), f"upload code={code}")
        url = str(((data.get("data") or {}) if isinstance(data, dict) else {}).get("downloadUrl") or "")
        if not url:
            raise GenUnavailable("provider_down", "upload без downloadUrl")
        return url
