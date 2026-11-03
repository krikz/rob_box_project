"""Адаптер Silero v5 (локальный, torch) на ``BaseTTSProvider`` — ADR-0160 §2.5, #4240.

Раньше загрузка модели и синтез жили методами ``tts_node`` (``_load_silero_model``, ``_synthesize_silero``). Здесь —
единственная реализация: ленивый импорт torch (#2609), модель только из ресурсного пака (ADR-0125, без сетевого
torch.hub), нарезка под лимит, retry-halve, конверт ``<speak>``.

Фоновый прогрев (поток, событие готовности) остаётся у ноды: это жизненный цикл ROS-процесса, а не сервиса.

Флаги сервиса (``capabilities``): SSML понимает; ударения ставит сам (``put_accent``); цифры НЕ читает
(нормализатор — PR-5, пока текст уходит как есть); паузы такта ``<break>`` адаптер пока не ставит (PR-4) — метки
MiniMax ``<#x#>`` снимаются, ``pause_markup=False`` честно.
"""

from __future__ import annotations

import asyncio
import threading
from typing import Any, AsyncIterator, Optional, Sequence

import numpy as np

from rob_box_dialog.plan.rap_pauses import text_for as _pause_text_for
from rob_box_llm.tts import TTSAudio, TTSChunk, TTSFormat, TTSSettings
from rob_box_llm.tts_provider_base import BaseTTSProvider, TTSCapabilities, TTSVoice

from ..tts_chunking import CHUNK_LIMITS, DEFAULT_MAX_RETRIES, MIN_CHUNK_CHARS, synthesize_with_retry
from ..tts_engines import SILERO_MODEL_PATHS, find_silero_model_path, load_silero_package, silero_apply_tts
from ..tts_ssml import silero_ssml


class SileroTTSProvider(BaseTTSProvider):
    """Silero v5: :meth:`load`, :meth:`prepare`, синхронный :meth:`synthesize_np` и async-контракт."""

    name = "silero"

    def __init__(
        self,
        *,
        logger: Any,
        sample_rate: int = 48000,
        default_speaker: str = "baya",
        put_accent: bool = True,
        put_yo: bool = True,
        put_stress_homo: bool = True,
        put_yo_homo: bool = True,
        chunk_max_chars: int = CHUNK_LIMITS["silero_v5"],
        chunk_max_retries: int = DEFAULT_MAX_RETRIES,
        chunk_min_chars: int = MIN_CHUNK_CHARS,
        model_paths: Sequence[str] = SILERO_MODEL_PATHS,
    ) -> None:
        self._logger = logger
        self.sample_rate = int(sample_rate)
        self._default_speaker = default_speaker
        self.put_accent = put_accent
        self.put_yo = put_yo
        self.put_stress_homo = put_stress_homo
        self.put_yo_homo = put_yo_homo
        self._chunk_max_chars = max(1, int(chunk_max_chars))
        self._chunk_max_retries = max(1, int(chunk_max_retries))
        self._chunk_min_chars = max(1, int(chunk_min_chars))
        self._model_paths = tuple(model_paths)
        self.model: Any = None
        #: ``torch.device`` создаётся при первой загрузке (#2609: без torch при старте ноды).
        self.device: Any = None
        self._loading = threading.Lock()

    # ---- модель ----

    def load(self) -> bool:
        """Загрузить модель (идемпотентно, параллельный вызов не грузит второй раз) → ``True``, если модель есть.

        Issue #2609 — torch импортируется ЗДЕСЬ, не на уровне модуля: основной провайдер ноды облачный, torch нужен
        только когда Silero реально звучит. Ошибка импорта torch пробрасывается (громко), ошибка модели — в лог и
        ``False``: нода честно сообщит отказ синтеза.
        """
        if self.model is not None:
            return True
        if not self._loading.acquire(blocking=False):
            self._logger.warn("⏳ Silero модель уже загружается...")
            return False
        try:
            self._logger.info("🔄 Загрузка Silero TTS v5...")
            try:
                import torch  # noqa: PLC0415 — lazy import by design (#2609)
            except ImportError as exc:
                self._logger.error(
                    f"❌ Issue #2609: torch import failed (Silero unavailable): {exc}. Verify that the "
                    "voice_assistant image installed the CPU-only wheel from https://download.pytorch.org/whl/cpu."
                )
                raise
            if self.device is None:
                self.device = torch.device("cpu")
            # ⚡ Настройки для ARM64 Pi.
            torch.set_num_threads(4)
            torch._C._jit_set_profiling_mode(False)
            torch.set_grad_enabled(False)
            try:
                self.model = self._load_from_resource_pack()
                self._logger.info("✅ Silero TTS v5 загружен (ARM64 оптимизация)")
            except Exception as e:  # noqa: BLE001 — модель недоступна: честный отказ синтеза выше
                self._logger.error(f"❌ Ошибка загрузки Silero: {e}")
                self.model = None
            return self.model is not None
        finally:
            self._loading.release()

    def _load_from_resource_pack(self) -> Any:
        """Модель только из ресурсного пака (ADR-0125); сетевого torch.hub нет намеренно (ADR-0018)."""
        model_path = find_silero_model_path(self._model_paths)
        if not model_path:
            raise FileNotFoundError(
                f"Silero v5 не найден ни в одном из путей: {list(self._model_paths)}. "
                "Модель доставляет Ресурсный пак (ADR-0125) в /opt/rob_box/models/silero/v5_ru.pt на хосте, "
                "откуда она приходит в контейнер bind-mount'ом /models. Проверь шаг деплоя «Ensure STT/TTS models» "
                "и вывод apply_resource_pack.sh --only silero-tts-v5-ru. Сетевой fallback (torch.hub) удалён намеренно."
            )
        self._logger.info(f"📦 Загрузка Silero v5: {model_path}")
        # Silero v5 — torch.package, не torch.jit.
        return load_silero_package(model_path, self.device)

    # ---- флаги и подготовка текста ----

    def capabilities(self) -> TTSCapabilities:
        """SSML и ударения — да; цифры — нет (нормализатор PR-5); метки пауз — пока нет (PR-4)."""
        return TTSCapabilities(ssml=True, audio_format_pcm=True, custom_endpoint=True, sets_stress=True)

    def prepare(self, text: str) -> str:
        """Метки пауз MiniMax ``<#x#>`` сняты. Конверт ``<speak>`` ставится на каждый чанк в :meth:`synthesize_np`:
        чанкер режет текст, а не XML."""
        return _pause_text_for(self.name, text)

    def _build_request_payload(
        self, text: str, settings: TTSSettings, voice_meta: Optional[TTSVoice]
    ) -> dict[str, Any]:
        """Аргументы ``model.apply_tts`` для одного чанка (конверт уже с экранированием)."""
        return {
            "ssml_text": silero_ssml(text),
            "speaker": settings.voice or self._default_speaker,
            "sample_rate": self.sample_rate,
            "put_accent": self.put_accent,
            "put_yo": self.put_yo,
            "put_stress_homo": self.put_stress_homo,
            "put_yo_homo": self.put_yo_homo,
        }

    # ---- синтез ----

    def synthesize_np(self, text: str, *, speaker: Optional[str] = None) -> np.ndarray:
        """Синтез → float32 mono (чанки склеены), retry-halve на «too long»/«couldn't generate» (#933)."""
        if self.model is None:
            raise Exception("Silero TTS model не инициализирована")
        settings = TTSSettings(voice=speaker or self._default_speaker)

        def synthesize_chunk(chunk: str) -> np.ndarray:
            payload = self._build_request_payload(chunk, settings, None)
            return silero_apply_tts(
                self.model,
                payload.pop("speaker"),
                payload.pop("sample_rate"),
                **payload,
            )

        segments = synthesize_with_retry(
            text,
            "silero_v5",
            synthesize_chunk,
            max_chars=self._chunk_max_chars,
            max_retries=self._chunk_max_retries,
            min_chunk_chars=self._chunk_min_chars,
            is_too_long=lambda exc: any(m in str(exc).lower() for m in ("length", "generate", "too long")),
        )
        if not segments:
            raise Exception("Silero TTS: ни один chunk не вернул аудио")
        return np.concatenate(segments) if len(segments) > 1 else segments[0]

    # ---- async-контракт TTSProvider ----

    async def synthesize(self, text: str, *, settings: TTSSettings | None = None) -> TTSAudio:
        """:meth:`prepare` + :meth:`synthesize_np` в пуле потоков → PCM int16."""
        settings = settings or TTSSettings()
        audio_np = await asyncio.to_thread(self.synthesize_np, self.prepare(text), speaker=settings.voice)
        pcm = (np.clip(audio_np, -1.0, 1.0) * 32767.0).astype("<i2").tobytes()
        return TTSAudio(samples=pcm, sample_rate=self.sample_rate, format=TTSFormat.PCM)

    async def stream(self, text: str, *, settings: TTSSettings | None = None) -> AsyncIterator[TTSChunk]:
        """Потока нет: один терминальный кусок со всем аудио."""
        audio = await self.synthesize(text, settings=settings)
        yield TTSChunk(samples=audio.samples, sample_rate=audio.sample_rate, finish_reason="stop")


__all__ = ["SileroTTSProvider"]
