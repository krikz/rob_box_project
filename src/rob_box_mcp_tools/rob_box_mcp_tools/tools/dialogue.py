#!/usr/bin/env python3
"""
dialogue.py - Инструменты для управления диалогом с пользователем

Инструменты:
- SpeakTextTool: Произнести текст голосом (TTS)
- ListenForResponseTool: Ждать ответ пользователя (STT)
"""

from __future__ import annotations
from typing import List, TYPE_CHECKING

# Ленивый импорт ROS 2 модулей для поддержки unit тестов
if TYPE_CHECKING:
    from std_msgs.msg import String

from ..base import MCPTool, MCPToolParameter, MCPToolResult


class SpeakTextTool(MCPTool):
    """Инструмент для произнесения текста голосом."""

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from std_msgs.msg import String

        # Publisher для TTS запросов
        self.tts_pub = node.create_publisher(String, "/voice/tts/request", 10)
        # Publisher для анимаций (инициализируем сразу, чтобы не создавать дубли в execute())
        self.animation_pub = node.create_publisher(String, "/voice/animation/request", 10)
        # Subscriber для получения завершения произношения
        self.finished_sub = node.create_subscription(String, "/voice/tts/finished", self._on_tts_finished, 10)
        # Subscriber для получения текущего dialogue_id от dialogue_node
        # (чтобы включать его в TTS запросы и tts_node мог отбрасывать старые)
        self._current_dialogue_id: str | None = None
        self._dialogue_id_sub = node.create_subscription(
            String, "/voice/current_dialogue_id", self._on_current_dialogue_id, 1
        )
        # Publisher для batch_complete — публикуется, когда последний чанк
        # батча получает /voice/tts/finished (issue #980). Используется
        # dialogue_node, чтобы не рубить музыку между чанками длинного
        # speak_text (рэп, стихи): cleanup ждёт batch_complete, а не debounce.
        self.batch_complete_pub = node.create_publisher(
            String, "/voice/tts/batch_complete", 10
        )
        # Issue #992: a separate "batch registered" prelude so dialogue_node
        # can learn about every in-flight batch_id BEFORE the first chunk
        # finishes. This is required for multi-speak_text cycles — when the
        # LLM fires two ``speak_text`` tool calls in one cycle, each call
        # creates its own ``batch_id`` and produces its own
        # ``/voice/tts/batch_complete``. Without the prelude, dialogue_node
        # only discovers a batch via the first ``tts_finished`` for it,
        # which means batch #1's ``batch_complete`` fires cleanup while
        # batch #2 is still pending. The prelude lets dialogue_node register
        # the batch up front so cleanup is held until the last batch
        # completes. Payload: ``{"batch_id": str, "chunks_total": int}``.
        self.batch_registered_pub = node.create_publisher(
            String, "/voice/tts/batch_registered", 10
        )
        # Трекер активных произношений: speech_id -> {batch_id, batch_total};
        # batch_total == 0 указывает на «одиночный» вызов (без split) —
        # в этом случае batch_complete публикуется после первого finished,
        # чтобы не сломать совместимость с уже существующими потребителями.
        self.pending_speeches: dict = {}
        # batch_id -> {"remaining": int, "total": int, "started_at_ms": float,
        #              "success": bool}; создаётся в execute(), обновляется
        # и закрывается в _on_tts_finished().
        self.pending_batches: dict = {}
        import threading
        self.pending_speeches_lock = threading.Lock()

    def _on_current_dialogue_id(self, msg: "String"):
        """Обновление текущего dialogue_id от dialogue_node."""
        self._current_dialogue_id = msg.data

    def _on_tts_finished(self, msg: "String"):
        """Обработка завершения произношения.

        1. Удаляет запись speech_id из pending_speeches.
        2. Декрементирует счётчик батча. Когда батч завершён (remaining==0),
           публикует /voice/tts/batch_complete с chunks_total и
           batch_duration_ms (issue #980). dialogue_node подписан на этот
           топик и публикует /mcp/music_cleanup ровно один раз после
           последнего чанка — музыка играет весь рэп, а не вырубается
           после 1-го чанка из 4.
        """
        import json
        try:
            result = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.log_error(f"❌ Ошибка парсинга TTS finished: {exc}")
            return
        speech_id = result.get("speech_id")
        finished_success = bool(result.get("success", False))
        self.log_info(
            f"🔔 TTS finished: speech_id="
            f"{speech_id[:8] if speech_id else 'None'}..., "
            f"success={finished_success}"
        )
        if not speech_id:
            return

        with self.pending_speeches_lock:
            entry = self.pending_speeches.pop(speech_id, None)
            if entry is None:
                self.log_warning(
                    f"⚠️ Speech {speech_id[:8]}... не найден в pending_speeches "
                    "(возможно уже удалён)"
                )
                return
            batch_id = entry["batch_id"]
            batch = self.pending_batches.get(batch_id)
            if batch is None:
                # Баланс между двумя dict'ами мог разъехаться только при
                # баге — отмечаем как warning, но не падаем.
                self.log_warning(
                    f"⚠️ Batch {batch_id[:8] if batch_id else 'None'}... не "
                    f"найден при finished для speech {speech_id[:8]}..."
                )
                return
            # Если чанк упал — батч помечаем как unsuccessful, чтобы
            # dialogue_node мог почистить ресурсы даже при сбое.
            if not finished_success:
                batch["success"] = False
            batch["remaining"] -= 1
            remaining = batch["remaining"]
            total = batch["total"]
            # Берём snapshot до pop(), чтобы не лезть в удалённый dict.
            batch_success = batch["success"]
            batch_started_at = batch["started_at_ms"]
            is_complete = (remaining == 0)
            if is_complete:
                # Удаляем батч, чтобы не рос в памяти.
                self.pending_batches.pop(batch_id, None)

        if is_complete:
            self._publish_batch_complete(
                batch_id=batch_id,
                chunks_total=total,
                success=batch_success,
                started_at_ms=batch_started_at,
            )
        else:
            self.log_info(
                f"⏳ Batch {batch_id[:8]}... "
                f"{total - remaining}/{total} чанков завершено"
            )

    @property
    def name(self) -> str:
        return "speak_text"

    @property
    def description(self) -> str:
        return (
            "Произнести текст голосом через TTS. "
            "ИСПОЛЬЗУЙ ЭТО вместо возврата JSON с SSML. "
            "ОБЯЗАТЕЛЬНО указывай animation - это покажет соответствующую анимацию на LED матрице робота (happy, sad, police_lights, и т.д.). "
            "Можешь вызвать несколько раз для разных фраз, делать паузы между ними через play_sound или play_animation."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="text",
                type="string",
                description="Текст для произнесения. Можно использовать русские ударения (+ после гласной).",
                required=True,
            ),
            MCPToolParameter(
                name="animation",
                type="string",
                description=(
                    "Анимация для отображения на LED матрице во время речи. "
                    "Выбирай подходящую анимацию для контекста (эмоциональные: happy, sad, angry, surprised; "
                    "специальные: police_lights, fire_truck, thinking, и т.д.). "
                    "Псевдонимы нормализуются: neutral→idle, excited→happy, confused→thinking, talk→talking. "
                    "Если указано неизвестное значение — будет warning в лог и анимация останется без изменений."
                ),
                required=False,
            ),
        ]

    # Максимальная длина чанка для Yandex TTS (лимит ~250 символов SSML)
    _MAX_CHUNK_CHARS = 200

    @staticmethod
    def _split_sentences(text: str, max_len: int = 200) -> list:
        """Разбивает текст на предложения не длиннее max_len символов.

        Сначала режет по знакам конца предложения (. ! ? ;), потом при
        необходимости разбивает слишком длинные куски по запятым/пробелам.
        """
        import re
        # Разбиваем по концу предложений, сохраняя разделители
        raw = re.split(r'(?<=[.!?;])\s+', text.strip())
        chunks: list = []
        buf = ""
        for part in raw:
            part = part.strip()
            if not part:
                continue
            candidate = (buf + " " + part).strip() if buf else part
            if len(candidate) <= max_len:
                buf = candidate
            else:
                if buf:
                    chunks.append(buf)
                # Если одно предложение само по себе длиннее лимита — бьём по запятым
                if len(part) > max_len:
                    sub_parts = re.split(r'(?<=,)\s+', part)
                    sub_buf = ""
                    for sp in sub_parts:
                        sub_candidate = (sub_buf + " " + sp).strip() if sub_buf else sp
                        if len(sub_candidate) <= max_len:
                            sub_buf = sub_candidate
                        else:
                            if sub_buf:
                                chunks.append(sub_buf)
                            # Последний резорт — режем по словам
                            words = sp.split()
                            word_buf = ""
                            for w in words:
                                wc = (word_buf + " " + w).strip() if word_buf else w
                                if len(wc) <= max_len:
                                    word_buf = wc
                                else:
                                    if word_buf:
                                        chunks.append(word_buf)
                                    word_buf = w
                            sub_buf = word_buf
                    buf = sub_buf
                else:
                    buf = part
        if buf:
            chunks.append(buf)
        return [c for c in chunks if c.strip()]

    def execute(self, text: str, animation: str = "neutral") -> MCPToolResult:
        """Произнести текст."""
        import json
        import re
        import uuid

        # 🔴 FIX (live 06.08): LLM (MiniMax-M3) иногда ВКЛЮЧАЕТ параметры
        # вызова В ТЕКСТ: text='...128 ударов!", animation="happy"'.
        # Вырезаем вшитый синтаксис: `", animation="..."` / `", voice="..."`
        # и хвост `")` / `)` — иначе TTS читает «анимейшин хеппи».
        text = re.sub(r'"\s*,\s*(?:animation|voice|rate|pitch)="[^"]*"\s*\)?\s*$', '', text)
        text = re.sub(r'\)\s*$', '', text) if text.endswith(')') else text
        text = text.strip()

        self.log_info(f"Произношение текста: {text[:50]}... (animation: {animation})")

        if not text:
            return MCPToolResult(success=False, error="Пустой текст", message="Текст не может быть пустым")

        # Нормализация анимаций (для обратной совместимости и маппинга несуществующих)
        animation_map = {
            # Русские названия
            "нейтрально": "idle",
            "нейтральная": "idle",
            "нейтральный": "idle",
            "радость": "happy",
            "радостный": "happy",
            "счастливый": "happy",
            "грустный": "sad",
            "грусть": "sad",
            "печаль": "sad",
            "злой": "angry",
            "злость": "angry",
            "возбужденный": "happy",
            "возбуждение": "happy",
            "смущенный": "thinking",
            "смущение": "thinking",
            "растерянный": "thinking",
            # Несуществующие анимации → замена на похожие
            "neutral": "idle",
            "excited": "happy",
            "confused": "thinking",
            "laughing": "happy",
            "smiling": "happy",
            "dancing": "excited",
            "singing": "happy",
            # LLM часто пишет "talk" вместо "talking"
            "talk": "talking",
        }
        # Множество реально существующих анимаций (без алиасов)
        _KNOWN_ANIMATIONS = {
            "idle", "talking", "wakeup", "sleep",
            "happy", "sad", "angry", "surprised", "thinking", "victory",
            "error", "low_battery", "charging",
            "police_lights", "ambulance", "fire_truck", "road_service",
            "turn_left", "turn_right", "accelerating", "braking",
        }
        animation = animation_map.get(animation.lower() if animation else "idle", animation) if animation else "idle"
        if animation not in _KNOWN_ANIMATIONS:
            self.log_warning(
                f"⚠️ Неизвестная анимация '{animation}' — "
                f"использую 'talking' (робот же говорит), текст будет произнесён"
            )
            animation = "talking"

        # Определяем pitch для голоса на основе анимации (только для эмоциональных)
        pitch_map = {
            "happy": "high",
            "sad": "low",
            "angry": "high",
            "excited": "x-high",
            "surprised": "high"
        }
        pitch = pitch_map.get(animation, None)

        def _make_ssml(chunk: str) -> str:
            if pitch:
                return f"<speak><prosody pitch='{pitch}'>{chunk}</prosody></speak>"
            return f"<speak>{chunk}</speak>"

        # Разбиваем текст на чанки ≤ _MAX_CHUNK_CHARS (лимит Yandex TTS)
        chunks = self._split_sentences(text, self._MAX_CHUNK_CHARS)
        if len(chunks) > 1:
            self.log_info(f"✂️ Текст разбит на {len(chunks)} чанков (длина: {len(text)} символов)")

        # Issue #980: один speak_text(...) может породить несколько чанков
        # TTS-запросов. dialogue_node должен знать, когда ВСЕ чанки этого
        # вызова закончили проигрываться, чтобы корректно отложить cleanup
        # музыки. Генерируем batch_id заранее, до публикации первого чанка,
        # и регистрируем каждый speech_id с этим batch_id в pending_speeches.
        import time as _time
        batch_id = str(uuid.uuid4())
        chunks_total = len(chunks)

        # Issue #992: announce the batch to dialogue_node BEFORE the first
        # TTS request lands. dialogue_node needs to know how many
        # ``batch_id``s are in flight so it can hold the music_cleanup
        # until the last ``batch_complete`` (the LLM may fire several
        # ``speak_text`` calls in one cycle, each with its own batch).
        try:
            from std_msgs.msg import String as _RegMsg
            _reg_msg = _RegMsg()
            _reg_msg.data = json.dumps(
                {"batch_id": batch_id, "chunks_total": chunks_total},
                ensure_ascii=False,
            )
            self.batch_registered_pub.publish(_reg_msg)
        except Exception as _exc:  # noqa: BLE001
            # Prelude is best-effort — without it dialogue_node falls back
            # to discovering the batch via the first tts_finished, which
            # is correct for single-speak_text cycles but breaks the
            # multi-speak_text case. Log loudly so operators notice.
            self.log_warning(
                f"⚠️ [issue 992] Не удалось опубликовать "
                f"/voice/tts/batch_registered: {_exc}"
            )

        from std_msgs.msg import String
        speech_ids = []
        for i, chunk in enumerate(chunks):
            speech_id = str(uuid.uuid4())
            speech_ids.append(speech_id)

            # Анимацию ставим только для первого чанка (чтобы не мигало)
            if i == 0 and animation and animation != "idle":
                try:
                    from std_msgs.msg import String as StringMsg
                    anim_msg = StringMsg()
                    anim_msg.data = animation
                    self.animation_pub.publish(anim_msg)
                    self.log_info(f"🎨 Установлена анимация '{animation}'")
                except Exception as e:
                    self.log_warning(f"⚠️ Не удалось установить анимацию: {e}")

            # Регистрируем ожидание: каждый speech_id знает свой batch_id
            # и chunks_total. Это нужно _on_tts_finished(), чтобы понять —
            # последний это чанк или промежуточный.
            with self.pending_speeches_lock:
                self.pending_speeches[speech_id] = {
                    "batch_id": batch_id,
                    "batch_total": chunks_total,
                }
                # Батч создаётся один раз — при обработке первого чанка.
                if batch_id not in self.pending_batches:
                    self.pending_batches[batch_id] = {
                        "remaining": chunks_total,
                        "total": chunks_total,
                        "started_at_ms": _time.monotonic() * 1000.0,
                        "success": True,
                    }

            tts_request = {
                "ssml": _make_ssml(chunk),
                "speech_id": speech_id,
                # batch_id публикуется в payload запроса — даже если tts_node
                # его игнорирует, мы не зависим от него: batch_id хранится
                # и в pending_speeches, и восстанавливается без участия
                # tts_node. Поле оставлено для трейсинга в логах tts_node.
                "batch_id": batch_id,
            }
            if self._current_dialogue_id:
                tts_request["dialogue_id"] = self._current_dialogue_id
            msg = String()
            msg.data = json.dumps(tts_request, ensure_ascii=False)
            self.tts_pub.publish(msg)
            self.log_info(
                f"📤 TTS чанк {i+1}/{len(chunks)}: {chunk[:40]!r}... "
                f"(speech_id: {speech_id[:8]}, batch_id: {batch_id[:8]}, "
                f"dialogue_id: {self._current_dialogue_id[:8] if self._current_dialogue_id else 'None'})"
            )

        self.log_info(
            f"✅ TTS запрос(ов) принято: {chunks_total} "
            f"(асинхронный режим, batch_id: {batch_id[:8]})"
        )
        return MCPToolResult(
            success=True,
            data={
                "text": text,
                "animation": animation,
                "chunks": chunks_total,
                "speech_ids": speech_ids,
                "batch_id": batch_id,
                "async": True,
            },
            message=(
                f"TTS запрос отправлен ({chunks_total} чанк(ов)): {text[:50]}..."
            ),
        )

    def _publish_batch_complete(
        self,
        *,
        batch_id: str,
        chunks_total: int,
        success: bool,
        started_at_ms: float,
    ) -> None:
        """Публикует /voice/tts/batch_complete (issue #980).

        Вызывается ровно один раз для каждого батча — когда последний
        ``tts/finished`` прилетел. ``batch_duration_ms`` — настенное
        время от первого чанка до последнего; полезно для диагностики
        раскладки рэпов (issue #949 уже использует duration_sec).
        """
        import json as _json
        import time as _time
        now_ms = _time.monotonic() * 1000.0
        batch_duration_ms = max(0.0, now_ms - started_at_ms)
        payload = {
            "batch_id": batch_id,
            "chunks_total": chunks_total,
            "batch_duration_ms": int(batch_duration_ms),
            "success": bool(success),
        }
        try:
            from std_msgs.msg import String as _StringMsg
            msg = _StringMsg()
            msg.data = _json.dumps(payload, ensure_ascii=False)
            self.batch_complete_pub.publish(msg)
            self.log_info(
                f"🎬 TTS batch_complete: batch_id={batch_id[:8]}... "
                f"chunks_total={chunks_total} "
                f"batch_duration_ms={int(batch_duration_ms)} "
                f"success={success}"
            )
        except Exception as exc:  # noqa: BLE001
            # Cleanup музыки зависит от batch_complete — не теряем ошибку.
            self.log_error(
                f"❌ Не удалось опубликовать /voice/tts/batch_complete: {exc}"
            )

class ListenForResponseTool(MCPTool):
    """Инструмент для ожидания ответа пользователя."""

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from std_msgs.msg import String

        # Publisher для запроса активации STT
        self.stt_request_pub = node.create_publisher(String, "/voice/stt/request", 10)

    @property
    def name(self) -> str:
        return "listen_for_response"

    @property
    def description(self) -> str:
        return (
            "Ждать ответ пользователя через речь. "
            "Робот активирует микрофон и будет ждать ответа (таймаут 30 секунд). "
            "ИСПОЛЬЗУЙ когда задал вопрос пользователю или ждёшь продолжения диалога."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="timeout_seconds",
                type="integer",
                description="Таймаут ожидания в секундах (по умолчанию 30)",
                required=False,
            ),
            MCPToolParameter(
                name="prompt_text",
                type="string",
                description="Текст-подсказка что ждёшь от пользователя (для логирования)",
                required=False,
            ),
        ]

    def execute(self, timeout_seconds: int = 30, prompt_text: str = "") -> MCPToolResult:
        """Активировать режим ожидания ответа."""
        self.log_info(f"Ожидание ответа пользователя (таймаут: {timeout_seconds}s)")

        if prompt_text:
            self.log_info(f"Подсказка: {prompt_text}")

        # Публикуем запрос на активацию STT
        from std_msgs.msg import String
        msg = String()
        msg.data = f"listen:{timeout_seconds}"
        self.stt_request_pub.publish(msg)

        self.log_info("STT активирован для ожидания ответа")

        return MCPToolResult(
            success=True,
            data={"timeout_seconds": timeout_seconds, "prompt_text": prompt_text},
            message=f"Жду ответ пользователя ({timeout_seconds}s таймаут)",
        )


class EstimateTtsDurationTool(MCPTool):
    """Estimate TTS playback duration for a given text (#949).

    Uses a calibrated chars-per-second constant for Russian TTS with
    the ROBBOX chipmunk 2x speedup effect.  Accurate enough for the
    LLM to plan music arrangements timed to rap/poem duration.
    """

    #: Calibrated Russian TTS speed with chipmunk 2x (chars/second).
    #: Derived from v20 log: 947 chars / 27.8 s chipmunk playback ≈ 34 cps.
    #: Using a conservative 30 cps to leave headroom for SSML overhead.
    DEFAULT_CHARS_PER_SECOND: float = 30.0

    def __init__(self, node) -> None:
        super().__init__(node)

    @property
    def name(self) -> str:
        return "estimate_tts_duration"

    @property
    def description(self) -> str:
        return (
            "Оценить длительность TTS-озвучки для заданного текста в секундах. "
            "Используется для планирования аранжировки музыки под длительность рэпа/стиха. "
            "Возвращает estimate_sec (float) — примерное время звучания с учётом chipmunk-ускорения."
        )

    @property
    def parameters(self) -> list:
        return [
            MCPToolParameter(
                name="text",
                type="string",
                description="Текст для оценки длительности озвучки.",
                required=True,
            ),
            MCPToolParameter(
                name="chars_per_second",
                type="number",
                description=(
                    "Скорость озвучки (символов/сек). По умолчанию 30 — "
                    "калиброванное значение для русского TTS с chipmunk 2x."
                ),
                required=False,
            ),
        ]

    @property
    def execution_type(self):
        from ..base import ToolExecutionType
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, text: str, chars_per_second: float | None = None) -> MCPToolResult:
        cps = float(chars_per_second) if chars_per_second is not None else self.DEFAULT_CHARS_PER_SECOND
        cps = max(1.0, cps)  # safety floor
        estimate_sec: float = round(len(text) / cps, 1)
        self.log_info(
            f"[estimate_tts_duration] text_len={len(text)}, cps={cps}, "
            f"estimate={estimate_sec}s"
        )
        return MCPToolResult(
            success=True,
            data={
                "estimate_sec": estimate_sec,
                "text_length": len(text),
                "chars_per_second": cps,
            },
            message=(
                f"Оценка длительности: ~{estimate_sec}с "
                f"({len(text)} символов при {cps} симв/с)"
            ),
        )


class RegisterSpeakerTool(MCPTool):
    """Issue #1077 + #1101 — регистрация голоса спикера через LLM tool.

    LLM вызывает этот tool когда:
    - Пользователь сказал «меня зовут X» (LLM извлёк имя из контекста)
    - LLM хочет спросить имя незнакомца (name=null → publish ask_event)

    Tool публикует в /voice/speaker/register JSON {"name": "..."}
    speaker_id_node получает d-vector текущей фразы и сохраняет embedding.
    """

    def __init__(self, node):
        super().__init__(node)
        from std_msgs.msg import String
        self._speaker_register_pub = node.create_publisher(
            String, "/voice/speaker/register", 10
        )
        # Issue #1101 — rename коррекции («я не X, я Y») публикуются на
        # ОТДЕЛЬНЫЙ топик /voice/speaker/rename: speaker_id_node слушает
        # rename на нём (_on_rename_request), а /voice/speaker/register
        # принимает только {"name": ...} и игнорирует {old_name,new_name}.
        self._speaker_rename_pub = node.create_publisher(
            String, "/voice/speaker/rename", 10
        )

    @property
    def name(self) -> str:
        return "register_speaker"

    @property
    def description(self) -> str:
        return (
            "Зарегистрировать голос текущего спикера в voice biometric DB. "
            "Вызывай когда: (1) пользователь представился («меня зовут X») — "
            "передай name=X, (2) хочешь узнать имя незнакомца — передай name=null "
            "и спроси «Как вас зовут?» через speak_text. "
            "Имя сохранится в БД вместе с эмбеддингом голоса (resemblyzer d-vector), "
            "после этого пользователя можно будет узнавать по голосу."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="name",
                type="string",
                description=(
                    "Имя спикера для регистрации (Cyrillic). "
                    "Передай null/None если хочешь спросить имя незнакомца — "
                    "робот спросит «Как вас зовут?» и ждёт ответа. "
                    "Если пользователь ИСПРАВЛЯЕТ имя («я не X, я Y») — "
                    "передай name=Y и old_name=X."
                ),
                required=False,
            ),
            MCPToolParameter(
                name="old_name",
                type="string",
                description=(
                    "Предыдущее имя (если пользователь исправляет: "
                    "«я не Эйджик, я Денчик» → old_name='Эйджик', name='Денчик'). "
                    "Опусти, если пользователь представляется впервые."
                ),
                required=False,
            ),
        ]

    @property
    def destructive(self) -> bool:
        return False

    # Issue #1101 — noise-name guard. LLM sometimes passes the trigger
    # word instead of the actual name ("робот меня зовут" → name="зовут"
    # instead of name="Денис"). Without this filter every such call
    # produced a junk row in /data/speakers.db (``name='Зовут'``). The
    # set covers the highest-frequency false positives observed on the
    # robot on 2026-08-10/11 (see PR-1101 cleanup migration notes).
    _NOISE_NAMES: frozenset[str] = frozenset(
        {
            "зовут",
            "имя",
            "меня",
            "зовут-это",
            "зовут меня",
            "это",
            "называю",
            "зовут-меня",
            "моё",
            "мое",
            "моё имя",
            "мое имя",
            "имя мне",
            "имя моё",
            "имя мое",
        }
    )

    def execute(self, name: str | None = None, old_name: str | None = None) -> MCPToolResult:
        import json
        from std_msgs.msg import String

        # Step 1 — if old_name provided, rename the existing entry first.
        if old_name and old_name.strip():
            old_clean = old_name.strip()
            if old_clean.lower() not in {"null", "none"} and len(old_clean) >= 2:
                rename_msg = String()
                rename_msg.data = json.dumps(
                    {"old_name": old_clean, "new_name": (name or "").strip() or old_clean},
                    ensure_ascii=False,
                )
                # Issue #1101 — rename идёт на /voice/speaker/rename
                # (speaker_id_node._on_rename_request), НЕ на register:
                # register-хендлер читает только {"name": ...} и
                # игнорировал бы {old_name, new_name} как пустое имя.
                self._speaker_rename_pub.publish(rename_msg)
                self.log_info(
                    f"[register_speaker] rename {old_clean!r} → "
                    f"{(name or '').strip()!r}"
                )

        if name is None or name.strip() == "":
            if old_name:
                return MCPToolResult(
                    success=True,
                    data={"renamed": True, "old_name": old_name},
                    message=f"Старое имя '{old_name}' исправлено. Новое имя не указано — спроси.",
                )
            self.log_info("[register_speaker] name=None → ask user")
            return MCPToolResult(
                success=True,
                data={"ask_required": True, "name": None},
                message="Спроси имя у пользователя через speak_text и вызови register_speaker(name=X) ещё раз",
            )
        name_clean = name.strip().capitalize()
        # Issue #1101 (live 11.08) — LLM sometimes serialises a real
        # ``null`` JSON value through OpenAI tool-call as the literal
        # string ``"null"`` (or ``"None"``). Without this branch the
        # speaker_id_node persists an embedding under ``name='Null'``
        # — a fresh junk row in /data/speakers.db. Treat ``"null"``
        # the same as ``None``: ask the user.
        if name_clean.lower() in {"null", "none"}:
            self.log_info(
                "[register_speaker] name='null' literal — treat as ask_user"
            )
            return MCPToolResult(
                success=True,
                data={"ask_required": True, "name": None},
                message=(
                    "Передан литерал 'null'/'None' — спроси имя у пользователя "
                    "через speak_text и вызови register_speaker(name=X) ещё раз"
                ),
            )
        if len(name_clean) < 2:
            return MCPToolResult(
                success=False,
                data={"error": "name_too_short"},
                message=f"Имя '{name}' слишком короткое — минимум 2 символа",
            )
        # Issue #1101 — reject trigger-word leakage from the user phrase.
        # Without this guard the LLM passes the first token after
        # «зовут» rather than the actual name (e.g. "робот меня зовут
        # Денис говорю" → name="Зовут"), polluting /data/speakers.db.
        if name_clean.lower() in self._NOISE_NAMES:
            self.log_info(
                f"[register_speaker] rejected noise name {name_clean!r} "
                "— LLM must extract actual name from user_input"
            )
            return MCPToolResult(
                success=False,
                data={"error": "noise_name", "received": name_clean},
                message=(
                    f"Имя '{name_clean}' выглядит как служебное слово из "
                    "фразы («зовут», «имя», «меня»). Извлеки реальное имя "
                    "из контекста (например, для «робот меня зовут Денис "
                    'говорю» — передай name="Денис") и вызови тул ещё раз.'
                ),
            )
        # publish в /voice/speaker/register — speaker_id_node привяжет d-vector
        msg = String()
        msg.data = json.dumps({"name": name_clean}, ensure_ascii=False)
        self._speaker_register_pub.publish(msg)
        self.log_info(f"[register_speaker] published name={name_clean!r}")
        return MCPToolResult(
            success=True,
            data={"registered_name": name_clean, "speaker_id": "pending"},
            message=f"Голос зарегистрирован как '{name_clean}'. Теперь этого пользователя можно узнавать по голосу.",
        )
