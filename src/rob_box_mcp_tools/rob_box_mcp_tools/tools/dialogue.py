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
from ..voice_state import VoiceStateStore
from ..animations import (
    KNOWN_ANIMATIONS,
    normalize_animation,
)


# Issue #1219 — LLM voice selection: единый реестр голосов живёт в
# rob_box_voice (чистый Python, без ROS). Ленивый импорт с fallback,
# чтобы пакет оставался импортируемым без rob_box_voice (CI linter).
try:
    from rob_box_voice.tts_voice_registry import (
        default_voice_for,
        resolve_voice,
        voices_for,
    )
except ImportError:  # pragma: no cover — fallback для minimal environments
    def default_voice_for(provider: str) -> str:
        return ""

    def resolve_voice(provider: str, requested) -> tuple:
        return (requested or ""), False

    def voices_for(provider: str) -> list:
        return []

# Issue #1709 — Unicode-script guard: LLM иногда вставляет в speak_text
# иероглифы/деванагари, TTS их бормочет. Общий pure-Python модуль в
# rob_box_voice (как tts_voice_registry). Ленивый импорт с no-op
# fallback — пакет должен оставаться импортируемым без rob_box_voice.
try:
    from rob_box_voice.tts_text_guard import (
        analyze as analyze_foreign_scripts,
        describe as describe_foreign_scripts,
        should_skip as should_skip_foreign_scripts,
    )
except ImportError:  # pragma: no cover — fallback для minimal environments
    def analyze_foreign_scripts(text: str):
        return None

    def describe_foreign_scripts(report) -> str:
        return "guard unavailable"

    def should_skip_foreign_scripts(text: str) -> bool:
        return False


def _active_tts_provider(node) -> str:
    """Активный TTS-провайдер для валидации голосов (issue #1229).

    Приоритет: фактический провайдер от tts_node (``node.actual_tts_provider``,
    после фолбека minimax→yandex это yandex) → параметр ``tts_provider``
    (номинальный, из YAML) → дефолт ``minimax``.

    Без этого speak_text/set_voice валидировали голоса по номинальному
    провайдеру, LLM выбирала minimax-голоса (male-chengshu и т.п.),
    которых нет у yandex после фолбека, и робот говорил тем же голосом.
    """
    actual = getattr(node, "actual_tts_provider", None)
    if actual:
        return str(actual)
    try:
        return str(node.get_parameter("tts_provider").value or "minimax")
    except Exception:  # noqa: BLE001 — stub/tests без параметра
        return "minimax"


class SpeakTextTool(MCPTool):
    """Инструмент для произнесения текста голосом."""

    def __init__(self, node, voice_store: VoiceStateStore | None = None):
        super().__init__(node)
        # Issue #1219 — общее in-memory хранилище current_voice с
        # SetVoiceTool (персистентный голос на диалог, Q7/Q11).
        self._voice_store = voice_store or VoiceStateStore()
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
        finished_error = result.get("error")

        # Issue #1709 — ПОЛНЫЙ текст чанка в логе. До фикса здесь был только
        # `speech_id`: когда TTS падал или его прерывали («🔇 Воспроизведение
        # прервано»), восстановить, ЧТО именно бормотал робот, было
        # невозможно — юзер слышал странный фрагмент, а в логе пусто.
        # Текст берём из snapshot'а pending_speeches (регистрируется в
        # execute() вместе с batch_id).
        with self.pending_speeches_lock:
            _known = self.pending_speeches.get(speech_id) if speech_id else None
            chunk_text = (_known or {}).get("text", "")
            chunk_voice = (_known or {}).get("voice", "")
        self._log_chunk_outcome(
            speech_id=speech_id,
            text=chunk_text,
            voice=chunk_voice,
            success=finished_success,
            error=finished_error,
            known=_known is not None,
        )
        if not speech_id:
            return

        with self.pending_speeches_lock:
            entry = self.pending_speeches.pop(speech_id, None)
            if entry is None:
                # 🔴 FIX (issue #776, live 12.08): /voice/tts/finished — общий
                # топик для ВСЕХ отправителей TTS. tts_node публикует finished
                # для любого speech_id, включая системные реплики dialogue_node
                # (триггеры thinking/confused, DJ-прощание и т.п.), которые НЕ
                # регистрируются в pending_speeches mcp_server. Плюс для
                # последнего чанка батча tts_node намеренно republish'ит
                # finished с batch_complete=true (issue #980, контракт
                # "subscribers are designed to be idempotent"). Оба случая —
                # штатное поведение общего топика: это не warning, а debug.
                # Реальная рассинхронизация (entry есть, batch None) ниже
                # по-прежнему остаётся warning.
                self.log_debug(
                    f"ℹ️ Speech {speech_id[:8]}... не найден в pending_speeches "
                    "(чужой TTS от dialogue_node или повторный finished с "
                    "batch_complete — игнорируем)"
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

    def _log_chunk_outcome(
        self,
        *,
        speech_id: str | None,
        text: str,
        voice: str,
        success: bool,
        error=None,
        known: bool,
    ) -> None:
        """Залогировать исход чанка с ПОЛНЫМ текстом (issue #1709).

        Acceptance issue #1709: «Логи TTS должны включать ``voice: <name>``
        + ``text: <full_text>`` для КАЖДОГО чанка (включая
        failed/interrupted)». До фикса ``_on_tts_finished`` печатал только
        ``speech_id`` + ``success``, поэтому по логу нельзя было понять,
        что именно бормотал робот в прерванном чанке — а именно там и
        сидели иероглифы (юзер слышал «хинди», в логе пусто).

        * успех → INFO;
        * провал/прерывание известного чанка → **WARNING** (это то, что
          диагностика ищет grep'ом);
        * чужой ``speech_id`` (общий топик ``/voice/tts/finished``, issue
          #776) → DEBUG, чтобы не засорять лог и не ломать
          ``test_finished_for_unknown_speech_does_not_publish_batch_complete``,
          который требует пустой ``warning_messages``.

        Текст НЕ обрезается: смысл фикса — видеть его целиком.
        """
        sid = f"{speech_id[:8]}..." if speech_id else "None"
        detail = (
            f"speech_id={sid} success={success} "
            f"voice={voice or 'default'} "
            f"error={error!r} "
            f"text={text!r}"
        )
        if not known:
            # Чужой/повторный finished — штатное поведение общего топика.
            self.log_debug(f"🔔 TTS finished (не наш чанк): {detail}")
        elif success:
            self.log_info(f"🔔 TTS finished: {detail}")
        else:
            self.log_warning(f"❌ TTS чанк НЕ произнесён: {detail}")

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
                name="voice",
                type="string",
                description=(
                    "Голос TTS для этой реплики (опционально, issue #1219). "
                    "Имя голоса активного провайдера, напр. alena/zahar (Yandex), "
                    "Russian_ReliableMan/Russian_BrightHeroine (MiniMax), aidar/baya (Silero). "
                    "Если голос не указан — используется голос, установленный set_voice, "
                    "иначе дефолтный голос провайдера. Неизвестный/недоступный голос "
                    "заменяется на дефолтный, фактический голос придёт в voice_used результата."
                ),
                required=False,
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
                enum=list(KNOWN_ANIMATIONS),
                # Нестрогий enum: список ведёт LLM к реальным именам, но
                # ``execute`` умеет нормализовать псевдонимы и русские
                # названия — валидация не должна рубить их до нормализации.
                enum_strict=False,
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

    def execute(self, text: str, animation: str = "neutral", voice: str | None = None) -> MCPToolResult:
        """Произнести текст."""
        import json
        import re
        import uuid

        # Issue #1219 — LLM voice selection: определяем активного провайдера
        # (фактический от tts_node после фолбека, иначе ROS-параметр
        # tts_provider; дефолт minimax — совпадает с tts_node.yaml).
        provider = _active_tts_provider(self.node)
        # Резолв голоса (issue #1219, Q6/Q7): разовый voice= из speak_text
        # важнее установленного set_voice; если оба отсутствуют — дефолт
        # провайдера. Неизвестный голос → дефолт + voice_used в результате.
        voice_used, voice_fell_back = self._voice_store.resolve(
            provider, requested=voice
        )
        if voice and voice_fell_back:
            self.log_warning(
                f"⚠️ Голос '{voice}' недоступен у провайдера '{provider}' — "
                f"использую дефолтный '{voice_used}'"
            )

        # 🔴 FIX (live 06.08): LLM (MiniMax-M3) иногда ВКЛЮЧАЕТ параметры
        # вызова В ТЕКСТ: text='...128 ударов!", animation="happy"'.
        # Вырезаем вшитый синтаксис `, animation="..."` / `, voice="..."` в
        # конце строки (опционально с лишней закрывающей кавычкой перед
        # запятой, одинарными кавычками, пробелами вокруг `=` и хвостовым
        # `)`), иначе TTS читает «анимейшин хеппи».
        text = re.sub(
            r'"?\s*,\s*(?:animation|voice|rate|pitch)\s*=\s*["\'][^"\']*["\']\s*\)?\s*$',
            '',
            text,
        )
        text = re.sub(r'\)\s*$', '', text) if text.endswith(')') else text
        text = text.strip()

        # Issue #1709 — ПОЛНЫЙ текст в логе (а не первые 50 символов): без
        # него нельзя восстановить, что именно ушло в TTS, когда чанк
        # падает или его прерывают.
        self.log_info(
            f"Произношение текста: animation={animation} "
            f"voice={voice_used} text={text!r}"
        )

        if not text:
            return MCPToolResult(success=False, error="Пустой текст", message="Текст не может быть пустым")

        # Issue #1709 — Unicode-script guard. LLM периодически вставляет в
        # speak_text иероглифы/деванагари/арабицу; TTS такой текст бормочет
        # (юзер слышал «что-то на хинди»). Если текст в основном состоит из
        # букв неподдерживаемых письменностей (>10%, см. tts_text_guard) —
        # НЕ произносим: логируем warning с полным текстом и возвращаем
        # честную ошибку, чтобы LLM переписала фразу транслитом.
        _guard_report = analyze_foreign_scripts(text)
        if _guard_report is not None and should_skip_foreign_scripts(text):
            self.log_warning(
                "🚫 [issue 1709] Чанк не произнесён: слишком много символов "
                f"неподдерживаемых письменностей — "
                f"{describe_foreign_scripts(_guard_report)}, "
                f"voice={voice_used}, text={text!r}"
            )
            return MCPToolResult(
                success=False,
                error="unsupported_script",
                data={
                    "text": text,
                    "foreign_ratio": round(_guard_report.ratio, 4),
                    "scripts": list(_guard_report.scripts),
                    "voice_used": voice_used,
                    "provider": provider,
                },
                message=(
                    "Текст не произнесён: содержит символы неподдерживаемых "
                    f"письменностей ({', '.join(_guard_report.scripts)}). "
                    "Перепиши фразу русскими буквами (транслитерацией) — "
                    "например «идиома звучит как „вай го“»."
                ),
            )

        # Нормализация анимаций: псевдонимы и русские названия приводятся к
        # реальному имени манифеста, всё остальное падает в "talking"
        # (робот же говорит) — текст при этом произносится в любом случае.
        requested_animation = animation
        animation = normalize_animation(animation, fallback="talking")
        if requested_animation and animation != str(requested_animation).strip().lower():
            if animation == "talking":
                self.log_warning(
                    f"⚠️ Неизвестная анимация '{requested_animation}' — "
                    f"использую 'talking', текст будет произнесён"
                )

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
                    # Issue #1709 — храним ПОЛНЫЙ текст и голос чанка,
                    # чтобы _on_tts_finished мог их залогировать даже для
                    # failed/interrupted чанка (раньше был только speech_id).
                    "text": chunk,
                    "voice": voice_used,
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
                # Issue #1219 — LLM voice selection: фактический голос после
                # валидации. tts_node резолвит его ещё раз по РЕАЛЬНОМУ
                # провайдеру (фолбек-цепочка) и логирует voice_used.
                "voice": voice_used,
            }
            if self._current_dialogue_id:
                tts_request["dialogue_id"] = self._current_dialogue_id
            msg = String()
            msg.data = json.dumps(tts_request, ensure_ascii=False)
            self.tts_pub.publish(msg)
            self.log_info(
                f"📤 TTS чанк {i+1}/{len(chunks)}: "
                f"speech_id={speech_id[:8]} batch_id={batch_id[:8]} "
                f"dialogue_id={self._current_dialogue_id[:8] if self._current_dialogue_id else 'None'} "
                f"voice={voice_used} text={chunk!r}"
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
                # Issue #1219 — LLM voice selection (Q6): фактический голос
                # после валидации + провайдер, для которого он выбран.
                "voice_used": voice_used,
                "provider": provider,
                "voice_fell_back": voice_fell_back,
            },
            message=(
                f"TTS запрос отправлен ({chunks_total} чанк(ов)): {text[:50]}... "
                f"[voice: {voice_used}, provider: {provider}]"
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
            "Зарегистрировать голос текущего собеседника в voice biometric DB "
            "(resemblyzer d-vector). Вызывай когда: (1) пользователь "
            "представился фразой типа «меня зовут X» / «моё имя X» — извлеки "
            "имя из user_input и передай name=ИМЯ (Cyrillic, ≥2 буквы, с "
            "заглавной); (2) хочешь узнать имя незнакомца — передай name=null "
            "и спроси «Как вас зовут?» через speak_text. "
            "ВАЖНО: НЕ передавай служебные слова «зовут», «имя», «меня», "
            "«зовут-это», «зовут меня» — это шумовые токены из фразы, а не "
            "реальные имена. Извлеки имя из контекста вручную (например, для "
            "фразы «робот меня зовут Денис говорю» — передай name=\"Денис\"). "
            "Имя сохранится в БД вместе с эмбеддингом голоса, после этого "
            "пользователя можно будет узнавать по голосу."
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


# Список поддерживаемых TTS-провайдеров для переключения (issue #1765).
# Совпадает с ключами tts_voice_registry.PROVIDER_VOICES и
# tts_node._default_provider_chain. Используется как enum в ToolSpec и
# при валидации пользовательского ввода (отсекаем опечатки типа
# «yandeх» / «minimax-tts» / «Яндекс» — LLM получит provider_unknown).
SUPPORTED_TTS_PROVIDERS: tuple[str, ...] = ("yandex", "minimax", "silero")


def _normalise_provider_name(raw: str | None) -> str:
    """Нормализовать имя провайдера: trim + lowercase.

    Принимает строки в любом регистре («Yandex», «Яндекс», « MINIMAX »),
    возвращает lowercase («yandex», «minimax»). Кириллицу не транслитерирует:
    «яandex» останется «яandex» и провалится валидацию в execute().
    """
    return (raw or "").strip().lower()


def _validate_provider(raw: str | None) -> tuple[str | None, str | None]:
    """Валидация имени провайдера для set_voice/set_tts_provider.

    Returns:
        ``(provider, error)`` — ровно одно из двух не-None:
        * ``(provider, None)`` — валидно;
        * ``(None, "<error_code>")`` — невалидно.
    """
    name = _normalise_provider_name(raw)
    if not name:
        return None, "provider_empty"
    if name not in SUPPORTED_TTS_PROVIDERS:
        return None, "provider_unknown"
    return name, None


class SetVoiceTool(MCPTool):
    """Issue #1219 — персистентный выбор голоса TTS (Q7/Q11, issue #1765).

    LLM вызывает этот tool когда пользователь просит «говори голосом X»
    или когда хочет сменить голос на диалог (например, рассказывать
    сказку от разных лиц). Голос хранится in-memory (VoiceStateStore) с
    привязкой к speaker_id; следующий ``speak_text`` без voice= говорит
    установленным голосом.

    Опциональный параметр ``provider`` (issue #1765): если задан, бот
    СНАЧАЛА переключает активного TTS-провайдера (yandex ↔ minimax ↔
    silero), затем валидирует голос против списка НОВОГО провайдера и
    сохраняет. Это закрывает баг «юзер сказал «Яндекс Артём», бот
    ответил «нет такого голоса», хотя Артём — yandex-голос, а активный
    был minimax». После успеха tool публикует ``/voice/tts/set_provider``
    (JSON {"provider", "voice"}); tts_node подписан и обновляет свой
    provider_chain + provider_dead_until, перепубликовывает
    ``/voice/tts/provider_state`` — dialogue_node/mcp_server увидят
    нового провайдера в ``[TTS]`` строке LLM-контекста.

    Контракт (docs/design/LLM_VOICE_SELECTION_PROPOSAL.md, issue #1765):
    ``{"tool": "set_voice", "params": {"voice": "artem"}}``
    → ``{"status": "ok", "voice_set": "artem", "provider": "yandex", ...}``

    ``{"tool": "set_voice", "params": {"voice": "artem", "provider": "yandex"}}``
    → ``{"status": "ok", "voice_set": "artem", "provider": "yandex",
       "provider_switched": true, ...}``
    """

    def __init__(self, node, voice_store: VoiceStateStore | None = None):
        super().__init__(node)
        self._voice_store = voice_store or VoiceStateStore()
        # Публикуем изменение, чтобы dialogue_node мог обновить LLM-контекст
        # ([TTS] current_voice) — payload JSON {"voice": str}.
        from std_msgs.msg import String

        self._voice_state_pub = node.create_publisher(
            String, "/voice/tts/current_voice", 10
        )
        # Issue #1765 — переключение TTS-провайдера. tts_node подписан на
        # этот topic и пересобирает provider_chain (новый провайдер
        # первым), чистит provider_dead_until для этого провайдера и
        # публикует provider_state обратно (dialogue_node/mcp_server
        # увидят нового провайдера в LLM-контексте).
        self._set_provider_pub = node.create_publisher(
            String, "/voice/tts/set_provider", 10
        )

    @property
    def name(self) -> str:
        return "set_voice"

    @property
    def description(self) -> str:
        return (
            "Установить голос TTS на диалог (персистентно, пока не сменят). "
            "Используй когда: (1) пользователь просит «говори голосом X» — "
            "передай voice=X; (2) хочешь рассказывать историю от разных лиц "
            "(старик → zahar, девушка → alena и т.п.); (3) хочешь вернуться "
            "к дефолтному голосу — передай voice=<default_voice>; (4) юзер "
            "просит голос с КОНКРЕТНОГО провайдера («Яндекс Артём», «Yandex "
            "anton») — передай voice=... И provider='yandex'. "
            "Доступные голоса перечислены в контексте: [TTS] voices=... "
            "После set_voice следующий speak_text без voice= говорит "
            "установленным голосом."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="voice",
                type="string",
                description=(
                    "Имя голоса для установки (из списка [TTS] voices=...). "
                    "Например: alena, zahar, jane (Yandex); Russian_ReliableMan, "
                    "Russian_BrightHeroine (MiniMax); aidar, baya (Silero)."
                ),
                required=True,
            ),
            MCPToolParameter(
                name="provider",
                type="string",
                description=(
                    "Опциональный TTS-провайдер («yandex» | «minimax» | "
                    "«silero»). Если задан — бот СНАЧАЛА переключает "
                    "провайдера, потом валидирует voice против голосов "
                    "НОВОГО провайдера. Используй когда юзер просит голос "
                    "по имени, привязанному к конкретному провайдеру "
                    "(«Яндекс Артём», «Yandex anton»), а в [TTS] provider: "
                    "сейчас другой провайдер."
                ),
                required=False,
                enum=list(SUPPORTED_TTS_PROVIDERS),
            ),
        ]

    @property
    def execution_type(self):
        from ..base import ToolExecutionType

        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, voice: str, provider: str = "") -> MCPToolResult:
        """Установить голос (опционально — переключить провайдера).

        Args:
            voice: имя голоса. Обязательно, не пустое.
            provider: опциональное имя TTS-провайдера (yandex/minimax/
                silero). Если задано и отличается от активного — сначала
                переключает провайдера, потом валидирует voice.
        """
        import json as _json

        voice_clean = (voice or "").strip()
        if not voice_clean:
            return MCPToolResult(
                success=False,
                error="voice_empty",
                message="Параметр voice не может быть пустым — передай имя голоса из [TTS] voices=...",
            )

        # Issue #1765 — опциональное переключение провайдера.
        # Валидируем параметр ДО валидации voice: если юзер указал
        # кривое имя провайдера, не сбрасываем его в активного молча —
        # возвращаем provider_unknown, LLM поправит.
        requested_provider, prov_err = _validate_provider(provider)
        if prov_err == "provider_empty":
            # Пустая строка — пользовательский «не указывать»; не ошибка.
            requested_provider = None
        elif prov_err == "provider_unknown":
            return MCPToolResult(
                success=False,
                data={
                    "error": "provider_unknown",
                    "requested": provider,
                    "supported": list(SUPPORTED_TTS_PROVIDERS),
                },
                message=(
                    f"Провайдер '{provider}' не поддерживается. "
                    f"Допустимые: {', '.join(SUPPORTED_TTS_PROVIDERS)}."
                ),
            )

        # Активный провайдер — как в speak_text (фактический от tts_node
        # после фолбека, иначе mcp_server param tts_provider).
        active_provider = _active_tts_provider(self.node)
        # Провайдер, относительно которого валидируем voice: если
        # requested_provider задан И отличается от активного — после
        # переключения будет он; иначе — активный.
        target_provider = requested_provider or active_provider
        provider_switched = bool(
            requested_provider and requested_provider != active_provider
        )

        # Валидация голоса. Делаем ПОСЛЕ вычисления target_provider, чтобы
        # для cross-provider случая (set_voice(voice="artem", provider="yandex")
        # при активном minimax) валидировали против yandex-списка, а не
        # против minimax (где «artem» отсутствует → voice_unavailable).
        known = voices_for(target_provider)
        if known and voice_clean not in known:
            return MCPToolResult(
                success=False,
                data={
                    "error": "voice_unavailable",
                    "requested": voice_clean,
                    "provider": target_provider,
                    "available": known,
                    "default_voice": default_voice_for(target_provider),
                },
                message=(
                    f"Голос '{voice_clean}' недоступен у провайдера "
                    f"'{target_provider}'. Доступные: {', '.join(known)}. "
                    f"Дефолтный: {default_voice_for(target_provider)}. "
                    f"Выбери голос из списка."
                ),
            )

        # Если провайдер переключался — публикуем set_provider ДО
        # сохранения голоса. tts_node подписан, пересоберёт chain и
        # переопубликует provider_state → mcp_server/dialogue_node
        # обновят [TTS] в LLM-контексте до следующего speak_text.
        if provider_switched:
            try:
                from std_msgs.msg import String as _String

                msg = _String()
                msg.data = _json.dumps(
                    {
                        "provider": requested_provider,
                        "voice": voice_clean,
                        "source": "set_voice",
                    },
                    ensure_ascii=False,
                )
                self._set_provider_pub.publish(msg)
            except Exception as exc:  # noqa: BLE001 — best-effort
                self.log_warning(
                    f"⚠️ [set_voice] publish set_provider failed: {exc}"
                )

        self._voice_store.set_voice(voice_clean)
        self.log_info(
            f"[set_voice] voice={voice_clean!r} provider={target_provider} "
            f"default={default_voice_for(target_provider)} "
            f"switched={provider_switched}"
        )
        # Публикуем смену голоса — dialogue_node обновит [TTS] current_voice.
        try:
            from std_msgs.msg import String as _String

            msg = _String()
            msg.data = _json.dumps(
                {"voice": voice_clean, "provider": target_provider},
                ensure_ascii=False,
            )
            self._voice_state_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 — best-effort
            self.log_warning(f"⚠️ [set_voice] publish voice_state failed: {exc}")

        return MCPToolResult(
            success=True,
            data={
                "status": "ok",
                "voice_set": voice_clean,
                "default_voice": default_voice_for(target_provider),
                "provider": target_provider,
                "previous_provider": active_provider if provider_switched else None,
                "provider_switched": provider_switched,
            },
            message=(
                f"Голос установлен: '{voice_clean}' на провайдере "
                f"'{target_provider}' (дефолтный: "
                f"{default_voice_for(target_provider)})"
                + (
                    f"; переключились с '{active_provider}'"
                    if provider_switched
                    else ""
                )
            ),
        )


class SetTtsProviderTool(MCPTool):
    """Issue #1765 — переключение активного TTS-провайдера.

    LLM вызывает этот tool когда юзер просит СМЕНИТЬ провайдера целиком
    без явного голоса («давай говорить Яндексом», «переключись на
    MiniMax», «давай через Silero — без сети»). Использует дефолтный
    голос нового провайдера (``yandex→anton``, ``minimax→male-qn-qingse``,
    ``silero→aidar``). Для смены голоса ВНУТРИ провайдера — используй
    ``set_voice``.

    Контракт:
    ``{"tool": "set_tts_provider", "params": {"provider": "yandex"}}``
    → ``{"status": "ok", "provider": "yandex", "default_voice": "anton"}``
    """

    def __init__(self, node, voice_store: VoiceStateStore | None = None):
        super().__init__(node)
        self._voice_store = voice_store or VoiceStateStore()
        from std_msgs.msg import String

        self._set_provider_pub = node.create_publisher(
            String, "/voice/tts/set_provider", 10
        )

    @property
    def name(self) -> str:
        return "set_tts_provider"

    @property
    def description(self) -> str:
        return (
            "Переключить активного TTS-провайдера (yandex ↔ minimax ↔ "
            "silero). Используй когда юзер просит СМЕНИТЬ ПРОВАЙДЕРА "
            "целиком: «давай говорить Яндексом», «переключись на "
            "MiniMax», «через Silero — без интернета». Голос будет "
            "дефолтным для нового провайдера. Для смены голоса внутри "
            "текущего провайдера — используй set_voice(voice=...)."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="provider",
                type="string",
                description=(
                    "Имя TTS-провайдера: «yandex» | «minimax» | «silero»."
                ),
                required=True,
                enum=list(SUPPORTED_TTS_PROVIDERS),
            ),
        ]

    @property
    def execution_type(self):
        from ..base import ToolExecutionType

        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, provider: str) -> MCPToolResult:
        import json as _json

        target_provider, prov_err = _validate_provider(provider)
        if prov_err == "provider_empty":
            return MCPToolResult(
                success=False,
                error="provider_empty",
                message=(
                    "Параметр provider обязателен. "
                    f"Допустимые: {', '.join(SUPPORTED_TTS_PROVIDERS)}."
                ),
            )
        if prov_err == "provider_unknown":
            return MCPToolResult(
                success=False,
                data={
                    "error": "provider_unknown",
                    "requested": provider,
                    "supported": list(SUPPORTED_TTS_PROVIDERS),
                },
                message=(
                    f"Провайдер '{provider}' не поддерживается. "
                    f"Допустимые: {', '.join(SUPPORTED_TTS_PROVIDERS)}."
                ),
            )

        # target_provider строго not-None после _validate_provider.
        assert target_provider is not None  # noqa: S101 — invariant
        active_provider = _active_tts_provider(self.node)
        if target_provider == active_provider:
            # Уже на нужном — no-op, но не ошибка: LLM мог перепутать
            # активного. Возвращаем текущее состояние, чтобы LLM не
            # крутил ретраи.
            return MCPToolResult(
                success=True,
                data={
                    "status": "ok",
                    "provider": target_provider,
                    "default_voice": default_voice_for(target_provider),
                    "provider_switched": False,
                    "noop": True,
                },
                message=(
                    f"Провайдер уже '{target_provider}', переключение "
                    f"не требуется."
                ),
            )

        default_voice = default_voice_for(target_provider)
        # Публикуем set_provider — tts_node пересоберёт chain,
        # почистит dead_until для нового провайдера и переопубликует
        # provider_state.
        try:
            from std_msgs.msg import String as _String

            msg = _String()
            msg.data = _json.dumps(
                {
                    "provider": target_provider,
                    "voice": default_voice,
                    "source": "set_tts_provider",
                },
                ensure_ascii=False,
            )
            self._set_provider_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 — best-effort
            self.log_warning(
                f"⚠️ [set_tts_provider] publish set_provider failed: {exc}"
            )

        # Запоминаем дефолтный голос нового провайдера в voice_store,
        # чтобы следующий speak_text без voice= говорил им (а не
        # старым голосом от старого провайдера).
        self._voice_store.set_voice(default_voice)

        self.log_info(
            f"[set_tts_provider] provider={target_provider!r} "
            f"default_voice={default_voice!r} previous={active_provider!r}"
        )

        return MCPToolResult(
            success=True,
            data={
                "status": "ok",
                "provider": target_provider,
                "default_voice": default_voice,
                "previous_provider": active_provider,
                "provider_switched": True,
            },
            message=(
                f"Провайдер переключён: '{active_provider}' → "
                f"'{target_provider}'. Дефолтный голос: '{default_voice}'."
            ),
        )


class ListTtsVoicesTool(MCPTool):
    """Issue #1765 — список голосов TTS по провайдеру (или всех).

    LLM вызывает когда юзер спрашивает «какие голоса есть на Яндексе?»,
    «какие у тебя есть голоса?», «а какие голоса у MiniMax?». Возвращает
    JSON-список. По умолчанию — голоса АКТИВНОГО провайдера; для
    кросс-провайдерного запроса — передай provider='yandex' /
    'minimax' / 'silero'.

    Контракт:
    ``{"tool": "list_tts_voices"}``
    → ``{"voices": [...], "provider": "<active>", "default_voice": "<...>"}``
    """

    @property
    def name(self) -> str:
        return "list_tts_voices"

    @property
    def description(self) -> str:
        return (
            "Список доступных голосов TTS. Без аргументов — голоса "
            "АКТИВНОГО провайдера (см. [TTS] provider: в контексте). "
            "С аргументом provider — голоса конкретного провайдера "
            "(«какие голоса есть на Яндексе?» → provider='yandex'). "
            "Используй когда юзер спрашивает «какие у тебя голоса?» / "
            "«а на yandex какие?»."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="provider",
                type="string",
                description=(
                    "Опциональный TTS-провайдер («yandex» | «minimax» | "
                    "«silero»). Без аргумента — активный провайдер."
                ),
                required=False,
                enum=list(SUPPORTED_TTS_PROVIDERS),
            ),
        ]

    @property
    def execution_type(self):
        from ..base import ToolExecutionType

        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, provider: str = "") -> "MCPToolResult":
        # Валидация (silent для пустой строки — берём активного).
        target_provider, prov_err = _validate_provider(provider)
        if prov_err == "provider_unknown":
            return MCPToolResult(
                success=False,
                data={
                    "error": "provider_unknown",
                    "requested": provider,
                    "supported": list(SUPPORTED_TTS_PROVIDERS),
                },
                message=(
                    f"Провайдер '{provider}' не поддерживается. "
                    f"Допустимые: {', '.join(SUPPORTED_TTS_PROVIDERS)}."
                ),
            )
        if target_provider is None:
            # Пустая строка → активный провайдер.
            target_provider = _active_tts_provider(self.node)
        assert target_provider is not None  # noqa: S101 — invariant
        voices = voices_for(target_provider)
        default_voice = default_voice_for(target_provider)
        return MCPToolResult(
            success=True,
            data={
                "provider": target_provider,
                "voices": voices,
                "default_voice": default_voice,
            },
            message=(
                f"Провайдер '{target_provider}': {len(voices)} голосов, "
                f"дефолтный '{default_voice}'."
            ),
        )
