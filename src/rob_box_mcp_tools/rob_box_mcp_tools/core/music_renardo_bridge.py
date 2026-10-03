#!/usr/bin/env python3
"""MusicRenardoBridge — low-level Renardo/SuperCollider plumbing for MusicManager.

Extracted from ``tools/music.py`` (issue chain: G-MUSIC → refactor) so the
listener-loop / OSC / SynthDef plumbing lives next to the rest of the core
business logic instead of inside the 6000-line ``music.py`` tools file.

What moved here:
  * ``DEFAULT_CRITICAL_SYNTHS`` constant.
  * ``_initialize_renardo`` / ``_verify_and_retry_synthdefs``.
  * ``_attach_renardo_reply_listener`` / ``_renardo_reply_listener_loop`` /
    ``_log_scsynth_reply_if_any`` / ``_log_osc_reply`` (issue #1808).
  * ``_ensure_renardo_available`` / ``_check_supercollider`` / ``_send_osc_raw``.
  * ``_split_osc_address`` / ``_decode_osc_args`` (module-level — pure functions
    on OSC byte buffers, no state, easy to test in isolation).
  * ``known_synth_names`` / ``_log_synth_truth_discrepancy`` /
    ``_evaluate_music_stack_health`` / ``is_music_stack_healthy`` /
    ``music_stack_unavailable_error``.

What did NOT move:
  * Music-session lifecycle (``_music_session_active_since``,
    ``_last_music_activity_at``, ``_music_deadline_at``, etc.) — these belong
    to ``MusicManager`` because they are about the LLM dialog session, not
    about the Renardo protocol.
  * DJ mode flag — same reason; ``set_dj_mode`` stays on ``MusicManager``.
  * Master limiter fader writes (``set_master_gain`` / ``_master_gain``) —
    also stay on ``MusicManager``; the bridge exposes ``DEFAULT_MASTER_GAIN``
    as a class-level default the manager can use.
"""

from __future__ import annotations

import logging as _logging_module  # noqa: F401  — kept for tests that patch it
import os
import socket
import struct
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from rob_box_voice.core.music_stack_validation import (
    MusicStackStatus,
    load_confirmed_synths,
    load_sclang_health,
)
from rob_box_voice.core.sc_only_custom_synthdefs import (
    CUSTOM_SC_ONLY_SYNTH_NAMES,
    register_sc_only_custom_synthdefs,
)


# ----------------------------------------------------------------------
# Module-level OSC helpers — pure functions on byte buffers, no state.
# ----------------------------------------------------------------------


def _split_osc_address(data: bytes) -> Tuple[Optional[str], bytes]:
    """Извлечь OSC-адрес из пакета; вернуть (адрес, остаток-с-выравниванием)."""
    if not data or data[0:1] != b"/":
        return None, b""
    end = data.find(b"\x00")
    if end == -1:
        return None, b""
    address = data[:end].decode("ascii", "replace")
    consumed = end + 1
    while consumed % 4:
        consumed += 1
    return address, data[consumed:]


def _decode_osc_arg_int(rest: bytes, offset: int) -> Tuple[Optional[int], int]:
    """Decode big-endian int32; return (value, next_offset). None → stop."""
    if offset + 4 > len(rest):
        return None, offset
    return struct.unpack(">i", rest[offset:offset + 4])[0], offset + 4


def _decode_osc_arg_float(rest: bytes, offset: int) -> Tuple[Optional[float], int]:
    """Decode big-endian float32; return (value, next_offset). None → stop."""
    if offset + 4 > len(rest):
        return None, offset
    return struct.unpack(">f", rest[offset:offset + 4])[0], offset + 4


def _decode_osc_arg_string(rest: bytes, offset: int) -> Tuple[Optional[str], int]:
    """Decode null-terminated string; align to 4 bytes; (value, next_offset)."""
    str_end = rest.find(b"\x00", offset)
    if str_end == -1:
        return None, offset
    value = rest[offset:str_end].decode("utf-8", "replace")
    next_offset = str_end + 1
    while next_offset % 4:
        next_offset += 1
    return value, next_offset


def _decode_osc_args(rest: bytes) -> List[Any]:
    """Разобрать OSC type-tag строку (``,ssif``...) и аргументы за ней.

    Разбит на per-type helper'ы (``_decode_osc_arg_int/float/string``) чтобы
    уложиться в CC≤12 — единый switch+if-elif по типу выдавал CC=13.
    """
    if not rest or rest[0:1] != b",":
        return []
    end = rest.find(b"\x00")
    if end == -1:
        return []
    tags = rest[1:end].decode("ascii", "replace")
    offset = end + 1
    while offset % 4:
        offset += 1
    args: List[Any] = []
    for tag in tags:
        if tag == "i":
            value, offset = _decode_osc_arg_int(rest, offset)
        elif tag == "f":
            value, offset = _decode_osc_arg_float(rest, offset)
        elif tag == "s":
            value, offset = _decode_osc_arg_string(rest, offset)
        else:
            # blob (b) и прочие типы не разбираем — для лога достаточно
            # того, что уже накопили; останавливаемся, а не падаем.
            break
        if value is None:
            break
        args.append(value)
    return args


class MusicRenardoBridge:
    """Low-level Renardo/SuperCollider bridge.

    Owns:
      * SynthDef bookkeeping (``_synthdefs_added``, ``_server_confirmed_synths``).
      * Renardo runtime state (``_renardo_available``, ``_renardo_context``,
        ``_renardo_last_error``, ``_renardo_reply_sock``).
      * Stack-health snapshot (``_music_stack_status``, ``_require_healthy``,
        ``_critical_synths``).
      * All OSC plumbing (``_send_osc_raw``, ``_check_supercollider``).
      * Reply-listener thread (issue #1808).

    Does NOT own:
      * LLM dialog-session timestamps (managed by ``MusicManager``).
      * DJ mode flag.
      * Master limiter fader writes (the bridge only carries the default
        value ``DEFAULT_MASTER_GAIN``; ``MusicManager.set_master_gain``
        consumes it).
      * Pattern history / current preset (managed by ``MusicManager``).

    The split mirrors the seam used in the original ``tools/music.py``:
    everything touching scsynth/Renardo is here, everything tied to the
    LLM-facing session stays on ``MusicManager``.
    """

    SC_HOST: str = "127.0.0.1"
    SC_PORT: int = 57110

    # ------------------------------------------------------------------
    # Issue #1808 — слушатель ответов scsynth (/fail, /done)
    # ------------------------------------------------------------------
    # Renardo шлёт ноты в scsynth fire-and-forget и НИКОГДА не читает ответы
    # (см. ``_attach_renardo_reply_listener`` ниже) — все отказы звукового
    # тракта («SynthDef not found», «too many nodes», «Group N not found»)
    # были видны только в логе контейнера ``supercollider`` (сам scsynth их
    # печатает), куда никто не смотрит при разборе инцидентов.
    #
    # Таймаут ниже используется ТОЛЬКО в ``_send_osc_raw`` (наши собственные
    # админ-сообщения — /g_new, /g_freeAll, /n_set мастер-фейдера): после
    # sendto() кратко слушаем тот же сокет на предмет /fail. На УСПЕШНЫЙ
    # /g_new или /n_set scsynth вообще ничего не шлёт в ответ — значит этот
    # таймаут оплачивается ПОЛНОСТЬЮ на каждом успешном вызове. Держим его
    # маленьким (заметно меньше уже существующей паузы 50ms между
    # /g_freeAll и /g_new, issue #778) — на loopback ответ, если он будет,
    # приходит за микросекунды, а лишние 30ms на нечастых admin-вызовах
    # (пересоздание группы, смена мастер-гейна) незаметны на фоне музыки.
    OSC_REPLY_TIMEOUT_SECONDS: float = 0.03

    # ------------------------------------------------------------------
    # Master limiter (docs/analysis/2026-08-30-music-quality-audit.md)
    # ------------------------------------------------------------------
    #: Node ID синта ``masterlimiter``, который ``foxdot_init.sc`` ставит в
    #: хвост RootNode. Держится НИЖЕ 1000: renardo раздаёт ID начиная с 1001
    #: и только вверх (``ServerManager.nextnodeID``), поэтому коллизии быть
    #: не может, а ``/g_freeAll 1`` (Clock.clear / stop_all) чистит только
    #: группу 1 и лимитер не трогает.
    MASTER_LIMITER_NODE: int = 999
    #: Уровень мастер-фейдера ПОСЛЕ лимитера. Именно он задаёт громкость
    #: музыки относительно речи (issue #986), а не покомпонентные капы amp.
    DEFAULT_MASTER_GAIN: float = 0.5
    #: Class-level fallback-ы: ``__init__`` их перекрывает, но мост
    #: конструируют и через ``MusicRenardoBridge.__new__`` (тесты,
    #: восстановление после частичной деградации). Без них ``execute_code``
    #: падал бы с AttributeError.
    _master_gain: float = DEFAULT_MASTER_GAIN
    _master_gain_applied: bool = False
    #: Issue #1808 — сокет Renardo (``_rt.Server.client.socket``), к которому
    #: подключён фоновый слушатель ответов scsynth. ``None`` пока слушатель
    #: не подключён (или подключить не удалось — best-effort).
    _renardo_reply_sock: Optional[Any] = None

    #: ``ROB_BOX_MUSIC_REQUIRE_HEALTHY=1`` → degraded sclang runtime blocks
    #: ``execute_music_code`` / ``set_vibe_preset`` instead of letting the LLM
    #: fight a broken Renardo stack. Defaults to ``False`` — music tools work
    #: in degraded mode (non-critical SynthDef parse errors don't block).
    #: Set to ``1`` to fail-fast on any sclang startup error.
    REQUIRE_HEALTHY_DEFAULT = False
    #: Critical SynthDefs the music subsystem depends on. Mirrors the list
    #: used by ``start_voice_assistant.sh`` — keep them in sync.
    DEFAULT_CRITICAL_SYNTHS: Tuple[str, ...] = (
        "strings",
        "wobblebass",
        "pianovel",
        "warmpad",
        "retrobass",
        "supersawlead",
        "imperialbrass",
        "marchstrings",
        "strangerpulsepad",
        "strangerarp",
        "strangerbrass",
    )
    #: Шире-диапазонный список критичных сэмплов для верификации/досылки
    #: в ``_verify_and_retry_synthdefs``. Изначально был модуль-уровневой
    #: константой ``CRITICAL_SYNTHS`` в ``tools/music.py``; переехал сюда
    #: как class attr при рефакторе G-MUSIC. Отличается от
    #: ``DEFAULT_CRITICAL_SYNTHS`` (это boot-time health-check список,
    #: 11 имён) — здесь полный рабочий набор из 37 сэмплов, который
    #: ``_verify_and_retry_synthdefs`` зондирует через ``/s_new`` после
    # основной загрузки ``sdef.add()``.
    CRITICAL_SYNTHS: Tuple[str, ...] = (
        # melody
        "blip", "arpy", "pianovel", "epiano", "rhpiano", "karp", "sitar",
        "marimba", "bell", "cs80lead", "supersawlead", "imperialbrass",
        "strangerarp",
        # bass
        "dub", "wobblebass", "fuzz", "dirt", "subbass", "moogbass",
        "retrobass",
        # pads
        "strings", "pads", "ambi", "space", "sinepad", "warmpad",
        "strangerpulsepad",
        # brass
        "brass", "flute", "soprano", "eoboe", "organ", "strangerbrass",
        # glitch
        "rave", "donk", "varsaw", "pulse", "tb303",
        # не в палитре, но используются напрямую
        "bass", "gong", "pluck", "saw", "square", "faim", "viola",
        "noise", "scatter", "orient", "creep", "play1", "play2",
    )

    def __init__(
        self,
        critical_synths: Optional[List[str]] = None,
        require_healthy: Optional[bool] = None,
        sclang_log_path: Optional[str] = None,
    ) -> None:
        #: When True, ``execute_code`` / ``set_vibe_preset`` reject calls when
        #: ``_music_stack_status.is_healthy`` is False. Set False only for
        #: tests / dev environments where we explicitly want degraded mode.
        if require_healthy is None:
            env_flag = os.environ.get("ROB_BOX_MUSIC_REQUIRE_HEALTHY")
            if env_flag is None:
                self._require_healthy: bool = self.REQUIRE_HEALTHY_DEFAULT
            else:
                self._require_healthy: bool = env_flag.strip().lower() not in {"0", "false", "no", "off"}
        else:
            self._require_healthy = bool(require_healthy)
        #: Critical SynthDef set used by the boot-time health check.
        if critical_synths is None:
            env_synths = os.environ.get("ROB_BOX_MUSIC_CRITICAL_SYNTHS")
            if env_synths:
                self._critical_synths: Tuple[str, ...] = tuple(
                    name.strip() for name in env_synths.split(",") if name.strip()
                )
            else:
                self._critical_synths = self.DEFAULT_CRITICAL_SYNTHS
        else:
            self._critical_synths = tuple(critical_synths)
        #: SynthDef-ы, уже загруженные через sdef.add(). Повторный add()
        #: мутирует UGen-граф (osc*env) → компаундинг ("too big for
        #: sending") → scsynth не тянет → "late" и троттл (live 20.08).
        self._synthdefs_added: set = set()
        #: Issue #2838 — SynthDef-ы, приход которых в scsynth подтвердил
        #: sclang (строки прелоада "SynthDef in scsynth: X" после
        #: Server.sync). ``None`` — подтверждения нет (лог недоступен или
        #: прелоад не завершён). Пишет ``_evaluate_music_stack_health``.
        self._server_confirmed_synths: Optional[frozenset] = None
        #: контекст выполнения для renardo
        self._renardo_context: Dict[str, Any] = {}
        #: True если renardo доступен, False/None иначе
        self._renardo_available: Optional[bool] = None
        #: Последняя ошибка инициализации renardo для диагностики
        self._renardo_last_error: Optional[str] = None
        #: Issue #1808 — сокет Renardo, к которому подключён фоновый
        #: слушатель ответов scsynth (см. ``_attach_renardo_reply_listener``).
        self._renardo_reply_sock: Optional[Any] = None
        #: Music-stack health snapshot (from ``load_sclang_health``). When
        #: ``is_healthy is False``, ``execute_music_code`` / ``set_vibe_preset``
        #: short-circuit with a clear "music unavailable" error so the LLM
        #: doesn't keep retrying against a broken Renardo/FoxDot upstream.
        self._music_stack_status: MusicStackStatus = MusicStackStatus(
            is_healthy=True,
            oscdef_registered=True,
            missing_synths=(),
            fatal_errors=(),
        )
        # ------------------------------------------------------------------
        # Music stack health (issue G-MUSIC, architect review v3)
        # ------------------------------------------------------------------
        # If sclang already wrote a startup log and it's degraded, refuse to
        # initialize Renardo and surface a clear "music unavailable" error.
        # We do this BEFORE calling _initialize_renardo() so a broken
        # upstream .scd file cannot manifest as silent exec errors later.
        self._evaluate_music_stack_health(sclang_log_path=sclang_log_path)
        self._initialize_renardo()

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _initialize_renardo(self) -> None:
        """Попытка инициализировать Renardo-контекст и загрузить SynthDef-ы в SC.

        Pipeline:
        1. Создаём директории семплов (иначе renardo_lib.runtime падает при импорте).
        2. Импортируем renardo_lib.runtime.
        3. Подключаемся к scsynth через Server.init_connection().
        4. Создаём Group 1 в scsynth через raw OSC (иначе /s_new падает).
        5. Загружаем все SynthDef-ы: sdef.add() → write(.scd) + load() →
           OSC /foxdot → sclang компилирует .scd → /d_recv → scsynth.
        6. Ждём 5 секунд пока sclang скомпилирует все 188 SynthDef-ов.

        NOTE: SynthDefs — это plain dict, НЕ объект с методом .reload()!
        Правильный способ: for sdef in SynthDefs.values(): sdef.add()

        Декомпозирован на helper'ы (``_ensure_renardo_samples``,
        ``_import_and_connect_renardo``, ``_load_all_synthdefs``,
        ``_finalize_renardo_init``) чтобы уложиться в CC≤12 — исходный
        6000-строчный блок тянул 15 точек ветвления (sample dirs × 2 +
        pitchglitch × 4 + sdef.add × 2 + effect_manager try/except + 2
        verification инициализации).
        """
        try:
            self._ensure_renardo_samples()
            _rt = self._import_and_connect_renardo()
            self._attach_renardo_reply_listener(_rt)
            self._send_osc_raw("/g_new", 1, 0, 0)
            self._load_all_synthdefs(_rt)
            self._reload_renardo_effects(_rt)
            # Ждём компиляции всех 188 SynthDef-ов через sclang.
            # Без паузы renardo сразу пытается играть, scsynth отвечает "not found".
            time.sleep(5)
            # 🔴 FIX (live 12.08): верификация — пробуем /s_new на критичные
            # синты и досылаем пропавшие через sdef.add() (до 3 раундов).
            # Без этого музыка тихо молчит при "SynthDef not found".
            self._verify_and_retry_synthdefs(_rt, self._send_osc_raw)
            self._finalize_renardo_init(_rt)
        except (ImportError, Exception) as exc:
            self._renardo_available = False
            self._renardo_context = {}
            self._renardo_last_error = str(exc)

    def _ensure_renardo_samples(self) -> None:
        """Создать пустую структуру директорий 0_foxdot_default + долить вокалы.

        renardo_lib.runtime при импорте пытается листить ``0_foxdot_default``
        — без него падает ``FileNotFoundError``. Также копируем из
        ``1_pitchglitch_samples`` букву 'c' (vokals), потому что play("c   ")
        ищет её именно в ``0_foxdot_default``, а Renardo жёстко зашит на
        этот каталог (``DEFAULT_SAMPLES_PACK_NAME``).
        """
        import pathlib
        import shutil

        samples_base = pathlib.Path.home() / ".config" / "renardo" / "samples" / "0_foxdot_default"
        _SAMPLE_SUBDIRS = ["_", "_loop_"] + list("abcdefghijklmnopqrstuvwxyz")
        for subdir in _SAMPLE_SUBDIRS:
            (samples_base / subdir).mkdir(parents=True, exist_ok=True)

        pitchglitch = pathlib.Path.home() / ".config" / "renardo" / "samples" / "1_pitchglitch_samples"
        if not pitchglitch.exists():
            return
        for letter in list("abcdefghijklmnopqrstuvwxyz"):
            for case_dir in ("lower", "upper"):
                src_dir = pitchglitch / letter / case_dir
                dst_dir = samples_base / letter / case_dir
                if not src_dir.exists():
                    continue
                self._merge_sample_letter(src_dir, dst_dir)

    @staticmethod
    def _merge_sample_letter(src_dir: Any, dst_dir: Any) -> None:
        """Скопировать ``*.wav`` из ``src_dir`` в ``dst_dir`` поверх существующих.

        Использует «set уже-имеющихся имён → copy2 только новых», чтобы
        не тратить I/O на повторную перезапись 100+ файлов при каждом
        старте контейнера.
        """
        import shutil as _shutil

        dst_dir.mkdir(parents=True, exist_ok=True)
        dst_wavs = {f.name for f in dst_dir.glob("*.wav")}
        for wav in src_dir.glob("*.wav"):
            if wav.name not in dst_wavs:
                _shutil.copy2(wav, dst_dir / wav.name)

    def _import_and_connect_renardo(self) -> Any:
        """Импортировать renardo_lib.runtime и подключиться к scsynth.

        ``Server.booted`` снимается вызовом ``init_connection()`` (renardo
        сам не делает это при импорте — было поломано в 2.0+). Возвращает
        загруженный ``renardo_lib.runtime`` namespace.
        """
        import renardo_lib.runtime as _rt

        if not _rt.Server.booted:
            _rt.Server.init_connection()
        return _rt

    def _load_all_synthdefs(self, _rt: Any) -> None:
        """Загрузить все SynthDef-ы через ``sdef.add()`` + пейсинг.

        ``sdef.add()`` пишет ``.scd`` на диск и шлёт ``/foxdot`` в sclang,
        который компилирует и передаёт в scsynth через ``/d_recv``. 188
        запросов залпом роняют UDP-буфер sclang (drops >500 в
        ``/proc/net/udp``) — добавлен пейсинг 0.1с между пачками по 5.
        Повторный ``add()`` уже отправленного имени пропускаем — повторный
        add() мутирует UGen-граф и даёт "too big for sending".
        """
        for idx, (name, sdef) in enumerate(_rt.SynthDefs.items()):
            if name in self._synthdefs_added:
                continue
            sdef.add()
            self._synthdefs_added.add(name)
            if idx % 5 == 4:
                time.sleep(0.1)

    def _reload_renardo_effects(self, _rt: Any) -> None:
        """``EffectManager.reload()`` с записью last_error при неудаче.

        Без reload() scsynth отвечает "SynthDef reverb/volume not found"
        на каждый Player с ``room=``/``amp=``-fx и музыка молчит (live
        05.08 — все e2e после деплоя тихие, TTS работает, музыка нет).
        EffectManager.reload() = ``effect.load()`` для каждого эффекта +
        In()/Out() (служебные bus-ноды).
        """
        try:
            _rt.effect_manager.reload()
        except Exception as exc:  # noqa: BLE001
            self._renardo_last_error = f"effect_manager.reload failed: {exc}"

    def _finalize_renardo_init(self, _rt: Any) -> None:
        """Снепшот контекста, флаг ``_renardo_available``, лог расхождений."""
        self._renardo_context = vars(_rt).copy()
        register_sc_only_custom_synthdefs(_rt, self._renardo_context)
        self._renardo_available = True
        self._renardo_last_error = None
        self._log_synth_truth_discrepancy()

    def _verify_and_retry_synthdefs(
        self,
        _rt: Any,
        _send_osc_raw: Any,
        max_rounds: int = 3,
    ) -> None:
        """Verify critical SynthDefs exist in scsynth; re-send missing ones.

        live 12.08: после бурста sdef.add() часть SynthDef-ов пропадает
        (UDP drops на 57120). Пробуем /s_new для каждого критичного синта,
        пропавшие досылаем через sdef.add() → /foxdot → sclang → /d_recv.

        Args:
            _rt: renardo_lib.runtime module.
            _send_osc_raw: callable для отправки OSC на scsynth.
            max_rounds: сколько раундов досылки пробовать.
        """
        import struct as _struct
        import time as _time

        _CRITICAL_SYNTHS = self.CRITICAL_SYNTHS

        def _probe_missing(names):
            """Return subset of names whose SynthDef is absent in scsynth."""
            missing: list[str] = []
            probe_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            probe_sock.settimeout(0.4)

            def _free_probe_node(node_id: int) -> None:
                """Free a probe node via /n_free (best-effort, fire-and-forget)."""
                free_msg = bytearray(b"/n_free\x00")
                free_msg.extend(b",i\x00\x00")
                free_msg.extend(_struct.pack(">i", node_id))
                try:
                    probe_sock.sendto(bytes(free_msg), (self.SC_HOST, self.SC_PORT))
                except OSError:
                    pass

            for i, name in enumerate(names):
                node_id = 9000 + i
                # 🔴 FIX (live 13.08): OSC-адрес обязан быть выровнен до
                # кратного 4. "/s_new" = 6 байт + 2 нуля = 8 (было 7 —
                # scsynth отвечал "FAILURE IN SERVER: /s_new Command not
                # found", а строка "not found" матчилась как «синт
                # отсутствует» → ложные 3 раунда досылки всех 31 синтов).
                msg = bytearray(b"/s_new\x00\x00")
                # 🔴 FIX (19.08, свист #1444): type tag был ",siiiif" —
                # control-имя "amp" типизировано как int (i), а не string (s).
                # scsynth читал "amp\x00" как int32 → amp=0 НЕ применялся,
                # пробные ноды играли с дефолтным amp=1 и sus=1 (sustained)
                # и никогда не освобождались → постоянный свист из динамика.
                # Правильный tag ",siiisf": name(s) + node/addAction/target
                # (iii) + "amp"(s) + 0.0(f).
                types = b",siiisf\x00"
                name_b = name.encode() + b"\x00"
                while len(name_b) % 4:
                    name_b += b"\x00"
                msg.extend(types)
                msg.extend(name_b)
                msg.extend(_struct.pack(">iii", node_id, 0, 1))
                msg.extend(b"amp\x00")
                msg.extend(_struct.pack(">f", 0.0))
                try:
                    probe_sock.sendto(bytes(msg), (self.SC_HOST, self.SC_PORT))
                    data, _ = probe_sock.recvfrom(512)
                    # Only a REAL "SynthDef X not found" counts as missing.
                    # "Command not found" (malformed) and "Group N not found"
                    # must not trigger re-sending.
                    if b"SynthDef" in data and b"not found" in data:
                        missing.append(name)
                    else:
                        # 🔴 FIX (19.08, свист #1444): освобождаем созданную
                        # пробную ноду сразу (иначе она живёт вечно с sus=1).
                        _free_probe_node(node_id)
                except socket.timeout:
                    # Нет ответа = def на месте = нода создана — освобождаем.
                    _free_probe_node(node_id)
            probe_sock.close()
            return missing

        missing = list(_CRITICAL_SYNTHS)
        for round_no in range(max_rounds):
            missing = _probe_missing(missing)
            if not missing:
                return
            self._log_warning(
                f"[music] round {round_no + 1}: missing SynthDefs: {missing} — re-sending"
            )
            for name in missing:
                try:
                    sdef = _rt.SynthDefs.get(name)
                except Exception:  # noqa: BLE001
                    continue
                if sdef is None:
                    continue
                try:
                    if name in self._synthdefs_added:
                        # Уже добавляли — повторный add() мутирует UGen-граф
                        # (компаундинг). Досылаем без мутации через load()
                        # (отправка готового .scd), если метод доступен.
                        load = getattr(sdef, "load", None)
                        if load is not None:
                            load()
                    else:
                        sdef.add()
                        self._synthdefs_added.add(name)
                except Exception:  # noqa: BLE001
                    continue
                _time.sleep(0.3)
            _time.sleep(5)  # время на компиляцию
        self._log_warning(
            f"[music] SynthDefs still missing after {max_rounds} rounds: {missing}"
        )

    def _log_warning(self, message: str) -> None:
        """Log via the bridge's logger when available (fallback to print)."""
        logger = getattr(self, "_logger", None)
        if logger is not None:
            try:
                logger.warning(message)
                return
            except Exception:  # noqa: BLE001
                pass
        import sys as _sys
        _sys.stderr.write(f"{message}\n")
        _sys.stderr.flush()

    # ------------------------------------------------------------------
    # Issue #1808 — слушатель ответов scsynth (/fail, /done)
    # ------------------------------------------------------------------
    #
    # РЕШЕНИЕ (обоснование выбора «только логировать», см. issue #1808):
    #
    # У scsynth-трафика два независимых источника:
    #   (а) наши собственные админ-команды — ``_send_osc_raw`` (создание
    #       Group 1, /g_freeAll+/g_new при Clock.clear(), /n_set мастер-
    #       фейдера) — синхронные, отправляются и завершаются внутри
    #       одного вызова Python;
    #   (б) реальные ноты, которые Renardo шлёт из своего Clock-потока —
    #       АСИНХРОННО, зачастую на следующий бит ПОСЛЕ того, как
    #       ``execute_code``/``compose_music`` уже вернул «успешно».
    #
    # Для (б) нет способа синхронно привязать ответ scsynth к конкретному
    # вызову тула — только эвристика по времени, а именно её юзер попросил
    # не городить («привязывать по времени осторожно, ложные срабатывания
    # хуже молчания»). Один /fail может относиться к вызову N, а прийти
    # уже во время обработки вызова N+1 — риск обвинить не тот tool-call.
    #
    # Поэтому оба источника (а) и (б) только ЛОГИРУЮТСЯ в лог ноды
    # mcp_server (тот же ``_log_warning``, что и остальные диагностические
    # сообщения в этом файле — попадает в ``docker logs voice-assistant``).
    # Возврат в результат ``execute_music_code``/``compose_music`` — заявлен
    # в issue как ценное развитие, оставлен как отдельный follow-up, когда
    # появится безопасный способ привязки без ложных срабатываний.

    def _attach_renardo_reply_listener(self, _rt: Any) -> None:
        """Повесить фоновый слушатель на СОБСТВЕННЫЙ сокет Renardo (best-effort).

        Renardo (``renardo.sc_backend.server_manager.ServerManager``) держит
        ОДИН долгоживущий UDP-сокет для всех сообщений к scsynth
        (``Server.client.socket``, законнекченный на 127.0.0.1:57110) и
        никогда его не читает — ``OSCClient.send()`` только ``sendall()``,
        ни одного ``recv`` во всём классе. Значит ответы scsynth на РЕАЛЬНЫЕ
        ноты («SynthDef blip not found», «/s_new too many nodes», «Group N
        not found» — именно те три бага, что стоили нам двух дней отладки)
        сейчас просто лежат непрочитанными в приёмном буфере этого сокета.

        Мы ничего не меняем в отправке (Renardo продолжает слать как
        раньше) и только ЧИТАЕМ из ТОГО ЖЕ сокета в отдельном потоке —
        recv() и send() на законнекченном UDP-сокете независимы друг от
        друга, гонки с Renardo нет (он этот сокет не читает вовсе).

        Полностью best-effort и не бросает исключений наружу: если версия
        Renardo другая и объектный граф не совпадает (``Server``/``client``/
        ``socket`` переименованы или отсутствуют), просто не получаем этот
        источник и остаёмся с логированием только ``_send_osc_raw`` —
        никогда не роняем инициализацию музыки и никогда не выдумываем
        логи (нечего слушать = тишина, а не ложное срабатывание).
        """
        try:
            server = getattr(_rt, "Server", None)
            client = getattr(server, "client", None)
            sock = getattr(client, "socket", None)
            if not isinstance(sock, socket.socket):
                return
            if sock is self._renardo_reply_sock:
                return  # уже слушаем этот же сокет (повторный _ensure_renardo_available)
            self._renardo_reply_sock = sock
            thread = threading.Thread(
                target=self._renardo_reply_listener_loop,
                args=(sock,),
                name="scsynth-reply-listener",
                daemon=True,
            )
            thread.start()
        except Exception:  # noqa: BLE001 — best-effort, не мешаем инициализации
            pass

    def _renardo_reply_listener_loop(self, sock: "socket.socket") -> None:
        """Фоновый цикл: блокирующий recv на сокете Renardo, лог каждого /fail.

        Сокет Renardo обычно блокирующий (без ``settimeout``) — поток тихо
        спит между ответами, CPU не тратит. Если сокет закроют (например,
        Renardo пересоздаст ``Server.client`` при повторной инициализации),
        ``_pump_one_message`` вернёт ``None`` (recvfrom бросит ``OSError``)
        и поток завершается сам, без шума.

        Декомпозиция (issue #1808 follow-up #3371):
          * чтение из сокета вынесено в ``_pump_one_message`` — единственная
            точка, где мы знаем про ``recvfrom`` / ``OSError``;
          * разбор и лог вынесены в ``_route_osc_reply`` (один путь на оба
            источника — фоновый поток и короткий таймаут из ``_send_osc_raw``),
            а предикат синтезаторных ошибок — в ``_is_scsynth_synthdef_log``.
            Сам цикл остаётся CC-дешёвым (≤12), а главное — каждое решение
            тестируется изолированно в ``test_music.py``.
        """
        while True:
            data = self._pump_one_message(sock)
            if data is None:
                return  # сокет закрыт или невосстановимая OSError
            try:
                self._route_osc_reply(data, when_iso=None)
            except Exception:  # noqa: BLE001 — единичный кривой пакет не должен убивать поток
                continue

    def _log_scsynth_reply_if_any(self, sock: "socket.socket") -> None:
        """После собственного ``sendto`` кратко послушать тот же сокет на /fail.

        Таймаут короткий (``OSC_REPLY_TIMEOUT_SECONDS``) — см. обоснование
        у объявления константы. Полностью best-effort: таймаут/любая ошибка
        чтения — это НОРМА (большинство успешных admin-команд scsynth не
        подтверждает вовсе), а не повод мешать вызывающему коду.
        """
        try:
            sock.settimeout(self.OSC_REPLY_TIMEOUT_SECONDS)
        except Exception:  # noqa: BLE001 — best-effort, не мешаем вызывающему коду
            return
        data = self._pump_one_message(sock)
        if data is None:
            return
        try:
            self._route_osc_reply(data, when_iso=None)
        except Exception:  # noqa: BLE001
            pass

    def _pump_one_message(self, sock: "socket.socket") -> Optional[bytes]:
        """Один проход чтения из UDP-сокета; вернуть payload или sentinel ``None``.

        Возвращает:
          * ``bytes`` — успешно прочитанный один OSC-пакет;
          * ``None``  — ``OSError`` (сокет закрыт / пересоздан — поток
                        слушателя должен тихо завершиться) или любой
                        единичный сбой (мусорный пакет) — вызывающий
                        решает, продолжать ли цикл.

        Буфер 4096 байт — ``OSC_REPLY_MAX_BYTES``, см. обоснование у
        объявления константы. Никаких side-effects: ни логов, ни
        dispatch'а ответов — это задача ``_route_osc_reply``.
        """
        try:
            data, _addr = sock.recvfrom(self.OSC_REPLY_MAX_BYTES)
            return data
        except OSError:
            return None
        except Exception:  # noqa: BLE001 — единичный кривой пакет не должен убивать поток
            return None

    def _is_scsynth_synthdef_log(self, reply: bytes) -> bool:
        """Предикат: этот OSC-ответ — отказ из-за ненайденного SynthDef?

        scsynth отдаёт «SynthDef not found» именно как ``/fail`` с тремя
        строковыми аргументами вида ``["/s_new", "SynthDef not found",
        "<имя>"]`` (см. ``docker/.../foxdot_init.sc`` комментарий строки
        27: «FAILURE IN SERVER /s_new SynthDef not found»). Это самое
        частое из «трёх тишины» в нашем логе инцидентов — выделяем его
        в отдельный канал, чтобы в логе было сразу видно «синтезатор не
        загружен», без разбора неструктурированного ``detail``.

        Чистый предикат (только OSC-байты → bool), CC≤12.
        """
        address, rest = _split_osc_address(reply)
        if address != "/fail":
            return False
        args = _decode_osc_args(rest)
        return any("SynthDef" in str(a) for a in args)

    def _route_osc_reply(self, reply: bytes, when_iso: Optional[str]) -> None:
        """Диспетчер одного OSC-ответа scsynth: synthdef-fail / /fail / noop.

        Единственная точка принятия решения «логировать или нет». Вызывается
        из обоих источников (фоновый поток + короткий таймаут после
        собственного ``sendto``) — раньше эта логика была размазана между
        ``_renardo_reply_listener_loop`` и ``_log_scsynth_reply_if_any``
        прямой цепочкой ``→ _log_osc_reply``, что мешало расширять (issue
        #1808 follow-up).

        Категории:
          * synthdef-fail (``/fail`` + ``"SynthDef"`` в тексте) — отдельный
            лог-канал для быстрой диагностики «не загружен синтезатор»;
          * ``/fail`` (прочие — «too many nodes», «Group N not found») —
            стандартный ``🔴 FAILURE IN SERVER``;
          * остальное (``/done``, ``/synced`` и т.п.) — подавляем: шум.

        ``when_iso`` — зарезервировано для follow-up «безопасная привязка
        ответа к вызову тула по времени» (issue #1808 §follow-up). Пока
        не используется — потребителей нет, ложные срабатывания хуже
        молчания. Параметр в сигнатуре, чтобы будущий код не правил
        вызывающие сайты.
        """
        if self._is_scsynth_synthdef_log(reply):
            address, rest = _split_osc_address(reply)
            args = _decode_osc_args(rest)
            detail = " ".join(str(a) for a in args) if args else rest.decode("utf-8", "replace")
            self._log_warning(f"🎹 [scsynth] SynthDef FAILURE: {detail}")
            return
        address, rest = _split_osc_address(reply)
        if address != "/fail":
            return
        args = _decode_osc_args(rest)
        detail = " ".join(str(a) for a in args) if args else rest.decode("utf-8", "replace")
        self._log_warning(f"🔴 [scsynth] FAILURE IN SERVER: {detail}")

    def _log_osc_reply(self, data: bytes) -> None:
        """Тонкая обёртка над ``_route_osc_reply`` (для обратной совместимости).

        Исторический entry-point, на который завязан тонкий proxy из
        ``MusicManager`` (``mgr._renardo._log_osc_reply`` через ``setattr``)
        и существующие тесты ``test_music.py::test_log_osc_reply_*``. Сама
        логика — в ``_route_osc_reply`` (issue #1808 follow-up #3371).
        """
        self._route_osc_reply(data, when_iso=None)

    def _ensure_renardo_available(self) -> bool:
        """Retry Renardo initialization when a previous startup attempt failed.

        This avoids a permanent degraded state when container startup races cause
        the first one-shot initialization to fail before scsynth/sclang are fully ready.
        """
        if self._renardo_available:
            return True

        self._initialize_renardo()
        return bool(self._renardo_available)

    # ------------------------------------------------------------------
    # Live-инцидент 21.09.2026 — реально загруженные SynthDef-ы (для
    # валидации имён синтов в renardo_sanitizer._validate_synth_names)
    # ------------------------------------------------------------------

    def known_synth_names(self) -> Optional[frozenset]:
        """Множество SynthDef-имён, реально загруженных в scsynth.

        Issue #2838 (живой прогон 23.09.2026): раньше сюда шёл весь
        ``self._synthdefs_added`` — то, что Python-сторона renardo
        ОТПРАВИЛА через ``sdef.add()`` (UDP ``/foxdot`` → sclang, без
        подтверждения). Часть этих пакетов теряется на порту sclang
        (drops в ``/proc/net/udp``), и потерянный ``sine`` остался
        «известным»: валидатор сам подсказал его LLM, та им сыграла —
        235 × "SynthDef sine not found".

        Теперь источник истины — ``self._server_confirmed_synths``: имена,
        которые sclang подтвердил в scsynth строкой прелоада "SynthDef in
        scsynth: X" (печатается после ``Server.sync``, см.
        ``foxdot_init.sc``). Пересекаем его с тем, для чего есть
        Python-обёртка (``_synthdefs_added`` ∪ ``CUSTOM_SC_ONLY_SYNTH_NAMES``):
        синт без обёртки код всё равно не вызовет. Это же отсекает
        служебные шины ``masterlimiter``/``masterfilter`` — они есть на
        сервере, но не тембры для ``lead_synth``/``bass_synth``/``pad_synth``.

        Если подтверждения нет (sclang-лог недоступен / прелоад не
        завершён) — прежнее поведение: ``_synthdefs_added`` ∪
        ``CUSTOM_SC_ONLY_SYNTH_NAMES``. Оно НЕ проверено сервером; на
        старте это логируется (``_log_synth_truth_discrepancy``).

        Returns:
            ``None``, пока ``_synthdefs_added`` пуст (Renardo ещё не
            инициализирован, или тест создал ``MusicRenardoBridge`` через
            ``__new__`` в обход ``__init__``) — вызывающая сторона должна
            трактовать это как «набор неизвестен», а не «ничего не
            разрешено», иначе валидатор блокировал бы ЛЮБОЙ синт до
            завершения инициализации. Иначе — frozenset имён (нижний
            регистр — как их печатает sclang).
        """
        added = getattr(self, "_synthdefs_added", None)
        if not added:
            return None
        wrapped = frozenset(added) | frozenset(CUSTOM_SC_ONLY_SYNTH_NAMES)
        confirmed = getattr(self, "_server_confirmed_synths", None)
        if confirmed is None:
            return wrapped
        return wrapped & confirmed

    def _log_synth_truth_discrepancy(self) -> None:
        """Issue #2838: залогировать расхождение «отправлено» vs «на сервере».

        Вызывается один раз в конце успешного ``_initialize_renardo``.
        Ничего не меняет — только делает видимым, какие синты Python-сторона
        считает добавленными, но sclang не подтвердил в scsynth (валидатор
        их отклоняет и не подсказывает).
        """
        confirmed = getattr(self, "_server_confirmed_synths", None)
        if confirmed is None:
            self._log_warning(
                "[music #2838] нет подтверждения прелоада SynthDef-ов в "
                "sclang-логе — валидатор синтов работает по списку "
                "ОТПРАВЛЕННЫХ (sdef.add()), он не проверен сервером"
            )
            return
        unconfirmed = sorted(set(self._synthdefs_added) - confirmed)
        wrapped = set(self._synthdefs_added) | set(CUSTOM_SC_ONLY_SYNTH_NAMES)
        no_wrapper = sorted(confirmed - wrapped)
        sent = len(self._synthdefs_added)
        allowed = len(self.known_synth_names() or ())
        self._log_warning(
            f"[music #2838] SynthDef truth: подтверждено в scsynth "
            f"{len(confirmed)}, отправлено renardo {sent}, "
            f"разрешено валидатору {allowed}; "
            f"без подтверждения ({len(unconfirmed)}, отклоняются): "
            f"{unconfirmed}; на сервере без Python-обёртки: {no_wrapper}"
        )

    # ------------------------------------------------------------------
    # Music-stack health (issue G-MUSIC, architect review v3)
    # ------------------------------------------------------------------

    def _evaluate_music_stack_health(
        self,
        sclang_log_path: Optional[str] = None,
    ) -> MusicStackStatus:
        """Snapshot sclang health from the startup log and mark the bridge.

        When ``is_healthy is False`` AND ``_require_healthy`` is True, this
        will also clear ``_renardo_available`` (without touching
        ``_renardo_last_error``) so downstream tools see consistent state.

        Args:
            sclang_log_path: Override log location. Falls back to
                ``SCLANG_LOG_PATH`` env var, then ``/tmp/sclang.log``.

        Returns:
            The :class:`MusicStackStatus` that was applied.
        """

        status = load_sclang_health(
            sclang_log_path,
            critical_synths=list(self._critical_synths),
        )
        self._music_stack_status = status
        self._server_confirmed_synths = load_confirmed_synths(sclang_log_path)

        if not status.is_healthy and self._require_healthy:
            # Mark Renardo as unavailable WITHOUT clearing the existing
            # last_error (which might be informative for diagnostics). The
            # operator should see both "music stack degraded" AND any
            # subsequent renardo init failure that follows.
            self._renardo_available = False

        return status

    def is_music_stack_healthy(self) -> bool:
        """True if the sclang startup log was healthy at the last check."""

        return bool(self._music_stack_status.is_healthy)

    def music_stack_unavailable_error(self) -> Dict[str, str]:
        """Build a stable error payload for ``music unavailable`` replies.

        Used by ``execute_code`` / ``set_vibe_preset`` / ``stop_music`` so
        the LLM gets a single, recognizable error message rather than
        a different string for each entry-point.
        """

        status = self._music_stack_status
        details: List[str] = []
        if status.fatal_errors:
            details.append("; ".join(status.fatal_errors[:3]))
        if status.missing_synths:
            details.append(f"missing SynthDefs: {', '.join(status.missing_synths)}")
        detail_str = (" — " + "; ".join(details)) if details else ""
        log_path = os.environ.get("SCLANG_LOG_PATH", "/tmp/sclang.log")
        return {
            "success": False,
            "error": (
                "Музыка недоступна: sclang стартовал в degraded-режиме "
                "(syntax error в startup-логе Renardo/FoxDot)"
                f"{detail_str}. См. {log_path}."
            ),
        }

    # ------------------------------------------------------------------
    # SuperCollider check
    # ------------------------------------------------------------------

    def _check_supercollider(self) -> bool:
        """Проверить, запущен ли SuperCollider, отправив OSC /status по UDP.

        scsynth слушает на UDP-порту SC_PORT. Отправляем минимальный OSC
        /status запрос и ждём ответа. TCP-проверка не подходит — scsynth
        по умолчанию принимает только UDP.

        Returns:
            True если scsynth отвечает на SC_PORT.
        """
        # Минимальный OSC /status: "/status\0" (8 байт) + ",\0\0\0" (4 байта)
        osc_status = b"/status\x00,\x00\x00\x00"
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.settimeout(1.0)
                sock.sendto(osc_status, (self.SC_HOST, self.SC_PORT))
                data, _ = sock.recvfrom(512)
                return len(data) > 0
        except OSError:
            return False

    def _send_osc_raw(self, address: str, *args: Any) -> None:
        """Отправить raw OSC сообщение на scsynth (UDP 57110).

        OSC-packet собирается вручную: 4-byte aligned address + type-tag +
        big-endian args. Поддерживает int (``i``), float (``f``) и
        string (``s``) аргументы. Строки нужны для ``/n_set <node>
        <control-name> <value>`` (мастер-фейдер лимитера): имя контрола
        обязано ехать как ``s``, иначе scsynth читает первые 4 байта имени
        как int32 и молча игнорирует установку — ровно та же ловушка, что
        описана в ``_verify_and_retry_synthdefs`` про ``"amp"``.

        Выделено как self-метод вместо замыкания, чтобы можно было
        переиспользовать из ``execute_music_code`` / ``stop_all`` без
        дублирования byte-packing логики. Раньше ``execute_music_code``
        собирал ``/g_new`` руками bytearray-ом, что (а) дублировало код
        и (б) легко ломалось при изменении формата.

        Issue #778 (deployment critical_log ``FAILURE IN SERVER /g_new
        negative node IDs are reserved``): между ``/g_freeAll`` и
        ``/g_new`` нужна пауза ≥50ms — UDP fire-and-forget, scsynth не
        успевает освободить ID Group 1, и Renardo Player-ы присылают
        ``/s_new`` с target_id=1, которого ещё нет. Пауза лечит race
        condition без изменения семантики (свободные ноды умирают
        сами, мы просто даём scsynth обработать free до пересоздания
        Group).

        🔴 FIX (issue #1808): раньше сокет закрывался сразу после
        ``sendto`` (``with`` выходил из блока) — если scsynth отвечал
        ``/fail`` (например «Group 1 not found»), ответ прилетал уже на
        закрытый сокет и терялся молча. Теперь перед закрытием кратко
        слушаем этот же сокет (``_log_scsynth_reply_if_any``) — см.
        обоснование таймаута у ``OSC_REPLY_TIMEOUT_SECONDS``.
        """
        msg = bytearray()
        addr_bytes = address.encode() + b"\x00"
        while len(addr_bytes) % 4:
            addr_bytes += b"\x00"

        def _tag(value: Any) -> bytes:
            if isinstance(value, str):
                return b"s"
            # bool is a subclass of int — проверяем int после str, как раньше.
            return b"i" if isinstance(value, int) else b"f"

        types = b"," + b"".join(_tag(a) for a in args) + b"\x00"
        while len(types) % 4:
            types += b"\x00"
        msg.extend(addr_bytes)
        msg.extend(types)
        for a in args:
            if isinstance(a, str):
                blob = a.encode() + b"\x00"
                while len(blob) % 4:
                    blob += b"\x00"
                msg.extend(blob)
            elif isinstance(a, int):
                msg.extend(struct.pack(">i", a))
            elif isinstance(a, float):
                msg.extend(struct.pack(">f", a))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(bytes(msg), (self.SC_HOST, self.SC_PORT))
            # Issue #1808 — см. docstring выше и обоснование у
            # OSC_REPLY_TIMEOUT_SECONDS. Best-effort, никогда не бросает.
            self._log_scsynth_reply_if_any(sock)