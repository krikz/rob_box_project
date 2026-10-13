"""Звуковой сервер для движка v2: Renardo, scsynth, палитра синтов, мастер-фейдер (#3614, ADR-0149 PR-13).

Раньше всё это поднимал ``MusicManager`` старого пути (``tools/music.py``) — вместе с исполнением кода LLM,
историей паттернов и сторожем сессии. Движку v2 из него нужны были только: контекст Renardo (``Clock``,
``Samples``, плееры), палитра синтов, подтверждённая сервером, отправка raw OSC в scsynth, слушатель ``/fail``
и уровень мастер-фейдера. Это и есть :class:`RenardoRuntime`; остальное удалено (#3614).

Что здесь делается при старте (порядок важен, каждый шаг — живой инцидент):

1. каталоги сэмплов ``0_foxdot_default`` (иначе ``renardo_lib.runtime`` падает при импорте), докопировка
   вокальных сэмплов ``c`` из ``1_pitchglitch_samples``;
2. ``Server.init_connection()`` и фоновый слушатель ответов scsynth на сокете Renardo (#1808: Renardo шлёт
   ноты fire-and-forget и ответы не читает — «SynthDef not found» видели только в логе supercollider);
3. ``/g_new 1`` — группа, в которую Renardo шлёт ноты (без неё «Group 1 not found» на каждый ``/s_new``);
4. ``sdef.add()`` всех SynthDef с паузой 0.1 с на каждые 5 (live 12.08: залп рвал UDP-буфер sclang),
   эффекты ``effect_manager.reload()`` (live 05.08: без них ``reverb``/``volume`` not found — тишина),
   5 с на компиляцию, проба критичных синтов ``/s_new`` с досылкой пропавших (до 3 раундов);
5. подтверждённая палитра — строки прелоада ``SynthDef in scsynth: X`` из лога sclang (#2838).
"""

from __future__ import annotations

import socket
import struct
import sys
import threading
import time
from typing import Any, Callable, Dict, Optional

from rob_box_voice.core.music_stack_validation import load_confirmed_synths
from rob_box_voice.core.sc_only_custom_synthdefs import (
    CUSTOM_SC_ONLY_SYNTH_NAMES,
    register_sc_only_custom_synthdefs,
)

from . import renardo_adapter

#: Синты, которые проверяются ``/s_new`` после прелоада и досылаются, если пропали (live 12.08, 30.08: синт
#: из палитры не доехал до scsynth — трек без темы). Палитра движка v2 (ADR-0152 PR-6) — в конце.
CRITICAL_SYNTHS: tuple = (
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
    # прочие тембры
    "bass", "gong", "pluck", "saw", "square", "faim", "viola",
    "noise", "scatter", "orient", "creep", "play1", "play2",
    # палитра движка v2 (ADR-0152 PR-6): лиды и бас семей тембров, в прелоаде робота
    "kalimba", "hoover", "keys", "jbass",
)

#: Уровень мастер-фейдера ``masterfilter`` (узел 999) по умолчанию — ровно дефолт ``gain`` в
#: ``masterfilter.scd`` и ROS-параметра ``music_master_gain`` (issue #986: громкость музыки под речь).
DEFAULT_MASTER_GAIN = 0.5


def osc_message(address: str, *args: Any) -> bytes:
    """OSC-пакет: адрес и type-tag с выравниванием до 4 байт, аргументы big-endian.

    ``str`` → ``s``, ``int`` → ``i`` (``bool`` тоже int), иначе ``f``. Имя контрола в ``/n_set`` обязано ехать
    как ``s``: как int32 scsynth молча игнорирует установку (#1444, свист пробных нод).
    """
    def pad(blob: bytes) -> bytes:
        return blob + b"\x00" * (-len(blob) % 4)

    def tag(value: Any) -> bytes:
        if isinstance(value, str):
            return b"s"
        return b"i" if isinstance(value, int) else b"f"

    msg = bytearray(pad(address.encode() + b"\x00"))
    msg.extend(pad(b"," + b"".join(tag(a) for a in args) + b"\x00"))
    for value in args:
        if isinstance(value, str):
            msg.extend(pad(value.encode() + b"\x00"))
        elif isinstance(value, int):
            msg.extend(struct.pack(">i", value))
        else:
            msg.extend(struct.pack(">f", float(value)))
    return bytes(msg)


class RenardoRuntime:
    """Renardo и scsynth для владельца плеера v2: контекст, палитра, raw OSC, ``/fail``, мастер-фейдер.

    Args:
        master_gain: стартовый уровень мастер-фейдера (ROS-параметр ``music_master_gain``).
        sclang_log_path: лог sclang с подтверждением прелоада (по умолчанию ``SCLANG_LOG_PATH`` или
            ``/tmp/sclang.log``).
        boot: поднять Renardo сразу (тесты передают ``False``).
    """

    SC_HOST: str = "127.0.0.1"
    SC_PORT: int = 57110
    #: После своей админ-команды кратко слушаем сокет на ``/fail`` (#1808). Успешный ``/n_set`` scsynth не
    #: подтверждает — таймаут оплачивается целиком, поэтому он маленький (на loopback ответ — микросекунды).
    OSC_REPLY_TIMEOUT_SECONDS: float = 0.03
    MASTER_NODE: int = renardo_adapter.MASTER_NODE

    def __init__(self, master_gain: Optional[float] = None, *, sclang_log_path: Optional[str] = None,
                 boot: bool = True) -> None:
        self._master_gain = max(0.0, min(1.0, DEFAULT_MASTER_GAIN if master_gain is None else float(master_gain)))
        self._sclang_log_path = sclang_log_path
        self._context: Dict[str, Any] = {}
        self._available = False
        self._last_error: Optional[str] = None
        #: SynthDef-ы, уже отправленные ``sdef.add()``: повторный ``add()`` мутирует UGen-граф (live 20.08).
        self._synthdefs_added: set = set()
        #: Подтверждённые sclang синты (#2838); ``None`` — подтверждения нет.
        self._server_confirmed: Optional[frozenset] = None
        self._reply_sock: Optional[socket.socket] = None
        self._boot_lock = threading.Lock()
        self._logger: Any = None
        #: Слушатель ``/fail`` scsynth — владелец плеера v2 (``PlayerOwner.on_server_fail``).
        self.osc_fail_listener: Optional[Callable[[str], None]] = None
        if boot:
            self.boot()

    # ------------------------------------------------------------------ состояние

    @property
    def available(self) -> bool:
        """Renardo поднят: есть ``Clock``/``Samples``, SynthDef-ы отправлены."""
        return self._available

    @property
    def last_error(self) -> Optional[str]:
        """Почему Renardo не поднялся (для лога и честного ответа о состоянии)."""
        return self._last_error

    def context(self) -> Dict[str, Any]:
        """Пространство имён Renardo (``Clock``, ``Samples``, плееры).

        Первый подъём упал (гонка старта контейнера: scsynth/sclang ещё не готовы) — пробуем снова здесь, на
        следующем заказе музыки, а не остаёмся без звука до рестарта.
        """
        if not self._available:
            self.boot()
        return self._context

    def attach_logger(self, logger: Any) -> None:
        """Логгер ноды: строки из потока клока Renardo иначе ушли бы только в stderr (#3166)."""
        self._logger = logger

    def _log(self, level: str, message: str) -> None:
        logger = self._logger
        if logger is not None:
            try:
                getattr(logger, level)(message)
                return
            except Exception:  # noqa: BLE001
                pass
        sys.stderr.write(f"{message}\n")
        sys.stderr.flush()

    # ------------------------------------------------------------------ подъём

    def boot(self) -> bool:
        """Поднять Renardo (см. модульный докстринг); ``True`` — поднят. Повторный вызов после успеха — no-op."""
        with self._boot_lock:
            if self._available:
                return True
            self._server_confirmed = load_confirmed_synths(self._sclang_log_path)
            try:
                rt = self._import_runtime()
                if not rt.Server.booted:
                    rt.Server.init_connection()
                self._attach_reply_listener(rt)
                self.send_osc("/g_new", 1, 0, 0)
                self._load_synthdefs(rt)
                self._context = vars(rt).copy()
                register_sc_only_custom_synthdefs(rt, self._context)
            except Exception as exc:  # noqa: BLE001 — без звука, но нода живёт; причина — в лог и в состояние
                self._available, self._context, self._last_error = False, {}, f"{type(exc).__name__}: {exc}"
                self._log("error", f"❌ [music] Renardo не поднялся: {self._last_error}")
                return False
            self._available, self._last_error = True, None
        self._log_synth_truth()
        self.set_master_gain(self._master_gain)
        return True

    @staticmethod
    def _import_runtime() -> Any:
        """Каталоги сэмплов до импорта ``renardo_lib.runtime`` (он их листит при импорте)."""
        import pathlib
        import shutil

        samples = pathlib.Path.home() / ".config" / "renardo" / "samples"
        base = samples / "0_foxdot_default"
        for subdir in ["_", "_loop_"] + list("abcdefghijklmnopqrstuvwxyz"):
            (base / subdir).mkdir(parents=True, exist_ok=True)
        # Renardo ищет сэмплы только в 0_foxdot_default; буквы 'c' (вокал) там нет — берём из pitchglitch.
        pitchglitch = samples / "1_pitchglitch_samples"
        if pitchglitch.exists():
            for letter in "abcdefghijklmnopqrstuvwxyz":
                for case_dir in ("lower", "upper"):
                    src, dst = pitchglitch / letter / case_dir, base / letter / case_dir
                    if not src.exists():
                        continue
                    dst.mkdir(parents=True, exist_ok=True)
                    have = {f.name for f in dst.glob("*.wav")}
                    for wav in src.glob("*.wav"):
                        if wav.name not in have:
                            shutil.copy2(wav, dst / wav.name)
        import renardo_lib.runtime as rt

        return rt

    def _load_synthdefs(self, rt: Any) -> None:
        """``sdef.add()`` с пейсингом, эффекты, 5 с на компиляцию, проба и досылка критичных синтов."""
        for idx, (name, sdef) in enumerate(rt.SynthDefs.items()):
            if name in self._synthdefs_added:
                continue
            sdef.add()
            self._synthdefs_added.add(name)
            if idx % 5 == 4:
                time.sleep(0.1)
        try:
            rt.effect_manager.reload()
        except Exception as exc:  # noqa: BLE001
            self._log("warning", f"⚠️ [music] effect_manager.reload упал: {exc}")
        time.sleep(5)
        self._resend_missing(rt)

    def _probe_missing(self, names: Any) -> list:
        """Синты из ``names``, которых нет в scsynth: пробный ``/s_new`` с ``amp 0`` и сразу ``/n_free``.

        Пропавшим считается только настоящий «SynthDef X not found» (не «Command not found»: адрес ``/s_new``
        обязан быть выровнен, live 13.08). Тег ``,siiisf``: имя контрола ``amp`` — строка (#1444, свист).
        """
        missing: list = []
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(0.4)
            for i, name in enumerate(names):
                node_id = 9000 + i
                try:
                    sock.sendto(osc_message("/s_new", name, node_id, 0, 1, "amp", 0.0), (self.SC_HOST, self.SC_PORT))
                    data, _ = sock.recvfrom(512)
                    if b"SynthDef" in data and b"not found" in data:
                        missing.append(name)
                        continue
                except socket.timeout:
                    pass  # нет ответа — нода создана
                except OSError:
                    continue
                try:
                    sock.sendto(osc_message("/n_free", node_id), (self.SC_HOST, self.SC_PORT))
                except OSError:
                    pass
        return missing

    def _resend_missing(self, rt: Any, max_rounds: int = 3) -> None:
        """До ``max_rounds`` раундов: проба критичных синтов и досылка пропавших (live 12.08: UDP drops)."""
        missing = list(CRITICAL_SYNTHS)
        for round_no in range(max_rounds):
            missing = self._probe_missing(missing)
            if not missing:
                return
            self._log("warning", f"[music] round {round_no + 1}: missing SynthDefs: {missing} — re-sending")
            for name in missing:
                sdef = rt.SynthDefs.get(name)
                if sdef is None:
                    continue
                try:
                    if name in self._synthdefs_added:
                        load = getattr(sdef, "load", None)  # повторный add() мутирует граф — шлём готовый .scd
                        if load is not None:
                            load()
                    else:
                        sdef.add()
                        self._synthdefs_added.add(name)
                except Exception:  # noqa: BLE001
                    continue
                time.sleep(0.3)
            time.sleep(5)
        self._log("warning", f"[music] SynthDefs still missing after {max_rounds} rounds: {missing}")

    # ------------------------------------------------------------------ палитра

    def known_synth_names(self) -> Optional[frozenset]:
        """Синты, которые реально есть на сервере и у которых есть Python-обёртка (#2838).

        Источник истины — подтверждение sclang (``SynthDef in scsynth: X``) ∩ (отправленные ``sdef.add()`` ∪
        SC-only синты). Подтверждения нет — отправленные без проверки сервером (лог при старте). ``None`` —
        Renardo ещё не поднят: «палитра неизвестна», а не «ничего нельзя».
        """
        if not self._synthdefs_added:
            return None
        wrapped = frozenset(self._synthdefs_added) | frozenset(CUSTOM_SC_ONLY_SYNTH_NAMES)
        confirmed = self._server_confirmed
        return wrapped if confirmed is None else wrapped & confirmed

    def _log_synth_truth(self) -> None:
        """#2838: расхождение «отправлено renardo» и «подтверждено в scsynth» — в лог, один раз при старте."""
        confirmed = self._server_confirmed
        if confirmed is None:
            self._log("warning", "[music #2838] нет подтверждения прелоада SynthDef-ов в sclang-логе — палитра "
                                 "движка v2 = ОТПРАВЛЕННЫЕ (sdef.add()), сервером не проверена")
            return
        sent = set(self._synthdefs_added)
        unconfirmed = sorted(sent - confirmed)
        no_wrapper = sorted(confirmed - sent - set(CUSTOM_SC_ONLY_SYNTH_NAMES))
        self._log("warning", f"[music #2838] SynthDef truth: подтверждено в scsynth {len(confirmed)}, отправлено "
                             f"renardo {len(sent)}, в палитре {len(self.known_synth_names() or ())}; без "
                             f"подтверждения ({len(unconfirmed)}): {unconfirmed}; без Python-обёртки: {no_wrapper}")

    # ------------------------------------------------------------------ OSC

    def send_osc(self, address: str, *args: Any) -> None:
        """Raw OSC в scsynth (UDP 57110) и короткое ожидание ``/fail`` на том же сокете (#1808)."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(osc_message(address, *args), (self.SC_HOST, self.SC_PORT))
            try:
                sock.settimeout(self.OSC_REPLY_TIMEOUT_SECONDS)
                data, _ = sock.recvfrom(4096)
            except Exception:  # noqa: BLE001 — таймаут: scsynth принял молча (норма)
                return
            self._on_reply(data)

    def _attach_reply_listener(self, rt: Any) -> None:
        """Фоновый ``recv`` на собственном сокете Renardo (``Server.client.socket``) — best-effort.

        Renardo сокет только пишет (``sendall``), поэтому чтение в отдельном потоке с ним не гонится. Другая
        версия Renardo без этого графа объектов — просто нет источника, музыка не страдает.
        """
        try:
            sock = getattr(getattr(getattr(rt, "Server", None), "client", None), "socket", None)
            if not isinstance(sock, socket.socket) or sock is self._reply_sock:
                return
            self._reply_sock = sock
            threading.Thread(target=self._reply_loop, args=(sock,), name="scsynth-reply-listener",
                             daemon=True).start()
        except Exception as exc:  # noqa: BLE001 — слушатель не держит подъём Renardo
            self._log("warning", f"⚠️ [music] слушатель ответов scsynth не подключён: {exc}")

    def _reply_loop(self, sock: socket.socket) -> None:
        while True:
            try:
                data, _ = sock.recvfrom(4096)
            except OSError:
                return  # сокет закрыт (Renardo пересоздал клиента) — поток завершается сам
            except Exception:  # noqa: BLE001 — кривой пакет не убивает поток
                continue
            try:
                self._on_reply(data)
            except Exception:  # noqa: BLE001
                continue

    def _on_reply(self, data: bytes) -> None:
        """``/fail`` scsynth — в лог и владельцу плеера (он свяжет отказ с ``track_id``); ``/done`` — норма."""
        detail = renardo_adapter.osc_fail_detail(data)
        if detail is None:
            return
        self._log("warning", f"🔴 [scsynth] FAILURE IN SERVER: {detail}")
        listener = self.osc_fail_listener
        if callable(listener):
            listener(detail)

    # ------------------------------------------------------------------ мастер-фейдер

    @property
    def master_gain(self) -> float:
        """Текущий уровень мастер-фейдера (0.0–1.0), #3125."""
        return self._master_gain

    def set_master_gain(self, gain: float) -> float:
        """``/n_set 999 gain <v>`` — единственная ручка громкости музыки под речь (#986, #3125).

        Фейдер стоит после динамики ``masterfilter`` и сглажен ``Lag`` — без щелчка. Узла нет (образ без
        ``masterfilter``) — scsynth ответит ``/fail``, музыка играет без фейдера. Возвращает применённое значение.
        """
        self._master_gain = max(0.0, min(1.0, float(gain)))
        try:
            self.send_osc("/n_set", self.MASTER_NODE, "gain", self._master_gain)
        except OSError as exc:
            self._log("warning", f"⚠️ [music] master gain не применён: {exc}")
        return self._master_gain


__all__ = ["CRITICAL_SYNTHS", "DEFAULT_MASTER_GAIN", "RenardoRuntime", "osc_message"]
