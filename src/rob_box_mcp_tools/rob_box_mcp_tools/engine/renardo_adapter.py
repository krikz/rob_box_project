"""Адаптер Renardo для владельца плеера v2 (ADR-0149 §2.2, §4.3; эпик #3312, PR-4).

Старт программы на границе формы с фазы 0, следующий трек сета на другой деке: встык на границе
формы уходящего (PR-5) или блэндом — входящий встаёт за ``overlap`` долей до неё, уходящая дека
снимается на границе и освобождается (PR-8); проверка ресурсов до exec, спуск
``gate=0 → /g_freeAll``. Сам Renardo (контекст, сокет к scsynth, палитра синтов,
подтверждённая сервером) по-прежнему поднимает ``MusicManager``: адаптер получает его
пространство имён, ``known_synth_names`` и ``_send_osc_raw`` как функции, второй
инициализации Renardo нет.

Здесь же живут общие со старым путём куски (старое место — импорт отсюда, ADR-0149 §8.1):
разбор ответа scsynth ``/fail`` (#1808), спуск группы (#3137), загрузка буферов сэмплов (#1815).

Темп: ``Clock.update_tempo_now(bpm)``. Пара ``Clock.bpm = N`` + ``Clock.set_time`` теряет
смену темпа: ``update_tempo`` откладывает её на следующий такт, а ``set_time`` сбрасывает
``bpm_start_time`` (``renardo_lib/TempoClock.py:218,420``; найдено в PR-2, #3329).
``Clock.clear()`` не вызывается (ADR-0149 §3.2).
"""

from __future__ import annotations

import struct
import time
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Tuple

from ..core.arranger import ALIGN_LEAD_BEATS

#: ``Clock.latency`` при v2 (#3328): на живом роботе 01.10 опоздавших бандлов в scsynth
#: 250/мин при 0.25 с и 2/мин при 0.5 с (PR-2, #3329). Цена — реакция на 0.25 с позже.
V2_CLOCK_LATENCY_S = 0.5

#: Пауза между ``gate=0`` и ``/g_freeAll`` (#3137, как у #1000 в ``stop_all``).
RAMP_DOWN_RELEASE_SECONDS = 0.05

#: Уходящая дека снимается за 1/32 до доли стыка: её события на самой доле (повтор формы —
#: бочка, аккорд пэда) не звучат поверх входящего трека (PR-5).
HANDOFF_STOP_BEATS = 0.125

#: Проблема с ресурсом до exec: ``(reason, detail)``.
Problem = Tuple[str, str]


def split_osc_address(data: bytes) -> Tuple[Optional[str], bytes]:
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


def _decode_one(tag: str, rest: bytes, offset: int) -> Tuple[Any, int]:
    """Один аргумент OSC; ``(None, -1)`` — дальше не разбираем."""
    if tag in ("i", "f"):
        if offset + 4 > len(rest):
            return None, -1
        return struct.unpack(">" + tag, rest[offset:offset + 4])[0], offset + 4
    if tag == "s":
        str_end = rest.find(b"\x00", offset)
        if str_end == -1:
            return None, -1
        value = rest[offset:str_end].decode("utf-8", "replace")
        offset = str_end + 1
        while offset % 4:
            offset += 1
        return value, offset
    # blob (b) и прочие типы не разбираем — для лога достаточно накопленного.
    return None, -1


def decode_osc_args(rest: bytes) -> List[Any]:
    """Разобрать OSC type-tag строку (``,ssif``...) и аргументы за ней."""
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
        value, offset = _decode_one(tag, rest, offset)
        if offset < 0:
            break
        args.append(value)
    return args


def osc_fail_detail(data: bytes) -> Optional[str]:
    """Текст отказа scsynth, если пакет — ``/fail``; иначе ``None`` (``/done`` и пр. — норма)."""
    address, rest = split_osc_address(data)
    if address != "/fail":
        return None
    args = decode_osc_args(rest)
    return " ".join(str(a) for a in args) if args else rest.decode("utf-8", "replace")


def ramp_down_group(send_osc: Callable[..., None], group: int = 1,
                    release_s: float = RAMP_DOWN_RELEASE_SECONDS) -> None:
    """``/n_set <group> gate 0`` → пауза → ``/g_freeAll <group>`` (anti-click, #1000/#3137).

    ``gate`` есть у меньшинства синтов (~60 из ~370), у остальных ``/n_set`` — no-op; главное —
    когда звать ``freeAll`` (подробно — ``MusicManager._ramp_down_group``). Best-effort: SC
    недоступен — старые ноды умрут сами.
    """
    try:
        send_osc("/n_set", group, "gate", 0.0)
    except Exception:  # noqa: BLE001
        pass
    try:
        time.sleep(release_s)
    except Exception:  # noqa: BLE001
        pass
    try:
        send_osc("/g_freeAll", group)
    except Exception:  # noqa: BLE001
        pass


def load_sample_buffers(samples: Any, symbols: Iterable[str]) -> List[str]:
    """Загрузить буферы символов ``play()`` до exec; вернуть символы без сэмпла.

    ``"."`` — пауза, пробел — разделитель; ``"-"`` — звучащий хэт (#1815). ``"X:12"`` — символ с
    номером файла ``sample=`` (``Program.samples`` v2, бочка ``knowledge.KICK_SOUNDS``): грузится тот
    буфер, что прозвучит, а не нулевой. Renardo отдаёт буфер-заглушку ``nil`` с ``bufnum == 0``,
    когда файла нет (``BufferManagement.py:116,209``). Повторный вызов — попадание в кэш Renardo.
    """
    missing: List[str] = []
    for entry in symbols:
        symbol, _, index = entry.partition(":") if len(entry) > 2 and entry[1] == ":" else (entry, "", "")
        if symbol.isspace() or symbol == ".":
            continue
        try:
            args = (symbol, 0, int(index)) if index else (symbol, 0)  # (символ, spack, номер файла)
            buf = samples.getBufferFromSymbol(*args)
        except Exception:  # noqa: BLE001 — символ может не иметь сэмпла
            missing.append(entry)
            continue
        if getattr(buf, "bufnum", None) == 0:
            missing.append(entry)
    return missing


class RenardoAdapter:
    """Деки на живом Renardo: проверка ресурсов, старт с фазы 0, стык следующего трека, стоп.

    Args:
        namespace: функция → текущее пространство имён Renardo (``Clock``, ``Samples``, плееры).
        known_synths: функция → синты, подтверждённые сервером (``None`` — палитра неизвестна).
        send_osc: отправка raw OSC в scsynth (``MusicManager._send_osc_raw``).
        latency_s: ``Clock.latency`` на время v2.
        lead_beats: за сколько долей до границы формы ставится клок (плееры встают на такт).
    """

    def __init__(self, namespace: Callable[[], Mapping[str, Any]],
                 known_synths: Callable[[], Optional[frozenset]],
                 send_osc: Callable[..., None], *, latency_s: float = V2_CLOCK_LATENCY_S,
                 lead_beats: float = ALIGN_LEAD_BEATS) -> None:
        self._namespace = namespace
        self._known_synths = known_synths
        self._send_osc = send_osc
        self.latency_s = float(latency_s)
        self._lead_beats = float(lead_beats)
        self._live_slots: Tuple[str, ...] = ()
        self._leaving: Tuple[str, ...] = ()  # дека уходящего трека, пока идёт блэнд
        # Поколение: start/stop делают устаревшим всё, что раньше запланировано на клоке
        # (стык, nearly_finished) — после стопа запланированный стык музыку не воскрешает.
        self._generation = 0

    def check(self, program: Any) -> Optional[Problem]:
        """Синты и буферы программы есть на сервере? ``None`` — да; иначе ``(reason, detail)``."""
        ns = self._namespace() or {}
        if ns.get("Clock") is None or ns.get("Samples") is None:
            return "renardo_unavailable", "нет Clock/Samples в контексте Renardo"
        known = self._known_synths()
        if known is None:
            return "synth_palette_unknown", "палитра синтов сервера неизвестна"
        unknown = sorted(s for s in program.synths if s.lower() not in known or not callable(ns.get(s)))
        if unknown:
            return "unknown_synth", f"синтов нет на сервере: {', '.join(unknown)}"
        missing = load_sample_buffers(ns["Samples"], sorted(program.samples))
        if missing:
            return "missing_sample", f"нет сэмплов для символов: {' '.join(missing)}"
        return None

    def start(self, program: Any, on_started: Callable[[Dict[str, Any]], None]) -> Dict[str, Any]:
        """Исполнить программу так, чтобы форма началась с доли, кратной ``form_beats``.

        Старые плееры деки снимаются; клок ставится на ``k·form − lead`` (плееры нового трека
        встают на ``Clock.next_bar() = k·form``); ``on_started`` зовётся из потока клока на
        фактической доле старта плееров. Бросает исключение exec — владелец превратит его в
        ``rejected``. Темп ставится здесь, один раз на сет: следующие треки сета — :meth:`cue`.
        """
        ns = self._namespace()
        clock = ns["Clock"]
        self._generation += 1
        # Снять и прошлый трек, и слоты новой программы: играющий плеер на ``>>`` лишь меняет
        # атрибуты и продолжает с текущей доли — фаза не 0 (живой прогон 01.10: 126.25 из 128).
        self._stop_slots(ns, (*self._live_slots, *self._leaving, *program.slots.values()))
        self._live_slots, self._leaving = (), ()
        if getattr(clock, "now_flag", False):
            clock.now_flag = False  # иначе плееры встанут не на такт (#3166)
        clock.latency = self.latency_s
        clock.update_tempo_now(program.bpm)
        form = float(program.form_beats)
        now = float(clock.now())
        clock.set_time(((now + self._lead_beats) // form + 1) * form - self._lead_beats)
        exec(program.code, ns)  # noqa: S102 — программа v2 = вывод render(), не текст LLM
        self._live_slots = tuple(program.slots.values())
        return self._arm_started(ns, program, on_started, origin=None)

    def cue(self, program: Any, at_beat: float, on_started: Callable[[Dict[str, Any]], None],
            on_failed: Callable[[str, str], None], *, leave_at: Optional[float] = None,
            on_left: Optional[Callable[[], None]] = None) -> None:
        """Следующий трек сета: плееры программы встают ровно на долю ``at_beat`` (фаза 0).

        Встык (``leave_at`` нет) ``at_beat`` — граница формы играющего трека (ADR-0149 §4.3);
        блэнд (PR-8, §3.12) — ``at_beat`` за ``overlap`` долей до границы ``leave_at``: оба трека
        звучат вместе, своп баса и бочки — в самих формах (``model.blend_bars``). Программа
        исполняется в потоке клока за ``lead_beats`` до ``at_beat`` (``Clock.next_bar()`` =
        ``at_beat``) на другой деке; плееры уходящего снимаются за ``HANDOFF_STOP_BEATS`` до
        ``leave_at`` (его повтор формы не звучит), дека свободна — ``on_left``. Без ``set_time``
        и без смены темпа: темп один на сет (§4.4). Ошибка exec → ``on_failed``, уходящий трек
        при этом играет дальше (не тишина).
        """
        ns = self._namespace()
        clock = ns["Clock"]
        generation = self._generation

        def _rbx_cue() -> None:
            if generation != self._generation:
                return  # стоп/рестарт раньше стыка
            leaving = tuple(s for s in self._live_slots if s not in program.slots.values())
            try:
                self._stop_slots(ns, program.slots.values())
                exec(program.code, ns)  # noqa: S102 — программа v2 = вывод render(), не текст LLM
            except Exception as exc:  # noqa: BLE001 — отказ громкий, уходящий трек играет дальше
                on_failed("exec_error", f"{type(exc).__name__}: {exc}")
                return
            self._live_slots, self._leaving = tuple(program.slots.values()), leaving
            info = self._arm_started(ns, program, on_started, origin=float(at_beat))
            leave = info["start_beat"] if leave_at is None else float(leave_at)

            def _rbx_deck_free() -> None:
                # Своей группы SC у деки нет: ``gate=0``/``/g_freeAll 1`` сняли бы и входящий трек.
                # Плееры уходящего снимаются с клока (новых нот нет), звучащие ноты доигрывают ``sus``.
                if generation != self._generation:
                    return  # стоп/рестарт уже снял обе деки
                self._stop_slots(ns, leaving)
                self._leaving = ()
                if on_left is not None:
                    on_left()

            clock.schedule(_rbx_deck_free, leave - HANDOFF_STOP_BEATS)

        clock.schedule(_rbx_cue, float(at_beat) - self._lead_beats)

    def at(self, beat: float, fn: Callable[[], None]) -> None:
        """Позвать ``fn`` из потока клока на доле ``beat``, если до неё не было start/stop."""
        generation = self._generation

        def _rbx_at() -> None:
            if generation == self._generation:
                fn()

        self._namespace()["Clock"].schedule(_rbx_at, float(beat))

    def stop(self) -> None:
        """Снять плееры деки и погасить группу 1 (``gate=0`` → ``/g_freeAll``)."""
        self._generation += 1
        self._stop_slots(self._namespace() or {}, (*self._live_slots, *self._leaving))
        self._live_slots, self._leaving = (), ()
        ramp_down_group(self._send_osc, 1)

    @staticmethod
    def _arm_started(ns: Mapping[str, Any], program: Any, on_started: Callable[[Dict[str, Any]], None],
                     origin: Optional[float]) -> Dict[str, Any]:
        """Доля старта — из самих плееров после exec; колбэк ``started`` — на эту долю.

        Фаза: у первого трека — остаток от формы; у стыка — сдвиг от ``origin`` (границы формы
        уходящего); 0 — встал ровно на неё.
        """
        clock = ns["Clock"]
        form = float(program.form_beats)
        starts = {slot: float(ns[slot].event_index) for slot in program.slots.values()}
        start_beat = min(starts.values())
        phase = start_beat % form if origin is None else start_beat - origin
        info = {"track_id": program.track_id, "deck": program.deck, "form_beats": form,
                "start_beat": start_beat, "players_aligned": len(set(starts.values())) == 1,
                "phase_in_form": round(phase, 3)}

        def _rbx_track_started() -> None:
            beat = float(clock.now())
            on_started(dict(info, clock_beat=round(beat, 3), late_beats=round(beat - start_beat, 3),
                            bpm=float(clock.get_bpm()), latency_s=float(clock.latency)))

        clock.schedule(_rbx_track_started, start_beat)
        return info

    @staticmethod
    def _stop_slots(ns: Mapping[str, Any], slots: Iterable[str]) -> None:
        for slot in dict.fromkeys(slots):
            stop = getattr(ns.get(slot), "stop", None)
            if callable(stop):
                stop()


__all__ = [
    "HANDOFF_STOP_BEATS", "RAMP_DOWN_RELEASE_SECONDS", "RenardoAdapter", "V2_CLOCK_LATENCY_S",
    "decode_osc_args", "load_sample_buffers", "osc_fail_detail", "ramp_down_group", "split_osc_address",
]
