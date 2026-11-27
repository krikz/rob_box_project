"""Заказ песни голосом робота (ADR-0158 §2.3, В3/В4; PR-7): мост, фраза ожидания, фраза провала — всё кодом.

Вынесено из ``RequestMusicTool`` (бюджет класса, ADR-0145): тот решает путь, этот класс исполняет ветку ``generate``.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Callable, Dict, Optional, Tuple

from rob_box_mcp_tools.tool_outcome import PENDING_ORDERS, tool_outcome, tool_outcome_line  # noqa: F401
from rob_box_music import knowledge as kn
from rob_box_music.source import DECLINE, GENERATE, SONG_GEN_KINDS, Route, SourceAsk, decline_text, gen_kind

from .gen.song import (
    BUSY_TEXT,
    SongFailed,
    SongRequest,
    SongStudio,
    failed_words,
    is_given_lyrics,
    is_spoken_lyrics,
    verbatim_refusal,
    wait_text,
)

#: Код отказа пути (``engine_tools.SOURCE_DECLINED``; равенство держит тест).
DECLINED = "music_source"


def _declined(message: str) -> Tuple[str, Dict[str, Any]]:
    return "data", {"ok": False, "reason": DECLINED, "message": message}


def bridge_hold(hold: Any, bridge: Any) -> Any:
    """Мост ожидания этого заказа (``BridgeHold``) или ``None``: мост не встал или это трек кэша (свой запуск на деке
    через ``play_library_track``), а не мост ожидания."""
    if hold is None or not bridge.success or hold.bridge_id != (bridge.data or {}).get("track_id"):
        return None
    return hold


def _given_after_phrase(heard: str) -> str:
    """Слова, которые человек дал после фразы «этими словами»/«этим текстом», или «».

    Решает код по таблице ``VOCAL_PHRASES`` (ADR-0148), не правило в промпте. Фраза в конце реплики
    («спой этими словами») слов не даёт — их нет после неё. «Своими словами» в таблице нет."""
    from rob_box_music.universes import words_of

    words = words_of(heard)
    plain = [w for w, _s, _e in words]
    hit = next((entry for entry in kn.VOCAL_PHRASES if _phrase_span(plain, entry)), None)
    if hit is None:
        return ""
    end = _phrase_span(plain, hit)
    return heard[words[end - 1][2] :].strip(" \t,.:;—–-") if end < len(words) else ""


def _phrase_span(words: list, entry: str) -> int:
    """Позиция сразу за фразой таблицы в словах реплики, или 0 — фразы нет."""
    parts = entry.split()
    for i in range(len(words) - len(parts) + 1):
        if words[i : i + len(parts)] == parts:
            return i + len(parts)
    return 0


def _supplied_part(part: str) -> bool:
    """Аргумент сам несёт поданные слова: многострочные (:func:`is_given_lyrics`) или сказанные
    вслух одной строкой (:func:`is_spoken_lyrics`). Короткая тема — ни то, ни другое."""
    return is_given_lyrics(part) or is_spoken_lyrics(part)


def supplied_lyrics(args: Tuple[Any, ...]) -> str:
    """Слова, которые человек уже подал, или «» — для отказа инструментала (#5222), не для песни.

    Хвост после «этими словами»/«этим текстом» важнее явного ``text``. Затем — первый из
    ``text`` и ``heard_text``, который сам дотягивает до готовых слов или до сказанных вслух.
    В ``SongRequest.lyrics`` это не кладётся: однострочную речь ``check_song`` отверг бы
    («строк мало»), и песня не заказалась бы. Путь песни судит только :func:`song_request`."""
    text, heard = str(args[1] or ""), str(args[5] or "")
    return _given_after_phrase(heard) or next((part for part in (text, heard) if _supplied_part(part)), "")


def song_request(args: Tuple[Any, ...], genre: str) -> SongRequest:
    """Заказ песни из аргументов тула ``(intent, text, mood, genre, turn_id, heard_text)``.

    Готовые слова (``SongRequest.lyrics``) — только когда человек их реально дал (#4973) или когда
    аргумент сам дотягивает до песни (:func:`is_given_lyrics`: от ``MIN_LINES`` строк). Сказанная
    вслух речь — одна строка, порог строк недостижим: в текст песни она не попадает, его пишет LLM
    (#5222). Иначе ``SongStudio.make`` зовёт ``given_song``, ``check_song`` говорит «строк мало»,
    и человек слышит отказ вместо песни. Тема («про енота-механика») готовым текстом не становится
    по той же причине: студия проверяет её как слова и падает за 14 мс, не позвав LLM (#5092).
    Тема — первый непустой из ``text`` и ``heard_text``.

    Порядок готовых слов: хвост после «этими словами»/«этим текстом» важнее явного ``text`` (ход
    17:16: LLM кладёт просьбу в ``text``, слова оставляет в ``heard_text``). Затем — ``text``, если
    он сам длиннее порога (ход 17:23: жалоба в ``heard_text`` не вытесняет поданные слова).
    ``heard_text`` целиком — только когда он сам длиннее порога и ``text`` пуст. Дословные слова
    идут и темой, и текстом: тема нужна ``check_song`` (название), текст — чтобы LLM его не
    переписывала. Слова для отказа инструментала — :func:`supplied_lyrics`, не это поле."""
    text, heard = str(args[1] or ""), str(args[5] or "")
    given = _given_after_phrase(heard)
    lyrics = given or next((part for part in (text, heard) if is_given_lyrics(part)), "")
    theme = given or text or heard
    return SongRequest(theme, genre, str(args[2] or ""), lyrics)


def voice_refusal(provider: str, voice: str, lyrics: str) -> str:
    """Фраза отказа до заказа, или «»: у голоса нет пресета, либо он не поёт письменность
    поданных слов.

    Отказ здесь не запускает заказ и не тратит кредитов; голос не подменяется молча
    (#4973)."""
    from rob_box_dialog_host.tts_voice_registry import SingPresetUnknown, sing_preset_for

    try:
        preset = sing_preset_for(provider, voice)
    except SingPresetUnknown:
        return f"Спеть сейчас не могу: {failed_words('no_preset')}."
    return verbatim_refusal(provider, voice, preset.language, lyrics)


def held_play(hold: Any) -> Optional[Callable[[str], Any]]:
    """Песня готова: войдёт с деки моста кроссфейдом (``BridgeHold.deliver``); деку занял другой заказ — не заиграла.
    Моста ожидания нет — ``None`` (студия играет ``play_library_track``)."""
    from ..base import MCPToolResult

    if hold is None:
        return None

    def play(track_id: str) -> Any:
        ok = hold.deliver(track_id)
        return MCPToolResult(
            success=ok, data={"track_id": hold.bridge_id}, error="" if ok else "мост снят другим заказом"
        )

    return play


class SongOrder:
    """``route`` — ``SourceRouter.route`` (выбор заново без ``generate:*``, В3)."""

    def __init__(self, studio: SongStudio, route: Callable[..., Route], hold: Callable[[], Any] = lambda: None) -> None:
        self._studio = studio
        self._route = route
        self._hold = hold  # мост ожидания (``BridgeHold``, ADR-0158 PR-8): песня входит вместо него кроссфейдом

    def handles(self, route: Route) -> bool:
        """Путь выбрал генерацию песни (вид с пением ``SONG_GEN_KINDS``, не инструментал и не кавер) и студия
        подключена. Песню на названное произведение кандидаты не строят (``source.NAMED_SONG``, #4015)."""
        return route.kind == GENERATE and self._studio is not None and gen_kind(route.id) in SONG_GEN_KINDS

    def start(self, ask: SourceAsk, args: Tuple[Any, ...], order: Callable[..., Any], wrap: Callable[..., Any]) -> Any:
        """``MCPToolResult`` заказа (ожидание или отказ; ``wrap`` — ``engine_tools.tool_result``). Мост не встал не из-за
        «нечем играть» (сет играет, не нашлось): песню не начинаем, отдаём его результат как есть. ``args`` — ``(intent, text, mood, genre, turn_id, heard_text)`` заказа; мост — тот же ``order`` без просьбы
        петь (кэш по жанру, иначе синтез)."""
        kind, data = self._begin(ask, args, order)
        return data if kind == "raw" else wrap(data, "песня не заиграла")

    def _begin(self, ask: SourceAsk, args: Tuple[Any, ...], order: Callable[..., Any]) -> Tuple[str, Any]:
        studio = self._studio
        try:
            provider, voice = studio.voice()
        except SongFailed as exc:
            return _declined(f"Спеть сейчас не могу: {failed_words(exc.reason)}.")
        if studio.ordered_in(args[4]):  # тот же ход: песня уже пишется, ответ тот же, что у первого вызова
            return "data", {"ok": True, "pending": "song", "source_message": wait_text(provider, voice, False)}
        if studio.busy():
            return _declined(BUSY_TEXT)
        bridge = order(*args, ask=replace(ask, vocal=False, robot_voice=False, generated=False), gen_failed="pending")
        if not bridge.success and (bridge.data or {}).get("reason") != DECLINED:
            return "raw", bridge
        request = song_request(args, ask.genre)
        # до launch: заказ не уходит, голос не подменяется
        refusal = voice_refusal(provider, voice, request.lyrics)
        if refusal:
            return _declined(refusal)
        hold = bridge_hold(self._hold(), bridge)
        if not studio.launch(
            request, lambda why: self.failed_text(ask, why, bridge.success, hold), args[4], play=held_play(hold)
        ):
            return _declined(BUSY_TEXT)
        return "data", {
            **(bridge.data or {}),
            "ok": True,
            "pending": "song",
            "source_message": wait_text(provider, voice, bridge.success),
        }

    def failed_text(self, ask: SourceAsk, reason: str, bridge: bool, hold: Any = None) -> str:
        """Фраза провала песни. Играет мост — он доиграет финалом (``BridgeHold.fail``); моста нет — выбор заново без
        ``generate:*``, отказ с причиной."""
        if hold is not None:
            hold.fail()
        if bridge:
            return f"Песню сделать не вышло: {failed_words(reason)}. Играет пока своё."
        route = self._route(ask, gen_failed=reason)
        if route.kind == DECLINE:
            return decline_text(replace(route, failed=True))
        return f"Песню сделать не вышло: {failed_words(reason)}."
