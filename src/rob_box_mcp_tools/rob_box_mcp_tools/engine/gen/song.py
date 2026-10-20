"""Песня голосом робота (ADR-0158 §2.3, §3.4, PR-7; решение Шифу В8: поём ТЕКУЩИМ голосом).

Конвейер, где решает код, а LLM пишет только слова (ADR-0148)::

    слоты (тема, жанр, настроение) ──► LLM: {title, lyrics} по JSON-схеме (правила — ``data/song.txt``)
      ──► валидатор кода (≤ 950 символов, строки, язык = ``SingPreset.language``; один повтор)
      ──► TTS ТЕКУЩЕГО провайдера/голоса + ``SingPreset`` ──► файл вокала (``rob_box_voice.sing_synth``)
      ──► KIE upload ──► ``add-instrumental`` (теги ``GEN_STYLES[жанр]`` + ``tags_hint``, пол и вес — из пресета)
      ──► кэш (оба варианта, с provider/voice) ──► дека ``PlayerOwner`` (``play_library_track``) ──► фраза кода

Ждать ~75 с вызывающий не может, поэтому :meth:`SongStudio.launch` отдаёт работу фоновому потоку; пока он пишет,
играет мост (решает ``RequestMusicTool``), а итоговую фразу — «песня готова», «не вышло: причина» — строит код из
результата (название и длительность — из кэша, голос — из провайдера; Silero — «запасным голосом»).

Silero не поём вовсе: модель живёт в ``tts_node`` (другой процесс), вторая копия здесь — неизмеренный риск OOM на
Pi. Текущий голос Silero — кандидата ``generate:*`` нет (``blocked``), чойсер берёт кэш/синтез/отказ.

Клон голоса Suno (validation-phrase/create-voice) не используется и не планируется (§2.3, проверка владельца голоса).
"""

from __future__ import annotations

import asyncio
import logging
import shutil
import tempfile
import threading
import time
from dataclasses import dataclass, replace
from importlib.resources import files
from typing import Any, Callable, Mapping, Optional, Tuple

from rob_box_music import knowledge as kn

from ...base import MCPToolResult
from .budget import Budget
from .pipeline import generate_into_cache
from .provider import GenJob, GenUnavailable

_LOG = logging.getLogger(__name__)

#: Предел текста песни (≈ 80 с вокала, ADR-0158 §2.3) и названия; меньше — песни нет.
SONG_MAX_CHARS = 950
TITLE_MAX = 40
MIN_LINES = 6
MIN_CHARS = 120
#: Доля букв нужного письма в тексте песни (куплет с парой чужих слов — ещё свой язык, смесь языков — нет).
SCRIPT_SHARE = 0.85
#: ``add-instrumental`` отвечает 422 «Please enter negativeTags» без этого поля (прогон 13.10): что Suno не должен петь/добавлять.
NEGATIVE_TAGS = "spoken word, rap, off-key vocals, laughter, crowd noise"
#: Жанр, если человек не назвал (``GEN_STYLES``).
DEFAULT_GENRE = "rock"
#: Ждать ответ LLM с текстом, секунд (HTTP-таймаут того же размера).
LYRICS_DEADLINE_S = 40.0
#: «Минуты полторы» из фразы ожидания — замер 13.10: добавить-инструментал ~75 с + текст и вокал.
WAIT_TEXT_S = 90
#: Ждать ``started`` песни на деке, секунд (граница формы ≤ длины песни; мост играет до неё).
STARTED_WAIT_S = 120.0
#: Провайдер ``provider_down`` — столько не заказываем снова (ADR-0158 §3.5).
DEAD_FOR_S = 600.0

LYRICS_SCHEMA: Mapping[str, Any] = {
    "type": "object",
    "properties": {"title": {"type": "string", "maxLength": TITLE_MAX},
                   "lyrics": {"type": "string", "maxLength": SONG_MAX_CHARS}},
    "required": ["title", "lyrics"],
    "additionalProperties": False,
}

#: Письмо языка голоса → диапазоны букв (``SingPreset.language``).
_SCRIPTS: Mapping[str, Tuple[Tuple[str, str], ...]] = {
    "ru": (("а", "я"), ("ё", "ё")), "en": (("a", "z"),), "de": (("a", "z"), ("ä", "ü")), "zh": (("一", "鿿"),),
}
LANGUAGE_NAMES = {"ru": "русский", "en": "английский", "de": "немецкий", "zh": "китайский"}

#: Занято: песня уже пишется (одна за раз — стенд и деньги общие).
BUSY_TEXT = "Песню я уже пишу, подожди немного."

#: ``(system, user, schema) -> {"title", "lyrics"}`` — LLM-ступень выбора ``llm_harness`` (DeepSeek, json_object).
LyricsAsk = Callable[[str, str, Mapping[str, Any]], Mapping[str, Any]]


class SongFailed(Exception):
    """Песня не получилась по нашей причине (слова — ``source.REASON_WORDS``); причины провайдера — ``GenUnavailable``."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason
        self.detail = detail


@dataclass(frozen=True)
class SongRequest:
    """Слоты песни: тема словами человека, жанр (ключ ``GEN_STYLES`` или «»), настроение (``MOOD_ENERGY`` или «»)."""

    theme: str
    genre: str = ""
    mood: str = ""


@dataclass(frozen=True)
class Song:
    title: str
    lyrics: str


@dataclass(frozen=True)
class SongResult:
    """Что получилось — из ответов провайдеров, а не из головы: по этому строится фраза успеха."""

    track_id: str
    title: str
    duration_s: float
    provider: str
    voice: str
    task_id: str
    credits: float


def rules_text() -> str:
    return files("rob_box_mcp_tools.data").joinpath("song.txt").read_text(encoding="utf-8")


def _script_share(text: str, language: str) -> float:
    ranges = _SCRIPTS.get(language, _SCRIPTS["ru"])
    letters = [c for c in text.lower() if c.isalpha()]
    ours = [c for c in letters if any(lo <= c <= hi for lo, hi in ranges)]
    return len(ours) / len(letters) if letters else 0.0


def check_song(title: str, lyrics: str, language: str) -> str:
    """Что не так с текстом, или «» (ADR-0158 §3.4). Проверяем то, что код проверить может; «свой, не перевод»
    проверить нечем (§9) — это правило скилла."""
    lines = [ln for ln in lyrics.splitlines() if ln.strip()]
    problems = ((not title.strip() or len(title) > TITLE_MAX, f"название от 1 до {TITLE_MAX} символов"),
                (len(lyrics) > SONG_MAX_CHARS, f"текст длиннее {SONG_MAX_CHARS} символов"),
                (len(lyrics) < MIN_CHARS or len(lines) < MIN_LINES, f"текст короче: нужно от {MIN_LINES} строк"),
                (_script_share(lyrics, language) < SCRIPT_SHARE, f"текст должен быть на языке «{language}»"))
    return next((why for bad, why in problems if bad), "")


def lyrics_prompt(request: SongRequest, language: str, problem: str = "") -> str:
    genre = kn.GEN_STYLES[request.genre].name if request.genre in kn.GEN_STYLES else "на твой выбор"
    lines = [f"Просьба человека: {request.theme}", f"Жанр: {genre}", f"Настроение: {request.mood or 'на твой выбор'}",
             f"Язык: {LANGUAGE_NAMES.get(language, language)}"]
    if problem:
        lines.append(f"Прошлый ответ не принят: {problem}. Исправь.")
    return "\n".join(lines)


def write_lyrics(ask: LyricsAsk, request: SongRequest, language: str, rules: str) -> Song:
    """Текст песни от LLM + проверка кода. Невалидное — один повтор с названной причиной; второй отказ или ошибка
    LLM — ``SongFailed("lyrics")`` (ADR-0148: без синтетических ретраев сверх одного)."""
    system, problem = rules.replace("{language}", LANGUAGE_NAMES.get(language, language)), ""
    for _attempt in (1, 2):
        try:
            answer = ask(system, lyrics_prompt(request, language, problem), LYRICS_SCHEMA)
        except Exception as exc:  # noqa: BLE001 — LLM не роняет заказ
            raise SongFailed("lyrics", f"LLM: {type(exc).__name__}: {exc}") from exc
        title, lyrics = str(answer.get("title") or "").strip(), str(answer.get("lyrics") or "").strip()
        problem = check_song(title, lyrics, language)
        if not problem:
            return Song(title, lyrics)
    raise SongFailed("lyrics", problem)


def harness_lyrics_ask(env: Optional[Mapping[str, str]] = None, log: Callable[[], Any] = lambda: _LOG
                       ) -> Optional[LyricsAsk]:
    """``LyricsAsk`` над провайдером ``llm_harness`` (тот же, что у разбора фраз и выбора); нет ключа — ``None``."""
    from ..chooser_model import build_provider, run_sync

    provider = build_provider("llm_harness", env, timeout_s=LYRICS_DEADLINE_S)
    if provider is None:
        log().warning("🎤 [song] LLM для текстов выключена: нет ключа/конфига llm_harness — петь не умею")
        return None

    def ask(system: str, user: str, schema: Mapping[str, Any]) -> Mapping[str, Any]:
        return run_sync(asyncio.wait_for(provider.complete_json(system, user, schema), LYRICS_DEADLINE_S))

    return ask


def song_tags(request: SongRequest, tags_hint: str) -> Tuple[str, str]:
    """``(ключ жанра, EN-теги Suno)``: теги — из таблицы ``GEN_STYLES`` + подсказка пресета, не от LLM."""
    genre = request.genre if request.genre in kn.GEN_STYLES else DEFAULT_GENRE
    return genre, ", ".join(t for t in (kn.GEN_STYLES[genre].tags, tags_hint) if t)


def voice_label(provider: str, voice: str) -> str:
    """Имя голоса для фразы: ``display_name`` каталога, иначе id."""
    from rob_box_voice.tts_voice_registry import voice_info_for

    info = voice_info_for(provider, voice)
    return str(info["display_name"]) if info else voice


def sung_by(provider: str, voice: str) -> str:
    """«голосом X» или «запасным голосом X» (Silero — запасной, офлайн-последний в цепочке TTS; ADR-0158 §2.3)."""
    return f"{'запасным ' if provider == 'silero' else ''}голосом {voice_label(provider, voice)}"


def wait_text(provider: str, voice: str, bridge: bool) -> str:
    """Фраза на время ожидания (В4): факты — от кода."""
    tail = " Пока играет своё." if bridge else ""
    return f"Пишу песню, пою {sung_by(provider, voice)} — минуты полторы.{tail}"


def done_text(result: SongResult) -> str:
    from rob_box_voice.core.media_phrases import duration_text

    return (f"Песня «{result.title}» готова, {duration_text(result.duration_s)}. "
            f"Пою {sung_by(result.provider, result.voice)}.")


def failed_words(reason: str) -> str:
    """Причина провала словами — одна таблица ``source.REASON_WORDS`` (и причины провайдера, и наши)."""
    from rob_box_music.source import REASON_WORDS

    return REASON_WORDS.get(reason, reason)


def current_voice(provider: Optional[str], store: Any = None) -> Tuple[str, str]:
    """``(провайдер, голос)`` сейчас: провайдер — из ``/voice/tts/provider_state`` (``node.actual_tts_provider``), голос —
    ``VoiceStateStore.resolve`` (установленный ``set_voice`` или дефолт провайдера). Нет провайдера — ``no_voice``."""
    if not provider:
        raise SongFailed("no_voice", "провайдер TTS ещё не известен")
    if store is not None:
        return provider, store.resolve(provider)[0]
    from rob_box_voice.tts_voice_registry import default_voice_for

    return provider, default_voice_for(provider)


#: Язык песен робота, когда голос не привязан к языку (MiniMax: язык задаёт ``language_boost`` при любом голосе —
#: прогон 13.10: дефолтный ``male-qn-qingse`` числится китайским, и песня вышла на китайском).
ROBOT_LANGUAGE = "ru"
UNBOUND_LANGUAGE_PROVIDERS = ("minimax",)


def song_language(provider: str, preset_language: str) -> str:
    """Язык текста песни: у голосов, привязанных к языку (Yandex, Silero), — язык пресета; у MiniMax — язык робота."""
    return ROBOT_LANGUAGE if provider in UNBOUND_LANGUAGE_PROVIDERS else preset_language


class _Language:
    """Прокси бэкенда пения: подаёт TTS язык песни вместо языка из каталога голоса (``song_language``)."""

    def __init__(self, inner: Any, language: str) -> None:
        self._inner, self._language = inner, language
        self.provider, self.fmt = inner.provider, inner.fmt

    def synth(self, text: str, preset: Any) -> Any:
        return self._inner.synth(text, replace(preset, language=self._language))


def _sing(provider: str, voice: str, lyrics: str, path: str) -> Any:
    """Вокал текущим голосом в файл; бэкенды пения создаются на заказ (Silero не остаётся в памяти)."""
    from rob_box_voice.sing_synth import SingSynthError, default_backends, synth_vocal_to_file

    backends = {name: _Language(b, song_language(name, "")) if name in UNBOUND_LANGUAGE_PROVIDERS else b
                for name, b in default_backends().items()}
    try:
        return synth_vocal_to_file(provider, voice, lyrics, path, backends=backends)
    except SingSynthError as exc:
        raise SongFailed(vocal_reason(exc.reason), f"{provider}:{voice}: {exc}") from exc


def vocal_reason(synth_reason: str) -> str:
    """Причина провала по коду ``SingSynthError``: ``vocal_<код>``, если у кода есть слова, иначе общее ``vocal``."""
    from rob_box_music.source import REASON_WORDS

    reason = f"vocal_{synth_reason}"
    return reason if reason in REASON_WORDS else "vocal"


class SongStudio:
    """Песня заказом: ``make`` — блокирующий конвейер, ``launch`` — то же в фоне с озвучкой итога.

    ``library`` — ленивый доступ к ``GeneratedMusicLibrary``; ``voice`` — ``() -> (провайдер, голос)``; ``play`` —
    ``track_id -> MCPToolResult`` (``play_library_track``); ``speak`` — реплика роботом (тот же ``speak_text``);
    ``stop`` — снять мост перед песней; ``sing`` — вокал в файл (подменяется в тестах)."""

    def __init__(self, provider: Any, library: Callable[[], Any], lyrics: LyricsAsk, voice: Callable[[], Tuple[str, str]],
                 play: Callable[[str], Any], speak: Callable[[str], None], *,
                 started: Callable[[str], Any] = lambda track_id: None,
                 sing: Callable[[str, str, str, str], Any] = _sing, rules: Callable[[], str] = rules_text,
                 clock: Callable[[], float] = time.monotonic, log: Callable[[], Any] = lambda: _LOG) -> None:
        self._provider, self._library, self._lyrics, self._voice = provider, library, lyrics, voice
        self._play, self._speak, self._sing, self._rules = play, speak, sing, rules
        self._started = started  # ждать ``started`` дольше: трек деки встаёт на ближайшую границу формы (до её длины)
        self._clock, self._log = clock, log
        self._lock = threading.Lock()
        self._busy = False
        self._turn: Optional[str] = None  # ход, заказавший идущую песню (LLM зовёт request_music и generate_music подряд)
        self._dead_until = 0.0

    # ── что видит чойсер пути ──
    def has_key(self) -> bool:
        return bool(self._provider.configured)

    def blocked(self) -> str:
        """Причина, по которой песню заказывать нельзя (жёсткий фильтр кода), или «». Silero (запасной голос) не поём:
        его модель живёт в ``tts_node``, а вторая копия в этом процессе — неизмеренный риск OOM на Pi."""
        try:
            return "silero_voice" if self._voice()[0] == "silero" else ""
        except SongFailed:
            return ""

    def provider_ok(self) -> bool:
        return self._clock() >= self._dead_until

    def voice(self) -> Tuple[str, str]:
        """``(провайдер, голос)`` сейчас; ``SongFailed("no_voice")``, если провайдер ещё не известен."""
        return self._voice()

    def voice_id(self) -> str:
        """Голос сейчас (для фильтра кэша «спето этим голосом»); не знаем — «»."""
        try:
            return self._voice()[1]
        except SongFailed:
            return ""

    def busy(self) -> bool:
        return self._busy

    def ordered_in(self, turn: Optional[str]) -> bool:
        """Идущую песню заказал этот же ход: второй вызов тула того же хода — не новый заказ."""
        return bool(self._busy and turn and turn == self._turn)

    # ── конвейер ──
    def make(self, request: SongRequest) -> SongResult:
        library = self._library()
        if library is None:
            raise SongFailed("no_track", "кэш генераций недоступен")
        provider, voice = self._voice()
        if provider == "silero":  # страховка: чойсер сюда не пускает (blocked)
            raise SongFailed("silero_voice", "Silero в этом процессе не грузим")
        from rob_box_voice.tts_voice_registry import SingPresetUnknown, sing_preset_for

        try:
            preset = sing_preset_for(provider, voice)
        except SingPresetUnknown as exc:
            raise SongFailed("no_preset", str(exc)) from exc
        started = self._clock()
        song = write_lyrics(self._lyrics, request, song_language(provider, preset.language), self._rules())
        workdir = tempfile.mkdtemp(prefix="rbx-song-")
        try:
            vocal = self._sing(provider, voice, song.lyrics, f"{workdir}/vocal")
            genre, tags = song_tags(request, preset.tags_hint)
            job = GenJob(kind="robot_song", style_key=genre, title=song.title, style_tags=tags,
                         negative_tags=NEGATIVE_TAGS, lyrics=song.lyrics, audio_path=vocal.path,
                         audio_weight=preset.audio_weight, vocal_gender=preset.vocal_gender)
            made = generate_into_cache(self._provider, library, Budget(library), job, theme=request.theme,
                                       tts_provider=provider, voice=voice, early=True, log=self._log)
        finally:
            shutil.rmtree(workdir, ignore_errors=True)
        saved = next((s for s in made.saved if s.get("success")), None)
        if saved is None:
            raise SongFailed("no_track", f"задача {made.task_id} без сохранённых треков")
        self._log().info(f"🎤 [song] «{song.title}» {provider}:{voice} жанр={genre} task={made.task_id} "
                         f"credits={made.credits:g} track={saved['id']} за {self._clock() - started:.0f} с")
        return SongResult(str(saved["id"]), song.title, float(saved.get("duration_ms") or 0) / 1000.0, provider, voice,
                          made.task_id, made.credits)

    # ── фон ──
    def launch(self, request: SongRequest, on_failed: Callable[[str], str], turn: Optional[str] = None,
               play: Optional[Callable[[str], Any]] = None) -> bool:
        """Запустить ``make`` в фоне; ``False`` — уже пишется другая песня. Итог говорит голос: ``done_text`` или
        ``on_failed(причина)``; песня играет с деки сразу после готовности — ``play`` (мост ожидания
        ``BridgeHold``: входит кроссфейдом, ADR-0158 PR-8), иначе ``play_library_track``."""
        with self._lock:
            if self._busy:
                return False
            self._busy, self._turn = True, turn
        threading.Thread(target=self._run, args=(request, on_failed, play or self._play), name="rbx-song",
                         daemon=True).start()
        return True

    def _run(self, request: SongRequest, on_failed: Callable[[str], str], play: Callable[[str], Any]) -> None:
        try:
            result = self.make(request)
            played = play(result.track_id)
            if self._late_start(played):
                self._log().info(f"🎤 [song] «{result.title}» ждёт границы формы на деке — мост играет до неё")
                played = MCPToolResult(success=True)
            if getattr(played, "success", False):
                self._say(done_text(result))
            else:
                self._log().warning(f"🎤 [song] не заиграла: {getattr(played, 'error', '')}")
                self._say(on_failed("play_failed"))
        except (SongFailed, GenUnavailable) as exc:
            reason = exc.reason
            if reason == "provider_down":
                self._dead_until = self._clock() + DEAD_FOR_S
            self._log().warning(f"🎤 [song] не вышло: {exc}")
            self._say(on_failed(reason))
        except Exception as exc:  # noqa: BLE001 — поток не должен молчать
            self._log().error(f"🎤 [song] сбой: {type(exc).__name__}: {exc}")
            self._say(on_failed("internal"))
        finally:
            self._busy = False

    def _late_start(self, played: Any) -> bool:
        """``play_library_track`` ждёт ``started`` 6 с, а трек деки встаёт на границу формы (до ~длины песни):
        ``not_started`` — не отказ; ждём событие этого запуска дольше, пока играет мост."""
        data = getattr(played, "data", None) or {}
        if getattr(played, "success", False) or data.get("reason") != "not_started" or not data.get("track_id"):
            return False
        event = self._started(str(data["track_id"]))
        return bool(event is not None and getattr(event, "event", "") == "started")

    def _say(self, text: str) -> None:
        try:
            self._speak(text)
        except Exception as exc:  # noqa: BLE001 — реплика не должна ронять поток
            self._log().warning(f"🎤 [song] реплика не сказана: {type(exc).__name__}: {exc}")


__all__ = ["BUSY_TEXT", "DEFAULT_GENRE", "STARTED_WAIT_S", "LYRICS_SCHEMA", "LyricsAsk", "Song", "SongFailed", "SongRequest",
           "SongResult", "SongStudio", "check_song", "song_language", "current_voice", "done_text", "failed_words",
           "harness_lyrics_ask", "lyrics_prompt", "rules_text", "song_tags", "sung_by", "wait_text", "write_lyrics"]
