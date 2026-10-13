#!/usr/bin/env python3
"""
music.py - музыкальные тулы вокруг движка v2 (ADR-0149)

Модуль предоставляет:
- StopMusicTool: выключить всю музыку (сет, деку движка v2, mp3 из библиотеки)
- GetMusicStateTool: что сейчас играет — по владельцу плеера v2 (#3613)
- SetMusicVolumeTool: громкость музыки (мастер-фейдер scsynth)
- TrackLibrary: персистентная медиатека треков (SQLite)
- LookupMelodyTool / SearchMelodyTool: поиск мелодий в RTTTL-архиве

Звук играет только движок v2 (``engine/``): старый путь «LLM пишет Renardo-код»
(``MusicManager``, ``execute_music_code``) удалён в #3614; Renardo и scsynth
поднимает ``engine.renardo_runtime.RenardoRuntime``.
"""

import json
import os
import re
import sqlite3
import threading
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from rob_box_music.works import key_slot

from ..engine.search import find, find_rare
from ..core.rtttl_library import RtttlLibrary, display_title, match_info


def _search_alternatives(
    library: Optional["RtttlLibrary"],
    query: Optional[str],
    chosen_title: Optional[str],
    limit: int = 4,
) -> List[Dict[str, Optional[str]]]:
    """До ``limit`` соседних результатов ``search(query)``, кроме выбранной записи."""
    if not query or library is None:
        return []
    alternatives: List[Dict[str, Optional[str]]] = []
    try:
        hits = library.search(query, limit=max(limit * 2, 6))
        for hit in hits:
            title = hit.get("title") or hit.get("name")
            if title == chosen_title:
                continue
            alternatives.append({"name": hit.get("name"), "title": title})
            if len(alternatives) >= limit:
                break
    except Exception:  # noqa: BLE001 — альтернативы необязательны, не мокнутый
        # search() в тестах (RtttlLibrary заменён на Mock() без .search)
        # не должен ронять весь вызов — альтернативы просто пустые.
        return []
    return alternatives


# ---------------------------------------------------------------------------
# issue #2964 — прозрачность вместо молчаливой подмены песни
# ---------------------------------------------------------------------------
# ``RtttlLibrary.get()`` (см. её докстринг) сознательно всегда возвращает
# лучшего ПО ТЕКСТУ кандидата — даже при слабом совпадении (issue #2896).
# Раньше единственной защитой было предупреждение в message: «сверь title
# с тем, что просил юзер». Живой прогон 24.09 показал, что LLM (minimax)
# это предупреждение игнорирует и играет что дали.
#
# Товарищ Шифу (правка к #2964): жёсткий порог/гейт по этому поводу НЕ
# нужен — история отката #2882→#2896 показала, что единая эвристика
# отказа под одно слабое совпадение («stranger things» → None) ломает
# сильные («super mario»). Вместо гейта — ``core.rtttl_library.match_info``/
# ``display_title`` (IDF-вес по корпусу, БЕЗ хардкод-списка стоп-слов) дают
# тулу и промпту скилла composer честную структурированную сверку; решение
# «это та же песня или нет» остаётся у модели по общему правилу в промпте.


def _resolve_melody_with_candidate(
    library: Optional["RtttlLibrary"],
    name: str,
    variants: Optional[List[str]],
) -> Tuple[Optional[str], Optional[Dict[str, Any]]]:
    """Пройти ``name`` → ``variants`` по порядку, вернуть первое найденное.

    Тот же порядок кандидатов, что и раньше (``name`` первым, затем
    ``variants``) — это НЕ вердикт «эта ли песня», просто первая запись,
    которую ``library.get()`` вообще нашла хоть как-то (issue #2896: он
    всегда возвращает лучшего по тексту кандидата). Возвращает
    ``(candidate, rec)`` — ``candidate`` нужен вызывающей стороне, чтобы
    строить ``alternatives``/``match_info`` против ТОГО ЖЕ запроса, что
    реально нашёл запись. ``(None, None)``, если ничего не нашлось вовсе.
    """
    if library is None:
        return None, None
    # #3399: слова запроса целиком нашлись в записи после разбора (падежи, транслит, служебные слова) —
    # «терминатора» → ``terminat``; запрос, который ``get`` и так покрывает, даёт ту же запись.
    hit = find(library, name)
    if hit.found and hit.confidence >= 1.0:
        return hit.query, hit.record
    rare = find_rare(library, name)  # #3599: «полёт валькирий» - редкое слово названия, остальное архиву неизвестно
    if rare.found:
        return rare.query, rare.record
    for candidate in [name] + [v for v in (variants or []) if v]:
        rec = library.get(candidate)
        if rec is not None:
            return candidate, rec
    return None, None


def _mismatch_note(match: Optional[Dict[str, Any]], requested: str) -> str:
    """Текст-предупреждение, если запрос покрыт записью не полностью.

    Не вердикт (см. модульный докстринг выше про issue #2964) — явно
    называет, какие значимые слова запроса не нашлись в найденной записи,
    чтобы модель сама решила, объявлять ли найденное под именем из
    запроса (общее правило — в промпте скилла composer, не здесь).
    """
    if not match or not match.get("unmatched"):
        return ""
    unmatched = ", ".join(match["unmatched"])
    return (
        f" ⚠️ Из запроса «{requested}» не нашлись слова: {unmatched} — "
        "возможно, это другая песня. Не объявляй найденное под именем из "
        "запроса, если по смыслу это не она."
    )


# ---------------------------------------------------------------------------
# Стоп, состояние и громкость — от владельца плеера v2 (#3614, #3613)
# ---------------------------------------------------------------------------


def stop_text(was_playing: bool) -> str:
    """Фраза о стопе по результату (ADR-0148): код знает, звучало ли что-то на деке."""
    return "Музыка остановлена." if was_playing else "Музыка и так не играла — на всякий случай всё остановил."


class StopMusicTool(MCPTool):
    """Выключить всю музыку: сет и деку движка v2 (через их владельцев) и mp3 из библиотеки в ``sound_node``."""

    @property
    def name(self) -> str:
        return "stop_music"

    @property
    def description(self) -> str:
        return (
            "Выключить всю музыку: диджей-сет, трек или пьесу движка и mp3 из библиотеки. "
            "Повторный вызов в тишине безопасен. Что сказать, вернёт тул."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self) -> MCPToolResult:
        """Сет закрывает ``DjSetTool``, деку — ``PlayerOwner`` (``MCPServer._stop_deck``), mp3 — ``sound_node``."""
        owner = getattr(self.node, "_player_owner", None)
        dj_set = getattr(self.node, "_dj_set_tool", None)
        was_playing = bool(owner is not None and owner.is_playing()) or bool(getattr(dj_set, "running", False))
        stop_deck = getattr(self.node, "_stop_deck", None)
        if callable(stop_deck):
            stop_deck("user_stop")
        if owner is not None and not was_playing:
            owner.stop("user_stop")  # явный стоп в «тишине»: снять деку и погасить группу всё равно
        stop_mp3 = getattr(self.node, "stop_generated_track_playback", None)
        if callable(stop_mp3):
            stop_mp3()
        self.log_info(f"[stop_music] was_playing={was_playing}")
        return MCPToolResult(success=True, data={"was_playing": was_playing}, message=stop_text(was_playing))


class GetMusicStateTool(MCPTool):
    """Что сейчас играет — по владельцу плеера v2 (сет, трек, пьеса), фразу строит код (#3613, ADR-0148)."""

    def __init__(self, node, runtime: Any = None) -> None:
        super().__init__(node)
        self._runtime = runtime

    @property
    def name(self) -> str:
        return "get_music_state"

    @property
    def description(self) -> str:
        return (
            "Что сейчас играет: диджей-сет (трек N из M, мелодия), трек или пьеса (название, источник, "
            "темп, инструмент, сколько играет и сколько осталось) или «ничего». Отвечай фразой из message."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def read_only(self) -> bool:
        return True

    @property
    def destructive(self) -> bool:
        return False

    def execute(self) -> MCPToolResult:
        """Сет идёт — его статус (``DjSetTool.status``); иначе дека (``PlayerOwner.now_playing``)."""
        from rob_box_voice.core.media_phrases import NOTHING_PLAYING_TEXT, now_playing_track_text

        owner = getattr(self.node, "_player_owner", None)
        dj_set = getattr(self.node, "_dj_set_tool", None)
        deck = owner.now_playing() if owner is not None else {"playing": False}
        status = dj_set.status() if dj_set is not None else None
        data: Dict[str, Any] = {"deck": deck}
        if isinstance(status, dict):
            data["dj_set"] = status
        if isinstance(status, dict) and status.get("active"):
            message = status["message"]
        elif deck.get("playing") or deck.get("pending"):
            message = now_playing_track_text(deck)
        else:
            message = NOTHING_PLAYING_TEXT
        runtime = self._runtime
        if runtime is None or owner is None or not runtime.available:
            reason = getattr(runtime, "last_error", None) if runtime is not None else "не создан"
            data["engine_error"] = reason
            message += f" Звуковой движок недоступен: {reason or 'Renardo не поднялся'}."
        data["playing"] = bool(deck.get("playing")) or bool(isinstance(status, dict) and status.get("active"))
        return MCPToolResult(success=True, data=data, message=message)


class SetMusicVolumeTool(MCPTool):
    """Issue #3125 — громкость МУЗЫКИ (мастер-фейдер scsynth), не голоса.

    Живой сет 28.09.2026: «играй громче» во время DJ-сета — у LLM был только
    ``set_volume``, а он крутит ``/tts_node volume_db``, то есть ГОЛОС. Уровень
    музыки задавал лишь ROS-параметр ``music_master_gain`` при старте, до LLM
    он не доходил, и модель честно выполнить просьбу не могла — фантазировала
    «подкручиваю трек на максимум» при неизменном уровне.

    Тул двигает тот же фейдер, что и параметр: ``RenardoRuntime.set_master_gain``
    → ``/n_set 999 gain <v>`` (синт ``masterlimiter``, сглаживание ``Lag.kr``,
    без щелчка). Шаг ``louder``/``quieter`` — ±3 dB (×√2 по амплитуде), как у
    ``set_volume`` для голоса. ``normal`` — значение ``music_master_gain``, с
    которым стартовал сервер. Уровень клэмпится в [0, 1]: выше 1.0 фейдер
    не поднимается (``set_master_gain``).

    Issue #3154: фейдер стоит ПОСЛЕ радио-динамики ``masterfilter`` (лимитер
    держит пик −1 dBFS до фейдера), поэтому шаг ±3 dB — ровно ±3 dB на
    выходе, а не упор в лимитер; на ``max`` (1.0) пик выхода −1 dBFS.
    """

    #: ±3 dB по амплитуде.
    STEP_FACTOR: float = 10 ** (3.0 / 20.0)
    #: Нижний предел ШАГОВОГО «тише»: шаги не должны молча заглушить музыку
    #: в ноль (для тишины есть ``stop_music``); явный ``level=0`` разрешён.
    MIN_STEP_GAIN: float = 0.05
    MAX_GAIN: float = 1.0

    def __init__(self, node, runtime: Any) -> None:
        super().__init__(node)
        self._runtime = runtime
        #: Уровень «как было при старте» — значение ROS-параметра
        #: ``music_master_gain``, с которым поднят звуковой сервер.
        self._normal_gain: float = float(runtime.master_gain)

    @property
    def name(self) -> str:
        return "set_music_volume"

    @property
    def description(self) -> str:
        return (
            "Громкость МУЗЫКИ (трек/DJ-сет: request_music, dj_set), "
            "а НЕ голоса робота. Юзер просит "
            "«громче/тише/погромче/потише» и сейчас играет музыка, или прямо "
            "говорит «музыку/трек/бит громче» — вызывай ЭТОТ тул, а не "
            "set_volume (set_volume меняет только голос). Музыку не "
            "перезапускает: играющий трек продолжает играть, меняется только "
            "уровень. action: louder/quieter — шаг ±3 dB, max — максимум, "
            "normal — стартовый уровень, set — абсолютный уровень level "
            "0..100 (% от максимума). mp3 из MiniMax-библиотеки "
            "(gen_play_from_library) этим тулом не регулируется."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="action",
                type="string",
                description=(
                    "louder — громче на шаг, quieter — тише на шаг, max — на "
                    "максимум, normal — стартовый уровень, set — выставить level"
                ),
                required=True,
                enum=["louder", "quieter", "max", "normal", "set"],
            ),
            MCPToolParameter(
                name="level",
                type="integer",
                description=(
                    "Только для action=set: уровень музыки в процентах от "
                    "максимума, 0..100 (значения вне диапазона обрезаются)."
                ),
                required=False,
            ),
        ]

    @property
    def slice(self) -> str:
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def _target_gain(self, action: str, level: Optional[float], current: float):
        """Целевой уровень для ``action`` или ``(None, error)``."""
        if action == "louder":
            return min(self.MAX_GAIN, current * self.STEP_FACTOR), None
        if action == "quieter":
            return max(self.MIN_STEP_GAIN, current / self.STEP_FACTOR), None
        if action == "max":
            return self.MAX_GAIN, None
        if action == "normal":
            return self._normal_gain, None
        if action == "set":
            if level is None:
                return None, "action=set требует level 0..100"
            try:
                pct = float(level)
            except (TypeError, ValueError):
                return None, f"level должен быть числом 0..100, получено {level!r}"
            return max(0.0, min(100.0, pct)) / 100.0 * self.MAX_GAIN, None
        return None, f"Неизвестное действие: {action}"

    def execute(self, action: str, level: Optional[float] = None) -> MCPToolResult:
        """Изменить уровень мастер-фейдера музыки."""
        current = float(self._runtime.master_gain)
        target, error = self._target_gain(action, level, current)
        if error is not None:
            return MCPToolResult(success=False, error=error)
        applied = self._runtime.set_master_gain(target)
        self.log_info(
            f"[set_music_volume] action={action} level={level} "
            f"master_gain {current:.2f} → {applied:.2f}"
        )
        pct = round(applied / self.MAX_GAIN * 100)
        if abs(applied - current) < 1e-3:
            edge = "максимальная" if applied >= self.MAX_GAIN else "уже такая"
            message = f"Громкость музыки не изменилась ({edge}, {pct}%)"
        else:
            message = f"Громкость музыки: {round(current / self.MAX_GAIN * 100)}% → {pct}%"
        return MCPToolResult(
            success=True,
            data={"old_gain": round(current, 3), "new_gain": round(applied, 3), "percent": pct},
            message=message,
        )


# ---------------------------------------------------------------------------
# TrackLibrary — персистентная медиатека треков (SQLite)
# ---------------------------------------------------------------------------

from pathlib import Path as _Path

# Migration file lookup: Docker mounts repo migrations/ at /migrations,
# dev/build environments find it relative to the package root.
_MIGRATION_FILE = _Path("/migrations/004_music_library.sql")
if not _MIGRATION_FILE.exists():
    _MIGRATION_FILE = _Path(__file__).resolve().parents[4] / "migrations" / "004_music_library.sql"


class TrackLibrary:
    """Персистентная SQLite медиатека треков.

    Использует ту же БД что и VoiceMemory/WaypointStore (VOICE_MEMORY_DB_PATH).
    Миграция ``004_music_library.sql`` применяется идемпотентно при инициализации
    (CREATE TABLE IF NOT EXISTS + INSERT OR IGNORE для bootstrap-трека).

    НЕ на ``/data/harness_voice.db`` (issue #2000 / ADR-0055): её таблицы
    (``music_tracks``, ``generated_tracks``) сами по себе не конфликтуют со
    схемой ``SQLiteVoiceMemory``, но ``WaypointStore``/``FAQStore`` — да
    (см. их докстринги), а все три стора делят один и тот же
    ``VOICE_MEMORY_DB_PATH``. Переносить музыку в одиночку значило бы
    расщепить единый файл ещё сильнее, а не объединить. См.
    ``docs/adr/0055-voice-memory-db-unify-with-harness.md`` (anti-goal §5.6).

    Thread-safe: все публичные методы используют ``self._lock``.

    Args:
        db_path: Путь к SQLite-файлу. По умолчанию — из VOICE_MEMORY_DB_PATH
                 или ``/data/voice_memory.db``.
    """

    def __init__(self, db_path: Optional[str] = None) -> None:
        self._db_path = db_path or os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        self._lock = threading.Lock()

        os.makedirs(os.path.dirname(self._db_path) or ".", exist_ok=True)
        self._conn = sqlite3.connect(self._db_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA foreign_keys=ON")
        self._conn.row_factory = sqlite3.Row
        self._apply_migration()

    # ------------------------------------------------------------------
    # Migration
    # ------------------------------------------------------------------

    def _apply_migration(self) -> None:
        """Применить 004_music_library.sql идемпотентно (IF NOT EXISTS + INSERT OR IGNORE)."""
        if not _MIGRATION_FILE.exists():
            raise FileNotFoundError(
                f"Миграция не найдена: {_MIGRATION_FILE}. "
                "Проверьте volume монтирование migrations/ в docker-compose."
            )
        ddl = _MIGRATION_FILE.read_text(encoding="utf-8")
        with self._lock:
            self._conn.executescript(ddl)
            self._conn.commit()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    #: Кириллица → латиница для ``_slug``.
    #:
    #: 🔴 FIX (live 30.08, vision-pi): старый ``_slug`` заменял КАЖДЫЙ
    #: не-ASCII символ на «_», поэтому любое русское имя превращалось в
    #: строку подчёркиваний той же длины. В живой медиатеке лежит
    #: ``('________________', 'комната_мудрости')``, а «тисбит» и «мурка»
    #: столкнулись бы в один slug, будь они одной длины. Юзер просил
    #: «сохрани как трек тисбит» — LLM, зная про это, каждый раз сама
    #: придумывала латинский slug и придумывала РАЗНЫЙ: в базе лежат
    #: ``tisbeat``, ``tisbit``, ``thisbit``, ``tinbit`` — четыре записи
    #: одного трека, и ни одну из них не находит «удали трек тисбит».
    _TRANSLIT: dict = {
        "а": "a", "б": "b", "в": "v", "г": "g", "д": "d", "е": "e",
        "ё": "e", "ж": "zh", "з": "z", "и": "i", "й": "y", "к": "k",
        "л": "l", "м": "m", "н": "n", "о": "o", "п": "p", "р": "r",
        "с": "s", "т": "t", "у": "u", "ф": "f", "х": "h", "ц": "ts",
        "ч": "ch", "ш": "sh", "щ": "sch", "ъ": "", "ы": "y", "ь": "",
        "э": "e", "ю": "yu", "я": "ya",
    }

    @classmethod
    def _slug(cls, name: str) -> str:
        """Имя трека → стабильный ASCII-slug.

        Кириллица транслитерируется, всё остальное не-ASCII схлопывается
        в «_», повторные «_» склеиваются. «тисбит» → ``tisbit`` при любом
        регистре, поэтому «сохрани как тисбит» и «удали трек тисбит»
        попадают в одну запись.
        """
        lowered = (name or "").lower().strip()
        translit = "".join(cls._TRANSLIT.get(ch, ch) for ch in lowered)
        slug = re.sub(r"[^a-z0-9_]", "_", translit)
        # Схлопываем подряд идущие «_», чтобы «комната мудрости» не
        # превращалась в частокол и чтобы slug оставался читаемым.
        slug = re.sub(r"_+", "_", slug).strip("_")
        return slug

    @staticmethod
    def _row_to_dict(row: sqlite3.Row, include_code: bool = True) -> Dict[str, Any]:
        d = dict(row)
        d["tags"] = json.loads(d.get("tags") or "[]")
        if not include_code:
            d.pop("code", None)
        return d

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def save_track(
        self,
        name: str,
        code: str,
        title: str = "",
        description: str = "",
        tags: Optional[List[str]] = None,
        rating: int = 0,
        notes: str = "",
    ) -> Dict[str, Any]:
        """Сохранить или обновить трек в медиатеке (INSERT OR REPLACE).

        Args:
            name: Уникальный идентификатор (slug, без пробелов).
            code: Renardo-код трека.
            title: Читаемое название.
            description: Описание трека.
            tags: Список тегов.
            rating: Оценка 0-5.
            notes: Личные заметки о треке.

        Returns:
            dict ``success``, ``message``, ``name``.
        """
        slug = self._slug(name)
        if not slug:
            return {"success": False, "error": "Некорректное имя трека"}

        ts = datetime.now(timezone.utc).isoformat()
        tags_json = json.dumps(tags or [], ensure_ascii=False)
        rating = max(0, min(5, rating))

        with self._lock:
            # Сохраняем created_at существующей записи если она есть
            row = self._conn.execute(
                "SELECT created_at FROM music_tracks WHERE name = ?", (slug,)
            ).fetchone()
            created_at = row["created_at"] if row else ts
            action = "обновлён" if row else "сохранён"

            self._conn.execute(
                """
                INSERT INTO music_tracks
                    (name, title, code, description, tags, rating, notes,
                     play_count, created_at, updated_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, COALESCE(
                    (SELECT play_count FROM music_tracks WHERE name = ?), 0
                ), ?, ?)
                ON CONFLICT(name) DO UPDATE SET
                    title       = excluded.title,
                    code        = excluded.code,
                    description = excluded.description,
                    tags        = excluded.tags,
                    rating      = excluded.rating,
                    notes       = excluded.notes,
                    updated_at  = excluded.updated_at
                """,
                (slug, title or name, code, description, tags_json,
                 rating, notes, slug, created_at, ts),
            )
            self._conn.commit()

        return {"success": True, "message": f"Трек '{slug}' {action}", "name": slug}

    def list_tracks(self, tag: Optional[str] = None, min_rating: int = 0) -> Dict[str, Any]:
        """Вернуть список треков с фильтрацией (без поля code).

        Args:
            tag: Фильтр по тегу (опционально).
            min_rating: Минимальный рейтинг (0-5).

        Returns:
            dict ``success``, ``tracks`` (list of dicts), ``total``.
        """
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT name, title, description, tags, rating, notes,
                       play_count, created_at, updated_at
                FROM music_tracks
                WHERE rating >= ?
                ORDER BY rating DESC, name ASC
                """,
                (min_rating,),
            ).fetchall()

        tracks = [self._row_to_dict(r, include_code=False) for r in rows]
        if tag:
            tracks = [t for t in tracks if tag in t.get("tags", [])]
        return {"success": True, "tracks": tracks, "total": len(tracks)}

    def load_track(self, name: str) -> Dict[str, Any]:
        """Получить код трека и инкрементировать play_count.

        Args:
            name: Имя трека.

        Returns:
            dict ``success``, ``code``, ``track`` (метаданные без code) или ``error``.
        """
        slug = self._slug(name)
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM music_tracks WHERE name = ?", (slug,)
            ).fetchone()

            if not row:
                names = [r[0] for r in self._conn.execute(
                    "SELECT name FROM music_tracks ORDER BY name"
                ).fetchall()]
                available = ", ".join(names) or "библиотека пуста"
                return {"success": False, "error": f"Трек '{slug}' не найден. Доступны: {available}"}

            self._conn.execute(
                "UPDATE music_tracks SET play_count = play_count + 1 WHERE name = ?",
                (slug,),
            )
            self._conn.commit()

        full = self._row_to_dict(row, include_code=True)
        code = full.pop("code")
        full["play_count"] += 1  # reflect incremented value
        return {"success": True, "code": code, "track": full}

    def find_melody(self, query: str) -> Optional[Dict[str, Any]]:
        """Найти известную мелодию (``type='melody'``) по имени/заголовку/тегу.

        Ищет регистро-независимо по ``name`` (slug), ``title`` и ``tags``
        (алиасы). Точное совпадение slug (через транслитерацию :meth:`_slug`)
        имеет приоритет над подстрочным совпадением.

        Args:
            query: Что юзер назвал («кузнечик», «имперский марш»).

        Returns:
            Запись мелодии с ``code``, либо ``None`` если не нашлось или
            колонка ``type`` ещё не добавлена (миграция 006 не применена).
        """
        q = (query or "").strip().lower()
        if not q:
            return None
        slug = self._slug(query)
        with self._lock:
            try:
                rows = self._conn.execute(
                    "SELECT * FROM music_tracks WHERE type = 'melody'"
                ).fetchall()
            except sqlite3.OperationalError:
                return None
        entries = [self._row_to_dict(r, include_code=True) for r in rows]
        # Точное совпадение slug — приоритет.
        for entry in entries:
            if slug and slug == entry.get("name"):
                return entry
        # Подстрочное совпадение по name/title/tags.
        for entry in entries:
            names = [
                entry.get("name") or "",
                entry.get("title") or "",
                *(entry.get("tags") or []),
            ]
            hay = " ".join(str(n).lower() for n in names)
            if q in hay:
                return entry
        return None

    def delete_track(self, name: str) -> Dict[str, Any]:
        """Удалить трек из медиатеки.

        Args:
            name: Имя трека.

        Returns:
            dict ``success``, ``message`` или ``error``.
        """
        slug = self._slug(name)
        with self._lock:
            cur = self._conn.execute(
                "DELETE FROM music_tracks WHERE name = ? RETURNING name", (slug,)
            )
            deleted = cur.fetchone()
            self._conn.commit()

        if not deleted:
            return {"success": False, "error": f"Трек '{slug}' не найден"}
        return {"success": True, "message": f"Трек '{slug}' удалён из медиатеки"}


class LookupMelodyTool(MCPTool):
    """Найти известную мелодию по имени и вернуть её ТОЧНЫЕ ноты.

    Ищет в RTTTL-библиотеке (архив ``data/rtttl_melodies.jsonl.gz``,
    10461 готовых мелодий) и возвращает СЫРУЮ RTTTL-строку в
    ``data['rtttl']`` — БЕЗ воспроизведения и БЕЗ конвертации. Ноты
    играет ``request_music`` (движок v2, ADR-0149 §5.1), а не модель.
    Фолбэк — курируемые мелодии ``music_tracks`` (012). Не допускает
    ошибку #1810 — сыграть гамму и назвать её «кузнечиком».
    """

    def __init__(
        self,
        node,
        library: TrackLibrary,
        rtttl_library: Optional[RtttlLibrary] = None,
    ) -> None:
        super().__init__(node)
        self._library = library
        self._rtttl_library = rtttl_library

    @property
    def name(self) -> str:
        return "lookup_melody"

    @property
    def description(self) -> str:
        return (
            "Найти известную мелодию по имени и вернуть её ТОЧНЫЕ ноты сырой "
            "RTTTL-строкой в data['rtttl'], НИЧЕГО не играя: проверить, есть ли "
            "такая мелодия и как она точно называется. Играет мелодию "
            "request_music(intent=melody, text=слова человека) — ноты и "
            "аранжировку решает код. Имя ищи на "
            "АНГЛИЙСКОМ или транслитом («имперский марш» → \"imperial march\"). "
            "Если не нашлось — честно скажи, что не знаешь точных нот."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="name",
                type="string",
                description="Название мелодии (английским или транслитом): "
                "«имперский марш» → \"imperial march\", «кузнечик» → "
                "\"grasshopper\", «happy birthday», «jingle bells»…",
                required=True,
            ),
            MCPToolParameter(
                name="variants",
                type="array",
                description=(
                    "Дополнительные варианты названия (английским/транслитом), "
                    "которые пробовать по порядку, если name не найдётся. "
                    "Например name=\"imperial march\", variants=[\"darth vader\", "
                    "\"star wars theme\"]."
                ),
                required=False,
                items=MCPToolParameter(
                    name="variant",
                    type="string",
                    description="Альтернативное написание/название мелодии.",
                ),
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def read_only(self) -> bool:
        return True

    @property
    def destructive(self) -> bool:
        return False

    def execute(
        self,
        name: str,
        variants: Optional[List[str]] = None,
    ) -> MCPToolResult:
        """Найти ноты и вернуть сырую RTTTL-строку (без воспроизведения)."""
        # 1. RTTTL-библиотека — приоритет.
        if self._rtttl_library is not None:
            # issue #2964 (правка товарища Шифу — НЕ жёсткий гейт, см.
            # модульный докстринг у _resolve_melody_with_candidate): как и
            # раньше (issue #2896), берётся лучший ПО ТЕКСТУ кандидат —
            # ``get()`` его всё равно всегда находит. Честность — в
            # прозрачности результата: реальное название (``display_title``
            # — с исполнителем, если title сам по себе неинформативен по
            # корпусу, напр. «Theme») и явная сверка значимых слов запроса
            # (``match`` — :func:`match_info`, IDF по корпусу, без
            # хардкод-списка стоп-слов). Решение «это та же песня» —
            # у модели, по общему правилу в промпте скилла composer.
            # ADR-0157 PR-3: тональность («Токкату ре минор») — слот, не слова поиска: «ре минор» находил
            # «Minority», а «токкату» оставалось вне поиска → роутер отдавал заказ LLM (6–13 с, #3491).
            name, key = key_slot(name)
            candidate, rec = _resolve_melody_with_candidate(
                self._rtttl_library, name, variants
            )
            if rec is not None:
                title = rec.get("title")
                shown_title = display_title(self._rtttl_library, rec)
                alternatives = _search_alternatives(self._rtttl_library, candidate, title)
                match = match_info(self._rtttl_library, rec, candidate or name)
                return MCPToolResult(
                    success=True,
                    data={
                        "name": rec.get("name"),
                        "title": title,
                        "display_title": shown_title,
                        "rtttl": rec.get("rtttl"),
                        "alternatives": alternatives,
                        "match": match,
                        "key": key,
                    },
                    message=(
                        f"Нашёл «{shown_title}». Точные ноты в data['rtttl']. "
                        + _mismatch_note(match, candidate or name)
                    ),
                )
        # 2. Фолбэк — курируемые мелодии в SQLite (type='melody', миграция 012).
        entry = self._library.find_melody(name)
        if entry is None:
            return MCPToolResult(
                success=False,
                error=(
                    f"Мелодия {name!r} не найдена в библиотеке. Скажи юзеру "
                    "честно, что не знаешь точных нот, и предложи сыграть "
                    "что-то в похожем духе — НЕ выдавай импровизацию за оригинал."
                ),
            )
        return MCPToolResult(
            success=True,
            data={
                "name": entry.get("name"),
                "title": entry.get("title"),
                "code": entry.get("code"),
            },
            message=f"Нашёл «{entry.get('title')}» (готовый Renardo-код в data['code']).",
        )


class SearchMelodyTool(MCPTool):
    """Поиск по RTTTL-библиотеке (10460 готовых мелодий) по имени/жанру/тегу.

    Возвращает кандидатов (метаданные, без нот). Ноты конкретной мелодии
    берутся через lookup_melody. Нужен, когда юзер хочет не одну мелодию, а
    выбор: «найди новогодние», «что есть из игр?».
    """

    def __init__(self, node, library: RtttlLibrary) -> None:
        super().__init__(node)
        self._library = library

    @property
    def name(self) -> str:
        return "search_melody"

    @property
    def description(self) -> str:
        return (
            "Найти мелодии в RTTTL-библиотеке по названию/жанру/тегу "
            "(английским или транслитом: «новогодние» → \"christmas\", "
            "«игры» → \"game\"). Возвращает до limit кандидатов с "
            "названием, артистом и тегами. Поиск идёт по названию, "
            "исполнителю, тегам и имени внутри формата мелодии. "
            "Чтобы СЫГРАТЬ конкретную — вызови lookup_melody(name=...)."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="query",
                type="string",
                description="Строка поиска: имя, артист, жанр или тег.",
                required=True,
            ),
            MCPToolParameter(
                name="limit",
                type="integer",
                description="Сколько кандидатов вернуть (по умолчанию 20).",
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, query: str, limit: int = 20) -> MCPToolResult:
        try:
            limit = max(1, min(50, int(limit or 20)))
        except (TypeError, ValueError):
            limit = 20
        hits = self._library.search(query, limit=limit)
        if not hits:
            return MCPToolResult(
                success=False,
                # 07.10: «танец утят» не нашёлся, хотя «Chicken Dance» в архиве есть, и робот сказал «в нотах
                # нет». Промах — по одной строке, а не отсутствие мелодии: код говорит, что именно проверено.
                error=(
                    f"В RTTTL-архиве (названия английские) по строке {query!r} записей нет — проверена только "
                    "эта строка. Если у мелодии есть оригинальное или английское название, повтори "
                    "search_melody с ним; не говори, что мелодии нет, пока не искал по нему."
                ),
            )
        return MCPToolResult(
            success=True,
            data={"melodies": hits, "total": len(hits)},
            message=f"Найдено {len(hits)} мелодий по запросу {query!r}.",
        )
