#!/usr/bin/env python3
"""
gen_library.py — MCP-тулы библиотеки сгенерированной музыки (кэш, ADR-0158).

Шесть тулов поверх ``engine.gen.library.GeneratedMusicLibrary``:

  gen_list_library        — список (recent/popular/rating)
  gen_search_library      — подстрочный поиск по title/prompt/lyrics
  gen_save_to_library     — дописать метаданные уже сохранённому треку
  gen_play_from_library   — путь к mp3 + длительность (воспроизведение — sound_node;
                            переезд на деку PlayerOwner — ADR-0158 PR-2)
  gen_delete_from_library — удалить трек из SQLite и с диска
  gen_get_track_info      — метаданные одного трека

Тул ``generate_music`` (MiniMax Music API, 410 Gone с 20.08.2026) удалён вместе
с клиентом; новый узкий ``generate_music`` придёт с чойсером пути (ADR-0158 PR-3).
Модуль держит ROS-импорты ленивыми — тулы тестируются без демона.
"""

from __future__ import annotations

import json
from typing import Any, List

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType, shared_publisher
from ..engine.gen.library import GeneratedMusicLibrary


# ── Shared parameter helpers ────────────────────────────────────────────────

def _track_id_param() -> MCPToolParameter:
    return MCPToolParameter(
        name="track_id",
        type="string",
        description=(
            "UUID трека в библиотеке (получи из gen_list_library / "
            "gen_search_library). Пример: "
            "'b7216742f61f44078e4a17f7acbed388'."
        ),
        required=True,
    )


# ── Tool 1: gen_list_library ────────────────────────────────────────────────

class GenListLibraryTool(MCPTool):
    """Показать список треков в библиотеке сгенерированной музыки."""

    def __init__(self, node: Any, library: GeneratedMusicLibrary) -> None:
        super().__init__(node)
        self._library = library

    @property
    def name(self) -> str:
        return "gen_list_library"

    @property
    def description(self) -> str:
        return (
            "Показать список треков из библиотеки сгенерированной музыки "
            "(/data/music_library). Возвращает id, title, prompt, tags, "
            "rating, play_count, created_at. Используй когда юзер спрашивает "
            "«что у нас в библиотеке», «какие треки есть», «что ты раньше "
            "генерировал»."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="limit", type="integer",
                description="Максимум треков (default 20, max 100).",
                required=False, default=20,
            ),
            MCPToolParameter(
                name="sort_by", type="string",
                description="Сортировка: 'recent' (default), 'popular', 'rating'.",
                required=False, default="recent",
                enum=["recent", "popular", "rating"],
            ),
            MCPToolParameter(
                name="tag", type="string",
                description="Фильтр по тегу (точное совпадение).",
                required=False, default="",
            ),
            MCPToolParameter(
                name="mood", type="string",
                description="Фильтр по настроению (точное совпадение).",
                required=False, default="",
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

    @property
    def idempotent(self) -> bool:
        return True

    def execute(
        self,
        limit: int = 20,
        sort_by: str = "recent",
        tag: str = "",
        mood: str = "",
        **_: Any,
    ) -> MCPToolResult:
        try:
            limit_int = max(1, min(100, int(limit)))
        except (TypeError, ValueError):
            limit_int = 20

        result = self._library.list_tracks(
            limit=limit_int, sort_by=sort_by, tag=tag or None, mood=mood or None,
        )
        if not result.get("success"):
            return MCPToolResult(success=False, error=result.get("error"))

        return MCPToolResult(
            success=True,
            data={
                "tracks": result["tracks"],
                "total": result["total"],
                "sort_by": sort_by,
                "limit": limit_int,
            },
            message=f"В библиотеке {result['total']} трек(ов)",
        )


# ── Tool 2: gen_search_library ──────────────────────────────────────────────

class GenSearchLibraryTool(MCPTool):
    """Поиск по библиотеке сгенерированной музыки."""

    def __init__(self, node: Any, library: GeneratedMusicLibrary) -> None:
        super().__init__(node)
        self._library = library

    @property
    def name(self) -> str:
        return "gen_search_library"

    @property
    def description(self) -> str:
        return (
            "Искать треки в библиотеке сгенерированной музыки по ключевому "
            "слову. Поиск по title/prompt/lyrics/genre/mood/notes. Используй "
            "когда юзер говорит «найди трек про дождь», «сыграй тот грустный "
            "трек», «есть что-то романтичное?»."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="query", type="string",
                description="Поисковый запрос (1-200 chars).",
                required=True,
            ),
            MCPToolParameter(
                name="limit", type="integer",
                description="Максимум результатов (default 5, max 20).",
                required=False, default=5,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def read_only(self) -> bool:
        return True

    @property
    def idempotent(self) -> bool:
        return True

    def execute(self, query: str, limit: int = 5, **_: Any) -> MCPToolResult:
        valid, err = self.validate_parameters(query=query)
        if not valid:
            return MCPToolResult(success=False, error=err)

        try:
            limit_int = max(1, min(20, int(limit)))
        except (TypeError, ValueError):
            limit_int = 5

        result = self._library.search_tracks(query, limit=limit_int)
        if not result.get("success"):
            return MCPToolResult(success=False, error=result.get("error"))

        return MCPToolResult(
            success=True,
            data={
                "query": query,
                "tracks": result["tracks"],
                "total": result["total"],
            },
            message=(
                f"По запросу «{query}» найдено {result['total']} трек(ов)"
                if result["total"]
                else f"По запросу «{query}» ничего не найдено"
            ),
        )


# ── Tool 3: gen_save_to_library ─────────────────────────────────────────────

class GenSaveToLibraryTool(MCPTool):
    """Доукомплектовать метаданные для уже сгенерированного трека.

    NOTE: основной сценарий — генерация (``engine.gen.pipeline``) уже пишет
    трек в кэш. Этот tool нужен чтобы **обновить теги/notes/rating** для трека
    после генерации (например, юзер сказал «поставь ему рейтинг 5 и тег
    "любимый"»).
    """

    def __init__(self, node: Any, library: GeneratedMusicLibrary) -> None:
        super().__init__(node)
        self._library = library

    @property
    def name(self) -> str:
        return "gen_save_to_library"

    @property
    def description(self) -> str:
        return (
            "Обновить метаданные (tags, rating, notes, mood, genre) для "
            "уже сгенерированного трека. Используй когда юзер говорит "
            "«сохрани этот трек», «пометь его как любимый», «поставь 5 звёзд»."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            _track_id_param(),
            MCPToolParameter(
                name="title", type="string",
                description="Новое название (пустая строка = не менять).",
                required=False, default="",
            ),
            MCPToolParameter(
                name="tags", type="string",
                description="Новый список тегов через запятую (заменяет существующие).",
                required=False, default="",
            ),
            MCPToolParameter(
                name="mood", type="string",
                description="Настроение (заменяет существующее).",
                required=False, default="",
            ),
            MCPToolParameter(
                name="genre", type="string",
                description="Жанр (заменяет существующий).",
                required=False, default="",
            ),
            MCPToolParameter(
                name="rating", type="integer",
                description="Рейтинг 0-5 (-1 = не менять).",
                required=False, default=-1,
            ),
            MCPToolParameter(
                name="notes", type="string",
                description="Заметки (заменяют существующие).",
                required=False, default="",
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    def execute(
        self,
        track_id: str,
        title: str = "",
        tags: str = "",
        mood: str = "",
        genre: str = "",
        rating: int = -1,
        notes: str = "",
        **_: Any,
    ) -> MCPToolResult:
        valid, err = self.validate_parameters(track_id=track_id)
        if not valid:
            return MCPToolResult(success=False, error=err)

        info = self._library.get_track_info(track_id)
        if not info.get("success"):
            return MCPToolResult(success=False, error=info.get("error"))

        # Merge metadata
        merged = dict(info)
        if title:
            merged["title"] = title
        if tags:
            merged["tags"] = [t.strip() for t in tags.split(",") if t.strip()]
        if mood:
            merged["mood"] = mood
        if genre:
            merged["genre"] = genre
        if 0 <= int(rating) <= 5:
            merged["rating"] = int(rating)
        if notes:
            merged["notes"] = notes

        # Persist via delete + re-save? Simpler: we don't have an update API,
        # so we rewrite meta.json + UPDATE SQLite row directly via internal conn.
        from datetime import datetime, timezone
        now_iso = datetime.now(timezone.utc).isoformat()
        meta_json_path = self._library.root_dir / track_id / "meta.json"
        if meta_json_path.exists():
            try:
                disk_meta = json.loads(meta_json_path.read_text(encoding="utf-8"))
            except Exception:  # noqa: BLE001
                disk_meta = dict(merged)
            disk_meta.update(merged)
            disk_meta["updated_at"] = now_iso
            try:
                meta_json_path.write_text(
                    json.dumps(disk_meta, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
            except OSError as exc:
                return MCPToolResult(
                    success=False, error=f"Не удалось обновить meta.json: {exc}",
                )

        # Update SQLite row
        try:
            with self._library._lock:  # noqa: SLF001 — internal but stable
                self._library._conn.execute(  # noqa: SLF001
                    """
                    UPDATE generated_tracks SET
                        title = ?, tags = ?, mood = ?, genre = ?,
                        rating = ?, notes = ?, updated_at = ?
                    WHERE id = ?
                    """,
                    (
                        merged.get("title", ""),
                        json.dumps(merged.get("tags") or [], ensure_ascii=False),
                        merged.get("mood", ""),
                        merged.get("genre", ""),
                        int(merged.get("rating") or 0),
                        merged.get("notes", ""),
                        now_iso,
                        track_id,
                    ),
                )
                self._library._conn.commit()  # noqa: SLF001
        except Exception as exc:  # noqa: BLE001
            return MCPToolResult(
                success=False, error=f"SQLite update failed: {exc}",
            )

        return MCPToolResult(
            success=True,
            data=merged,
            message=f"Метаданные трека '{track_id}' обновлены",
        )


# ── Tool 4: gen_play_from_library ───────────────────────────────────────────

class GenPlayFromLibraryTool(MCPTool):
    """Воспроизвести mp3 из библиотеки (публикует путь в sound_node).

    Публикует абсолютный путь к ``track.mp3`` на ``/voice/sound/play_file`` —
    ``sound_node`` декодирует mp3 (pydub → 16 kHz stereo) и играет через
    ``AudioPlaybackManager`` (issue #1392 follow-up). Дополнительно
    инкрементирует ``play_count`` и возвращает метаданные трека.
    """

    def __init__(self, node: Any, library: GeneratedMusicLibrary) -> None:
        super().__init__(node)
        self._library = library
        self._play_file_pub = None
        self._music_state_pub = None
        try:
            from std_msgs.msg import String

            if hasattr(node, "create_publisher"):
                self._play_file_pub = node.create_publisher(
                    String, "/voice/sound/play_file", 10
                )
                # Issue #3108: общий с mcp_server/StopMusicTool publisher.
                self._music_state_pub = shared_publisher(
                    node, String, "/voice/generated_music/state", 10
                )
        except Exception:  # noqa: BLE001 — unit tests / minimal install
            self._play_file_pub = None
            self._music_state_pub = None

    @property
    def name(self) -> str:
        return "gen_play_from_library"

    @property
    def description(self) -> str:
        return (
            "Воспроизвести mp3-трек из библиотеки сгенерированной музыки "
            "(публикует путь в sound_node, который играет файл через "
            "динамик). Возвращает track_id, path, duration_ms, title. "
            "play_count увеличивается на 1. Используй когда юзер говорит "
            "«сыграй тот грустный трек», «давай послушаем ещё раз»."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            _track_id_param(),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False  # mutates DB but logically a "play" action

    @property
    def starts_music(self) -> bool:
        return True

    def execute(self, track_id: str, **_: Any) -> MCPToolResult:
        valid, err = self.validate_parameters(track_id=track_id)
        if not valid:
            return MCPToolResult(success=False, error=err)

        info = self._library.get_track_info(track_id)
        if not info.get("success"):
            return MCPToolResult(success=False, error=info.get("error"))

        if not info.get("exists_on_disk", False):
            return MCPToolResult(
                success=False,
                error=f"mp3 файл для '{track_id}' отсутствует на диске",
            )

        # Best-effort play_count++
        self._library.increment_play_count(track_id)

        # Реальное воспроизведение: публикуем путь в sound_node
        # (issue #1392 follow-up). Деградируем в path-only если publisher
        # недоступен (юнит-тесты / minimal install).
        playback = "publish_failed"
        if self._play_file_pub is not None:
            try:
                from std_msgs.msg import String

                self._play_file_pub.publish(String(data=info["path"]))
                playback = "published"
            except Exception as exc:  # noqa: BLE001
                self.log_error(f"gen_play_from_library publish failed: {exc}")

        # Сообщаем dialogue_node «сейчас играет трек» — LLM видит состояние
        # в system_context и сама решает, когда вызывать stop_music.
        if self._music_state_pub is not None:
            try:
                from std_msgs.msg import String

                self._music_state_pub.publish(
                    String(
                        data=json.dumps(
                            {
                                "status": "playing",
                                "track_id": track_id,
                                "title": info.get("title", ""),
                                "duration_ms": info.get("duration_ms", 0),
                            },
                            ensure_ascii=False,
                        )
                    )
                )
            except Exception as exc:  # noqa: BLE001
                self.log_error(f"gen_play_from_library state publish failed: {exc}")

        return MCPToolResult(
            success=True,
            data={
                "track_id": track_id,
                "path": info["path"],
                "title": info.get("title", ""),
                "duration_ms": info.get("duration_ms", 0),
                "exists_on_disk": True,
                "playback": playback,
            },
            message=(
                f"Трек '{track_id}' воспроизводится: {info['path']} "
                f"({info.get('duration_ms', 0) / 1000:.1f}с)"
            ),
        )


# ── Tool 5: gen_delete_from_library ─────────────────────────────────────────

class GenDeleteFromLibraryTool(MCPTool):
    """Удалить трек из библиотеки сгенерированной музыки (SQLite + mp3)."""

    def __init__(self, node: Any, library: GeneratedMusicLibrary) -> None:
        super().__init__(node)
        self._library = library

    @property
    def name(self) -> str:
        return "gen_delete_from_library"

    @property
    def description(self) -> str:
        return (
            "Удалить трек из библиотеки сгенерированной музыки: запись в "
            "SQLite + mp3/meta.json на диске. Необратимо. Используй когда "
            "юзер говорит «удали тот грустный трек», «удали последний трек»."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            _track_id_param(),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    def execute(self, track_id: str, **_: Any) -> MCPToolResult:
        valid, err = self.validate_parameters(track_id=track_id)
        if not valid:
            return MCPToolResult(success=False, error=err)

        result = self._library.delete_track(track_id)
        if not result.get("success"):
            return MCPToolResult(success=False, error=result.get("error"))

        return MCPToolResult(success=True, message=result.get("message"))


# ── Tool 6: gen_get_track_info ──────────────────────────────────────────────

class GenGetTrackInfoTool(MCPTool):
    """Получить метаданные одного трека из библиотеки."""

    def __init__(self, node: Any, library: GeneratedMusicLibrary) -> None:
        super().__init__(node)
        self._library = library

    @property
    def name(self) -> str:
        return "gen_get_track_info"

    @property
    def description(self) -> str:
        return (
            "Получить подробные метаданные одного трека: title, prompt, "
            "lyrics, tags, mood, genre, duration_ms, sample_rate, "
            "bitrate, play_count, rating, created_at, path. "
            "Используй когда юзер спрашивает «что за трек такой-то», "
            "«расскажи про этот трек»."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            _track_id_param(),
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

    @property
    def idempotent(self) -> bool:
        return True

    def execute(self, track_id: str, **_: Any) -> MCPToolResult:
        valid, err = self.validate_parameters(track_id=track_id)
        if not valid:
            return MCPToolResult(success=False, error=err)

        info = self._library.get_track_info(track_id)
        if not info.get("success"):
            return MCPToolResult(success=False, error=info.get("error"))

        return MCPToolResult(success=True, data=info)
