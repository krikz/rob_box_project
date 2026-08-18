#!/usr/bin/env python3
"""
minimax_music.py — MCP tools for MiniMax music generation + library access.

Issue #1392 (closes #1358 / #1375 partial). Provides 7 MCP tools:

  generate_music        — call MiniMax Music Generation API, persist to library
  gen_list_library      — list tracks (sort by recent/popular/rating)
  gen_search_library    — substring search over title/prompt/lyrics
  gen_save_to_library   — save a previously-generated track with extra metadata
  gen_play_from_library — return mp3 path + duration for a saved track
                          (full playback wiring is a separate audio_node task)
  gen_delete_from_library — remove track from SQLite + filesystem
  gen_get_track_info    — fetch metadata for a single track

All tools degrade gracefully when ``MinimaxMusicClient`` or
``GeneratedMusicLibrary`` cannot be initialised — see
:mcp_server:_register_music_tools``.

This module deliberately keeps all ROS 2 imports lazy so the tools can be
unit-tested without a running daemon (mirrors design of
``tools/music.py:MusicManager`` etc.).
"""

from __future__ import annotations

import asyncio
import json
from typing import Any, Dict, List, Optional

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from ..core.generated_music_library import GeneratedMusicLibrary
from ..core.minimax_music_client import (
    MinimaxMusicAPIError,
    MinimaxMusicClient,
    MinimaxMusicConfigError,
    MinimaxMusicError,
)


# ── Shared parameter helpers ────────────────────────────────────────────────

def _prompt_param(required: bool = True) -> MCPToolParameter:
    return MCPToolParameter(
        name="prompt",
        type="string",
        description=(
            "Описание стиля/настроения трека (1-2000 chars): жанр, темп, "
            "инструменты, вокал, референсы ('Indie folk, melancholic, 92 bpm'). "
            "Для инструментальных треков передавай is_instrumental=true."
        ),
        required=required,
    )


def _lyrics_param() -> MCPToolParameter:
    return MCPToolParameter(
        name="lyrics",
        type="string",
        description=(
            "Текст песни с тегами [Verse]/[Chorus]/[Bridge] и т.п. "
            "Обязателен, если is_instrumental=false. До ~3000 символов."
        ),
        required=False,
        default="",
    )


def _instrumental_param() -> MCPToolParameter:
    return MCPToolParameter(
        name="is_instrumental",
        type="boolean",
        description=(
            "true — без вокала (только музыка). false (по умолчанию) — "
            "трек с пением по lyrics."
        ),
        required=False,
        default=False,
    )


def _track_id_param() -> MCPToolParameter:
    return MCPToolParameter(
        name="track_id",
        type="string",
        description=(
            "UUID трека в библиотеке (получи из gen_list_library / "
            "gen_search_library / generate_music). Пример: "
            "'b7216742f61f44078e4a17f7acbed388'."
        ),
        required=True,
    )


# ── Tool 1: generate_music ──────────────────────────────────────────────────

class GenerateMusicTool(MCPTool):
    """Сгенерировать музыкальный трек через MiniMax API и сохранить в библиотеку.

    Трек пишется в ``/data/music_library/<uuid>/track.mp3`` с метаданными в
    ``meta.json`` и индексируется в SQLite.

    ВАЖНО: генерация занимает **~40-160 секунд** (avg 88с in-container).
    DialogueNode должен сообщить юзеру «сейчас сгенерирую, подождите минуту…».
    """

    def __init__(
        self,
        node: Any,
        client: MinimaxMusicClient,
        library: GeneratedMusicLibrary,
    ) -> None:
        super().__init__(node)
        self._client = client
        self._library = library

    @property
    def name(self) -> str:
        return "generate_music"

    @property
    def description(self) -> str:
        return (
            "Сгенерировать новый музыкальный трек через MiniMax Music API и "
            "сохранить в библиотеку (/data/music_library). Возвращает track_id "
            "и путь к mp3. Генерация занимает 40-160 секунд — обязательно "
            "предупреди юзера о паузе! Используй когда юзер просит «спой/сыграй "
            "что-нибудь новое» или называет конкретную тему/стиль, которой нет "
            "в сохранённой библиотеке (проверь через gen_search_library)."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            _prompt_param(required=True),
            _lyrics_param(),
            _instrumental_param(),
            MCPToolParameter(
                name="mood",
                type="string",
                description=(
                    "Настроение (например 'romantic', 'dark', 'energetic', "
                    "'melancholic'). Сохраняется как тег в библиотеке."
                ),
                required=False,
                default="",
            ),
            MCPToolParameter(
                name="genre",
                type="string",
                description=(
                    "Жанр (например 'indie folk', 'synthwave', 'hip-hop'). "
                    "Сохраняется как тег в библиотеке."
                ),
                required=False,
                default="",
            ),
            MCPToolParameter(
                name="lang",
                type="string",
                description="Язык вокальной части ('ru', 'en', …). По умолчанию auto.",
                required=False,
                default="",
            ),
            MCPToolParameter(
                name="tags",
                type="string",
                description=(
                    "Доп. теги через запятую для поиска ('rainy,love,sad'). "
                    "Будут сохранены в библиотеке."
                ),
                required=False,
                default="",
            ),
            MCPToolParameter(
                name="model",
                type="string",
                description=(
                    "Override модели: 'music-3.0-free' (RPM 3, бесплатно) или "
                    "'music-3.0' (RPM 120, paid). По умолчанию — free."
                ),
                required=False,
                default="",
            ),
            MCPToolParameter(
                name="title",
                type="string",
                description="Читаемое название трека для meta.json.",
                required=False,
                default="",
            ),
            MCPToolParameter(
                name="auto_save",
                type="boolean",
                description=(
                    "true (по умолчанию) — автоматически сохранить в библиотеку. "
                    "false — только сгенерировать и вернуть audio_bytes, без записи."
                ),
                required=False,
                default=True,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.LONG

    @property
    def blocking(self) -> bool:
        # LLM должен дождаться результата чтобы рассказать юзеру track_id.
        return True

    @property
    def idempotent(self) -> bool:
        return False

    @property
    def interruptible(self) -> bool:
        return True

    async def execute(
        self,
        prompt: str,
        lyrics: str = "",
        is_instrumental: bool = False,
        mood: str = "",
        genre: str = "",
        lang: str = "",
        tags: str = "",
        model: str = "",
        title: str = "",
        auto_save: bool = True,
        **_: Any,
    ) -> MCPToolResult:
        """Асинхронно дёргает MiniMax API через ``asyncio.to_thread``."""
        valid, err = self.validate_parameters(prompt=prompt)
        if not valid:
            return MCPToolResult(success=False, error=err)

        if not is_instrumental and not lyrics.strip():
            return MCPToolResult(
                success=False,
                error="lyrics is required when is_instrumental=false",
            )

        tag_list = [t.strip() for t in (tags or "").split(",") if t.strip()]
        kwargs: Dict[str, Any] = dict(
            prompt=prompt,
            lyrics=lyrics,
            is_instrumental=bool(is_instrumental),
        )
        if model:
            kwargs["model"] = model

        self.log_info(
            f"🎼 generate_music: prompt={prompt[:60]!r}… "
            f"instrumental={is_instrumental} auto_save={auto_save}"
        )

        try:
            track = await asyncio.to_thread(self._client.generate, **kwargs)
        except ValueError as exc:
            # Argument validation inside the client (empty prompt, prompt>2000,
            # missing lyrics). Surface as a friendly tool error instead of
            # letting asyncio turn it into an unhandled exception.
            return MCPToolResult(success=False, error=f"invalid argument: {exc}")
        except MinimaxMusicConfigError as exc:
            return MCPToolResult(success=False, error=f"config: {exc}")
        except MinimaxMusicAPIError as exc:
            return MCPToolResult(
                success=False,
                error=f"MiniMax API: {exc}",
                data={"status_code": exc.status_code, "body": exc.body},
            )
        except MinimaxMusicError as exc:
            return MCPToolResult(success=False, error=f"MiniMax: {exc}")

        result: Dict[str, Any] = {
            "duration_ms": track.duration_ms,
            "sample_rate": track.sample_rate,
            "bitrate": track.bitrate,
            "model": track.model,
            "audio_size_bytes": len(track.audio_bytes),
        }

        if auto_save:
            saved = self._library.save_track(
                track.audio_bytes,
                prompt=prompt,
                lyrics=lyrics,
                title=title or "",
                model=track.model,
                duration_ms=track.duration_ms,
                sample_rate=track.sample_rate,
                bitrate=track.bitrate,
                tags=tag_list,
                mood=mood,
                genre=genre,
                lang=lang,
                is_instrumental=is_instrumental,
            )
            if not saved.get("success"):
                return MCPToolResult(
                    success=False,
                    error=f"Generated, but library save failed: {saved.get('error')}",
                    data=result,
                )
            result.update(
                {
                    "track_id": saved["id"],
                    "path": saved["path"],
                    "saved": True,
                }
            )
            return MCPToolResult(
                success=True,
                data=result,
                message=(
                    f"Мини-трек '{saved['id']}' сгенерирован ({len(track.audio_bytes)} байт, "
                    f"{track.duration_ms / 1000:.1f}с) и сохранён в {saved['path']}"
                ),
            )

        result["saved"] = False
        result["audio_bytes_b64_preview"] = track.audio_bytes[:32].hex()
        return MCPToolResult(
            success=True,
            data=result,
            message=(
                f"Мини-трек сгенерирован: {len(track.audio_bytes)} байт, "
                f"{track.duration_ms / 1000:.1f}с (auto_save=false, в библиотеку не записан)"
            ),
        )


# ── Tool 2: gen_list_library ────────────────────────────────────────────────

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


# ── Tool 3: gen_search_library ──────────────────────────────────────────────

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


# ── Tool 4: gen_save_to_library ─────────────────────────────────────────────

class GenSaveToLibraryTool(MCPTool):
    """Доукомплектовать метаданные для уже сгенерированного трека.

    NOTE: основной сценарий — ``generate_music(auto_save=true)`` уже пишет
    трек. Этот tool нужен чтобы **обновить теги/notes/rating** для трека
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


# ── Tool 5: gen_play_from_library ───────────────────────────────────────────

class GenPlayFromLibraryTool(MCPTool):
    """Вернуть путь к mp3 из библиотеки + инкрементировать play_count.

    Полноценное воспроизведение через ``/voice/audio/speech`` — отдельная
    задача (audio_node расширение). Этот tool даёт LLM минимально-достаточный
    артефакт: путь, длительность, метаданные.
    """

    def __init__(self, node: Any, library: GeneratedMusicLibrary) -> None:
        super().__init__(node)
        self._library = library

    @property
    def name(self) -> str:
        return "gen_play_from_library"

    @property
    def description(self) -> str:
        return (
            "Получить путь к mp3 из библиотеки сгенерированной музыки для "
            "последующего воспроизведения. Возвращает track_id, path, "
            "duration_ms, title. play_count увеличивается на 1. "
            "Используй когда юзер говорит «сыграй тот грустный трек», "
            "«давай послушаем ещё раз»."
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

        return MCPToolResult(
            success=True,
            data={
                "track_id": track_id,
                "path": info["path"],
                "title": info.get("title", ""),
                "duration_ms": info.get("duration_ms", 0),
                "exists_on_disk": True,
                "hint": (
                    "Playback через audio_node пока не автоматизирован — "
                    "передайте path в /voice/audio/speech (future work)."
                ),
            },
            message=(
                f"Трек '{track_id}' готов к воспроизведению: {info['path']} "
                f"({info.get('duration_ms', 0) / 1000:.1f}с)"
            ),
        )


# ── Tool 6: gen_delete_from_library ─────────────────────────────────────────

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


# ── Tool 7: gen_get_track_info ──────────────────────────────────────────────

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