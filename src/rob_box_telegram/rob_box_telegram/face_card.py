#!/usr/bin/env python3
"""face_card.py — read-only helpers for Telegram ``/faces`` and ``/face``
commands (issue #3025).

The Telegram bot mounts ``./data/faces:/data/faces:ro`` (same host
directory that ``vision-face`` writes; see docker/vision/docker-compose.yaml).
This module reads that directory directly — never goes through ``FaceStore``
or any ROS API — because the bot process does not share state with the
vision-face node, and the volume is read-only on purpose (ADR-0123 §4:
FaceStore writes only inside its own container; the bot must not
silently mutate the live store even by accident).

Design constraints from issue #3025:

* No new ROS services / topics. The two handlers consume only the file
  layout documented in ``rob_box_perception.face_store`` (``meta.json``,
  ``reference.jpg``, ``embeddings.npy``, ``encounters/*.jpg``).
* Both handlers are read-only. They must never write to ``/data/faces``.
* ``last_seen`` / dialogue counters belong to the «Знакомый» seam
  (ADR-0106), NOT FaceStore — the summary text uses ``last_encounter_ts``
  from ``meta.json`` with the explicit note "последняя встреча по лицу".
* Privacy (ADR-0123): the only consumer is the operator Telegram chat,
  already gated by ``@authorized`` in ``commands.py``. This module does
  not filter by person — that's the handler's job (``/face <id>`` only
  resolves records that the operator typed).

Tested without ROS / without PIL via injected readers (see
``test_face_card.py``); the PIL collage builder is exercised there too
with tiny in-memory JPEGs.
"""

from __future__ import annotations

import io
import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional, Tuple

if TYPE_CHECKING:
    import numpy as np  # only for type hints in annotations

logger = logging.getLogger(__name__)


# Names from ``FaceStore`` (face_store.py:_META_FILENAME et al.). They
# are private there, so we duplicate the constants here — same rationale
# as ``scripts/maintenance/face_store_admin.py``: these are the public
# on-disk format names, not internal logic.
META_FILENAME = "meta.json"
EMBEDDINGS_FILENAME = "embeddings.npy"
REFERENCE_FILENAME = "reference.jpg"
ENCOUNTERS_DIRNAME = "encounters"


@dataclass(frozen=True)
class FaceSummary:
    """Compact row for the ``/faces`` table.

    Mirrors the columns the task spec calls out: имя (или «незнакомец»),
    person_id (короткий), speaker_id, число встреч, размер галереи
    (count of embeddings, NOT byte size), gallery_cohesion.

    ``gallery_cohesion`` is the median pairwise cosine from ``FaceStore.
    people()`` when available; ``None`` for single-vector galleries (a
    pairwise cosine is not defined for one vector — see
    ``face_store_admin._gallery_cohesion``). On disk ``FaceStore`` does
    not persist this; it's recomputed by ``_cohesion_from_embeddings``
    when ``embeddings.npy`` is present.
    """

    person_id: str
    person_id_short: str
    name: Optional[str]
    speaker_id: Optional[str]
    encounter_count: int
    embeddings_count: int
    gallery_cohesion_median: Optional[float]
    gallery_bytes: int
    is_stranger: bool
    has_reference: bool


@dataclass(frozen=True)
class FaceDetail:
    """Full record for ``/face <id>`` — сводка + кollаж из снимков."""

    summary: FaceSummary
    meta: Dict[str, Any]
    created_at: Optional[float]
    last_encounter_ts: Optional[float]
    mode_recorded: Optional[str]
    # Reference first, then encounters ordered by filename ascending —
    # that matches what ``FaceStore._rotate_encounter_snapshots`` keeps
    # on disk (newer snapshots get higher numeric prefixes).
    tiles: List["FaceTile"] = field(default_factory=list)


@dataclass(frozen=True)
class FaceTile:
    """One image cell in the collage."""

    label: str  # "reference" / "000013" / "m-000007" (display name)
    raw_filename: str  # actual filename under person_dir
    width: int
    height: int
    jpeg_bytes: bytes


# ----------------------------------------------------------------------
# Low-level readers (no PIL, no numpy — keep this module's import cheap)
# ----------------------------------------------------------------------


def _safe_read_json(path: Path) -> Dict[str, Any]:
    """Read meta.json, return ``{}`` if the file is missing or corrupt.

    Same forgiveness policy as ``face_store_admin._read_meta``:
    a single broken record must not bring down ``/faces`` for everyone
    else. ``/face <id>`` then returns "not found" on the empty dict
    (the handler decides what to do).
    """
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
            if isinstance(data, dict):
                return data
            return {}
    except FileNotFoundError:
        return {}
    except (OSError, json.JSONDecodeError):
        logger.warning("face_card: failed to parse %s", path, exc_info=True)
        return {}


def _read_meta(person_dir: Path) -> Dict[str, Any]:
    return _safe_read_json(person_dir / META_FILENAME)


def _gallery_byte_size(person_dir: Path) -> int:
    """Bytes of ``embeddings.npy`` (0 if missing). Used by the ``/faces``
    table so the operator can see how heavy each gallery is on disk."""
    p = person_dir / EMBEDDINGS_FILENAME
    try:
        return p.stat().st_size
    except OSError:
        return 0


def _load_embeddings(person_dir: Path) -> Optional[List["np.ndarray"]]:
    """Load ``embeddings.npy`` into a list-of-vectors, or ``None``.

    Returns ``None`` if the file is missing or unloadable. We import
    numpy lazily inside the function so the rest of this module is
    usable in environments without numpy (the Telegram image runs with
    numpy present, but the unit tests that exercise the PIL collage
    builder do not need it).
    """
    p = person_dir / EMBEDDINGS_FILENAME
    if not p.exists():
        return None
    try:
        import numpy as np  # local import — see docstring

        arr = np.load(p)
        if arr.size == 0:
            return []
        return [arr[i].astype("float32").copy() for i in range(arr.shape[0])]
    except Exception:
        logger.warning("face_card: failed to load %s", p, exc_info=True)
        return None


def _cohesion_median(embeddings: Optional[List["np.ndarray"]]) -> Optional[float]:
    """Median pairwise cosine of an embeddings list, or ``None`` if
    fewer than two vectors (a pairwise cosine is not defined for one
    vector — see ``face_store._gallery_cohesion``)."""
    if not embeddings or len(embeddings) < 2:
        return None
    try:
        import numpy as np

        stack = np.stack(embeddings).astype("float32")
        # L2-normalize defensively (real ArcFace vectors are already
        # normalized, but a corrupt npy might not be).
        norms = np.linalg.norm(stack, axis=1, keepdims=True)
        norms = np.where(norms < 1e-9, 1.0, norms)
        normed = stack / norms
        sims = normed @ normed.T
        n = sims.shape[0]
        iu = np.triu_indices(n, k=1)
        if iu[0].size == 0:
            return None
        return float(np.median(sims[iu]))
    except Exception:
        logger.warning("face_card: cohesion computation failed", exc_info=True)
        return None


def _short_person_id(person_id: str) -> str:
    """8-char short id, matching what ``/face 4ff0ddc5`` already accepts
    from operators. We don't truncate silently — full id is also
    available in the detail view."""
    return person_id[:8]


def _list_encounter_files(person_dir: Path) -> List[Path]:
    """Sorted list of jpg/png files in ``encounters/`` (oldest first by
    filename — matches the numeric ``000013_face.jpg`` convention
    FaceStore uses). Skip dotfiles and the ``m-`` prefix; the latter is
    treated like any other filename by sorting."""
    enc = person_dir / ENCOUNTERS_DIRNAME
    if not enc.is_dir():
        return []
    files = [p for p in enc.iterdir() if p.is_file() and p.suffix.lower() in (".jpg", ".jpeg", ".png")]
    files.sort(key=lambda p: p.name)
    return files


def _has_reference(person_dir: Path) -> bool:
    return (person_dir / REFERENCE_FILENAME).is_file()


def _read_image(path: Path) -> Optional[Tuple[bytes, int, int]]:
    """Read a JPEG/PNG file and return (bytes, width, height) — width and
    height come from PIL if present, else (bytes, 0, 0). On any error
    returns ``None``. The collage builder skips ``None`` tiles.

    We don't decode the image here (PIL.Image.open is lazy); we use PIL
    only to learn the size for the caption. The collage itself decodes
    all tiles via PIL again — there's no point caching because we only
    ever read each tile once per request.
    """
    try:
        with open(path, "rb") as fh:
            data = fh.read()
        if not data:
            return None
        try:
            from PIL import Image  # local import

            with Image.open(io.BytesIO(data)) as im:
                w, h = im.size
            return data, int(w), int(h)
        except Exception:
            # PIL missing or image undecodable — return raw bytes, size 0x0.
            return data, 0, 0
    except OSError:
        return None


# ----------------------------------------------------------------------
# Public API used by handlers
# ----------------------------------------------------------------------


def list_people(
    root: str,
    *,
    root_exists: Optional[Callable[[str], bool]] = None,
) -> List[FaceSummary]:
    """Return one ``FaceSummary`` per record directory under ``root``.

    Sorted with named people first, strangers after (stable by person_id
    ascending within each group) — matches the spec in issue #3025
    ("Именованные — первыми").

    ``root_exists(root)`` is injected for tests so we can exercise both
    "directory missing" (empty list, not crash) and "directory present"
    without touching the real filesystem.
    """
    root_path = Path(root)
    if root_exists is not None and not root_exists(root):
        return []
    if not root_path.is_dir():
        return []

    out: List[FaceSummary] = []
    for entry in sorted(root_path.iterdir()):
        if not entry.is_dir():
            continue
        # Person_id is the directory name. The hex string lives in meta.json
        # too, but FaceStore always uses the dir name as the canonical id.
        person_id = entry.name
        meta = _read_meta(entry)
        if not meta:
            # Broken record — still surface it with whatever defaults,
            # so the operator knows it's there. Better noisy than invisible.
            name = None
            speaker_id = None
            encounter_count = 0
            created_at = None
            is_stranger = True
        else:
            name = meta.get("name") if meta.get("name") else None
            speaker_id = meta.get("speaker_id")
            try:
                encounter_count = int(meta.get("encounter_count") or 0)
            except (TypeError, ValueError):
                encounter_count = 0
            created_at = meta.get("created_at")
            is_stranger = name is None

        embeddings = _load_embeddings(entry)
        emb_count = len(embeddings) if embeddings is not None else 0
        cohesion = _cohesion_median(embeddings) if emb_count >= 2 else None
        bytes_ = _gallery_byte_size(entry)

        out.append(
            FaceSummary(
                person_id=person_id,
                person_id_short=_short_person_id(person_id),
                name=name,
                speaker_id=(str(speaker_id) if speaker_id else None),
                encounter_count=encounter_count,
                embeddings_count=emb_count,
                gallery_cohesion_median=cohesion,
                gallery_bytes=bytes_,
                is_stranger=is_stranger,
                has_reference=_has_reference(entry),
            )
        )

    out.sort(key=lambda s: (s.is_stranger, s.person_id))
    return out


def _resolve_by_query(records: List[FaceSummary], query: str) -> Optional[FaceSummary]:
    """Pick a record by short id (8 chars) OR by exact name (case-insensitive).
    Returns ``None`` when nothing matches. ``query`` is already stripped by
    the handler."""
    q = query.strip()
    if not q:
        return None
    q_lower = q.lower()
    # short id first (operator habit: paste 4ff0ddc5)
    for s in records:
        if s.person_id_short == q or s.person_id == q:
            return s
    for s in records:
        if s.name and s.name.lower() == q_lower:
            return s
    return None


def load_detail(
    root: str, person_id: str
) -> Optional[FaceDetail]:
    """Read everything ``/face <id>`` needs: meta.json + reference.jpg +
    encounters/*.jpg (with sizes), in order."""
    person_dir = Path(root) / person_id
    if not person_dir.is_dir():
        return None
    meta = _read_meta(person_dir)
    if not meta:
        # Directory exists but no meta.json — treat as "not found"
        # so the operator doesn't get a half-empty card.
        return None

    name = meta.get("name") if meta.get("name") else None
    speaker_id = meta.get("speaker_id")
    try:
        encounter_count = int(meta.get("encounter_count") or 0)
    except (TypeError, ValueError):
        encounter_count = 0
    embeddings = _load_embeddings(person_dir)
    emb_count = len(embeddings) if embeddings is not None else 0
    cohesion = _cohesion_median(embeddings) if emb_count >= 2 else None

    summary = FaceSummary(
        person_id=person_id,
        person_id_short=_short_person_id(person_id),
        name=name,
        speaker_id=(str(speaker_id) if speaker_id else None),
        encounter_count=encounter_count,
        embeddings_count=emb_count,
        gallery_cohesion_median=cohesion,
        gallery_bytes=_gallery_byte_size(person_dir),
        is_stranger=(name is None),
        has_reference=_has_reference(person_dir),
    )

    tiles: List[FaceTile] = []

    # reference first
    ref_path = person_dir / REFERENCE_FILENAME
    ref = _read_image(ref_path) if ref_path.is_file() else None
    if ref is not None:
        data, w, h = ref
        tiles.append(
            FaceTile(
                label="reference",
                raw_filename=REFERENCE_FILENAME,
                width=w,
                height=h,
                jpeg_bytes=data,
            )
        )

    # encounters sorted oldest -> newest
    for enc_path in _list_encounter_files(person_dir):
        # Skip encounters with face_snapshot=null semantics: those
        # don't exist as files on disk (FaceStore only writes a file
        # when face_snapshot is not None — see ``_maybe_store_snapshots``
        # in face_store.py), so there's nothing to filter here. The
        # mention in issue #3025 is about a different layer that the
        # bot doesn't see (live FaceStore state with null snapshots
        # that never made it to disk).
        tile = _read_image(enc_path)
        if tile is None:
            continue
        data, w, h = tile
        raw_name = enc_path.name
        # Display name: drop "_face" suffix (000013_face.jpg -> 000013),
        # drop file extension. Merged files keep their "m-" prefix.
        display = raw_name
        for suf in ("_face.jpg", "_face.jpeg", "_face.png", ".jpg", ".jpeg", ".png"):
            if display.endswith(suf):
                display = display[: -len(suf)]
                break
        tiles.append(
            FaceTile(
                label=display,
                raw_filename=raw_name,
                width=w,
                height=h,
                jpeg_bytes=data,
            )
        )

    return FaceDetail(
        summary=summary,
        meta=meta,
        created_at=(float(meta["created_at"]) if isinstance(meta.get("created_at"), (int, float)) else None),
        last_encounter_ts=(
            float(meta["last_encounter_ts"])
            if isinstance(meta.get("last_encounter_ts"), (int, float))
            else None
        ),
        mode_recorded=(str(meta["mode_recorded"]) if meta.get("mode_recorded") else None),
        tiles=tiles,
    )


# ----------------------------------------------------------------------
# Formatting (pure text — no Telegram types here, see commands.py)
# ----------------------------------------------------------------------


def format_faces_table(summaries: List[FaceSummary]) -> str:
    """Markdown table for ``/faces``. One row per person, named people
    first. The first column shows "имя (или «незнакомец»)" — that's the
    spec from issue #3025. We keep the column set deliberately small so
    the table fits a Telegram message without truncation (Telegram's
    limit is 4096 chars; 20 rows of this width ≈ 1.5 KB)."""
    if not summaries:
        return "ℹ️ Лицевая база пуста."
    lines = [
        "👥 *Лица в базе робота:*",
        "",
        "Имя | id | speaker | встреч | эмб. | cohesive | kB",
        "----|----|---------|--------|------|----------|----",
    ]
    for s in summaries:
        name_disp = s.name if s.name else "«незнакомец»"
        if s.name:
            # Escape Markdown-breaking chars in names.
            name_disp = name_disp.replace("`", "'").replace("_", "\\_")
        spk = s.speaker_id if s.speaker_id else "—"
        cohes = (
            f"{s.gallery_cohesion_median:.2f}"
            if s.gallery_cohesion_median is not None
            else "—"
        )
        kb = f"{s.gallery_bytes // 1024}" if s.gallery_bytes else "0"
        lines.append(
            f"{name_disp} | `{s.person_id_short}` | `{spk}` | {s.encounter_count} | "
            f"{s.embeddings_count} | {cohes} | {kb}"
        )
    return "\n".join(lines)


def format_face_summary(detail: FaceDetail) -> str:
    """Markdown text for ``/face <id>`` — сводка (no image; the image
    is sent as a separate ``reply_photo`` in the handler).

    Uses ``last_encounter_ts`` from meta.json explicitly labelled
    «последняя встреча по лицу» so the operator doesn't confuse it
    with the dialogue-level ``last_seen`` (ADR-0106) which lives
    elsewhere (issue #3025 spec)."""
    s = detail.summary
    name_disp = s.name if s.name else "«незнакомец»"
    if s.name:
        name_disp = name_disp.replace("`", "'").replace("_", "\\_")
    cohes = (
        f"{s.gallery_cohesion_median:.3f}"
        if s.gallery_cohesion_median is not None
        else "—"
    )
    lines = [
        f"🧑 *{name_disp}* (`{s.person_id_short}`)",
        "",
        f"speaker_id: `{s.speaker_id or '—'}`",
        f"создано: { _fmt_ts(detail.created_at)}",
        f"последняя встреча по лицу: { _fmt_ts(detail.last_encounter_ts)}",
        f"встреч: {s.encounter_count}",
        f"эмбеддингов: {s.embeddings_count}  (cohesion median: {cohes})",
        f"mode_recorded: {detail.mode_recorded or '—'}",
    ]
    return "\n".join(lines)


def _fmt_ts(ts: Optional[float]) -> str:
    """Human-readable timestamp (UTC) for the сводка. ``None`` →
    прочерк. We format in UTC because the operator and the robot sit in
    different timezones; showing local time would mislead."""
    if ts is None or ts <= 0:
        return "—"
    import datetime as _dt

    try:
        return _dt.datetime.fromtimestamp(ts, tz=_dt.timezone.utc).strftime(
            "%Y-%m-%d %H:%M UTC"
        )
    except (OverflowError, OSError, ValueError):
        return "—"


# ----------------------------------------------------------------------
# Collage (PIL — only here, only for ``/face``)
# ----------------------------------------------------------------------


def build_face_collage(
    detail: FaceDetail,
    *,
    columns: int = 4,
    rows: int = 3,
    tile_size: Tuple[int, int] = (240, 320),
    pad: int = 6,
    caption_height: int = 22,
) -> Optional[bytes]:
    """Compose a 4x3 grid (default — the spec from issue #3025) of the
    available tiles and return it as JPEG bytes. Empty cells are filled
    with white. Each cell carries a white caption strip below with the
    original size (e.g. ``Дэнчик 305x405``).

    Returns ``None`` if there's nothing usable (no tiles at all — the
    operator gets a "no snapshot yet" text in the handler instead).

    This function imports PIL lazily because the unit tests that only
    exercise ``list_people`` / ``format_faces_table`` don't need PIL
    installed.
    """
    if not detail.tiles:
        return None
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        logger.warning("face_card: PIL not available — cannot build collage")
        return None

    cols = max(1, columns)
    row_n = max(1, rows)
    tile_w, tile_h = tile_size
    cell_w = tile_w + 2 * pad
    cell_h = tile_h + caption_height + 2 * pad
    canvas_w = cols * cell_w
    canvas_h = row_n * cell_h

    canvas = Image.new("RGB", (canvas_w, canvas_h), color=(255, 255, 255))
    draw = ImageDraw.Draw(canvas)

    # font: try a common system font, fall back to PIL default
    font = _pick_font(caption_height - 4)

    name_label = (detail.summary.name or "незнакомец").replace("\n", " ")

    # Fill row-major, left-to-right then top-to-bottom.
    positions = [(c, r) for r in range(row_n) for c in range(cols)]
    for idx, (col, row) in enumerate(positions):
        x0 = col * cell_w + pad
        y0 = row * cell_h + pad
        if idx < len(detail.tiles):
            tile = detail.tiles[idx]
            try:
                tile_img = Image.open(io.BytesIO(tile.jpeg_bytes)).convert("RGB")
                tile_img = _cover_fit(tile_img, tile_w, tile_h)
                canvas.paste(tile_img, (x0, y0))
            except Exception:
                _draw_empty_cell(draw, x0, y0, tile_w, tile_h + caption_height)
            # caption strip
            caption = f"{name_label} {tile.width}x{tile.height}"
            _draw_caption(draw, x0, y0 + tile_h, tile_w, caption_height, caption, font)
        else:
            # empty cell — white with no caption
            _draw_empty_cell(draw, x0, y0, tile_w, tile_h + caption_height)

    buf = io.BytesIO()
    canvas.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def _pick_font(size: int):
    """Return a PIL ImageFont; try DejaVuSans first, fall back to default.

    We don't hard-fail if no font file is found — the default bitmap font
    is ugly but readable, and shipping a font with the bot image would
    bloat it for a feature that's diagnostic, not a customer surface.
    """
    try:
        from PIL import ImageFont

        for candidate in (
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/TTF/DejaVuSans.ttf",
        ):
            try:
                return ImageFont.truetype(candidate, size=size)
            except OSError:
                continue
        return ImageFont.load_default()
    except Exception:
        return None


def _draw_caption(draw, x: int, y: int, w: int, h: int, text: str, font) -> None:
    """White strip + black text."""
    draw.rectangle([x, y, x + w, y + h], fill=(255, 255, 255))
    try:
        draw.text((x + 4, y + 2), text, fill=(20, 20, 20), font=font)
    except Exception:
        # Font errors must not break the collage.
        pass


def _draw_empty_cell(draw, x: int, y: int, w: int, h: int) -> None:
    draw.rectangle([x, y, x + w, y + h], fill=(255, 255, 255), outline=(220, 220, 220))


def _cover_fit(img, w: int, h: int):
    """Resize-and-crop to exactly (w, h), preserving aspect ratio.

    We crop rather than letterbox so all tiles look uniform — Telegram
    shows images at varying widths and a letterboxed collage looks
    chaotic in a multi-row grid.
    """
    iw, ih = img.size
    if iw == 0 or ih == 0:
        return img
    target_ratio = w / h
    src_ratio = iw / ih
    if src_ratio > target_ratio:
        # too wide — crop sides
        new_w = int(ih * target_ratio)
        left = (iw - new_w) // 2
        img = img.crop((left, 0, left + new_w, ih))
    elif src_ratio < target_ratio:
        # too tall — crop top/bottom
        new_h = int(iw / target_ratio)
        top = (ih - new_h) // 2
        img = img.crop((0, top, iw, top + new_h))
    return img.resize((w, h))


# ----------------------------------------------------------------------
# Convenience: combined helper the handlers call directly
# ----------------------------------------------------------------------


def find_summary(
    root: str, query: str, *, root_exists: Optional[Callable[[str], bool]] = None
) -> Optional[FaceSummary]:
    """Locate a record by short id or name (case-insensitive). Used by
    ``/face <query>`` BEFORE loading the full detail."""
    summaries = list_people(root, root_exists=root_exists)
    return _resolve_by_query(summaries, query)


__all__ = [
    "META_FILENAME",
    "EMBEDDINGS_FILENAME",
    "REFERENCE_FILENAME",
    "ENCOUNTERS_DIRNAME",
    "FaceSummary",
    "FaceDetail",
    "FaceTile",
    "list_people",
    "load_detail",
    "find_summary",
    "format_faces_table",
    "format_face_summary",
    "build_face_collage",
]
