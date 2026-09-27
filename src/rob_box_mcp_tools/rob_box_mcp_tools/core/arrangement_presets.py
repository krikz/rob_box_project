"""arrangement_presets.py — пресеты ручек аранжировки (ADR-0132, PR-7).

Раньше «рецепты» под конкретные песни (Imperial March, Stranger Things,
«Кузнечик» …) жили КОДОМ в промпте ``composer.txt`` — готовым
``execute_music_code`` с зашитыми нотами и синтами. Каждая правка
требовала трогать промпт, а сам рецепт был невидим партитуре и ручкам
PR-4/PR-6. ADR-0132 §7: рецепты — ДАННЫЕ, набор ручек ``compose_music``
на конкретную мелодию, а не код.

Два хранилища:

* **shipped** — ``rob_box_mcp_tools/data/arrangement_presets.json``,
  часть пакета (реестр курируемых пресетов, миграция §7 composer.txt).
* **learned** — JSON-файл на диске рядом с
  :class:`core.generated_music_library.GeneratedMusicLibrary`
  (``$MUSIC_LIBRARY_PATH`` / ``/data/music_library``, та же
  персистентная точка): пресеты, которые модель сохранила через
  ``save_arrangement_preset`` после явной похвалы юзера.

``learned`` перекрывает ``shipped`` для того же ``melody_key`` — юзер,
явно похваливший трек и попросивший запомнить, знает лучше дефолта.

Ключ пресета — ``melody_key`` = ``rtttl_library`` ``rec["name"]``,
канонический слаг НАЙДЕННОЙ записи (не то, что написал юзер/модель в
``name=`` вызова) — так пресет находится независимо от того, каким
словом ("imperial march", "star wars", "дарт вейдер") модель к нему
обратилась в этот раз.
"""

from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timezone
from importlib.resources import as_file, files
from pathlib import Path
from typing import Any, Dict, Optional, Tuple, Union

from .compose_knobs import KNOB_PARAMS

__all__ = ["PRESET_KNOB_FIELDS", "ArrangementPresetStore"]

#: Поля, из которых может состоять ``knobs`` пресета (ADR-0132 PR-7,
#: постановка задачи §1): ручки ``compose_music`` (те же имена, что
#: :data:`core.compose_knobs.KNOB_PARAMS`) + тембры/форма/темп. Любое
#: другое поле молча отбрасывается и в shipped-, и в learned-хранилище —
#: пресет не может пронести произвольный параметр (``name``, ноты) в обход
#: контракта «ручки, а не рецепт».
PRESET_KNOB_FIELDS: Tuple[str, ...] = KNOB_PARAMS + (
    "lead_synth", "bass_synth", "pad_synth", "counter_synth",
    "drum_style", "form", "bpm",
)

#: Имя файла shipped-пресетов внутри пакета ``rob_box_mcp_tools/data/``.
_SHIPPED_NAME = "arrangement_presets.json"

#: Имя файла learned-пресетов рядом с ``GeneratedMusicLibrary``.
_LEARNED_NAME = "arrangement_presets.json"

_DEFAULT_ROOT = os.getenv("MUSIC_LIBRARY_PATH", "/data/music_library")


def _default_shipped() -> Union[Path, Any]:
    """Bundled ресурс (importlib.resources) → fallback на дерево исходников."""
    try:
        res = files("rob_box_mcp_tools.data").joinpath(_SHIPPED_NAME)
        if res.is_file():
            return res
    except (ModuleNotFoundError, TypeError):
        pass
    return Path(__file__).resolve().parent.parent / "data" / _SHIPPED_NAME


def _read_json(path: Union[Path, Any]) -> Dict[str, Any]:
    if isinstance(path, Path):
        if not path.is_file():
            return {}
        with path.open("r", encoding="utf-8") as fh:
            return json.load(fh)
    with as_file(path) as p:
        return _read_json(p)


def _clean_knobs(knobs: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Оставить только поля из :data:`PRESET_KNOB_FIELDS`, отбросить прочее."""
    return {k: v for k, v in (knobs or {}).items() if k in PRESET_KNOB_FIELDS}


class ArrangementPresetStore:
    """Пресеты ручек: shipped (пакет, только чтение) + learned (диск, RW).

    Args:
        shipped_path: свой путь к shipped JSON (тесты); по умолчанию —
            bundled ``data/arrangement_presets.json``.
        learned_root: каталог для learned-хранилища; по умолчанию —
            ``$MUSIC_LIBRARY_PATH`` (``/data/music_library``), та же точка,
            что ``GeneratedMusicLibrary``.
    """

    def __init__(
        self,
        shipped_path: Optional[Union[str, Path]] = None,
        learned_root: Optional[str] = None,
    ) -> None:
        self._shipped_path = Path(shipped_path) if shipped_path else _default_shipped()
        self._shipped: Dict[str, Dict[str, Any]] = _read_json(self._shipped_path)

        self._learned_root = Path(learned_root or _DEFAULT_ROOT)
        self._learned_path = self._learned_root / _LEARNED_NAME
        self._lock = threading.Lock()

    # ------------------------------------------------------------------

    def _read_learned(self) -> Dict[str, Dict[str, Any]]:
        if not self._learned_path.is_file():
            return {}
        try:
            with self._learned_path.open("r", encoding="utf-8") as fh:
                return json.load(fh)
        except (OSError, json.JSONDecodeError):
            return {}

    def get(self, melody_key: str) -> Optional[Dict[str, Any]]:
        """Пресет по ``melody_key`` — learned перекрывает shipped, или ``None``."""
        if not melody_key:
            return None
        with self._lock:
            learned = self._read_learned()
        preset = learned.get(melody_key) or self._shipped.get(melody_key)
        if preset is None:
            return None
        return {**preset, "knobs": _clean_knobs(preset.get("knobs"))}

    def save(
        self,
        melody_key: str,
        *,
        title: str,
        knobs: Dict[str, Any],
        note: str = "",
        approved_by_user_quote: str = "",
    ) -> Dict[str, Any]:
        """Сохранить learned-пресет (перезаписывает прежний для того же ключа).

        Возвращает сохранённую запись. Вызывающая сторона
        (``save_arrangement_preset``) отвечает за гейт похвалы — здесь
        только персистентность.
        """
        entry = {
            "title": title,
            "knobs": _clean_knobs(knobs),
            "note": note,
            "approved_by_user_quote": approved_by_user_quote,
            "created_at": datetime.now(timezone.utc).isoformat(),
        }
        with self._lock:
            self._learned_root.mkdir(parents=True, exist_ok=True)
            learned = self._read_learned()
            learned[melody_key] = entry
            tmp_path = self._learned_path.with_suffix(".json.tmp")
            with tmp_path.open("w", encoding="utf-8") as fh:
                json.dump(learned, fh, ensure_ascii=False, indent=2)
            os.replace(tmp_path, self._learned_path)
        return {"melody_key": melody_key, **entry}
