"""music_code_filter.py — code-safety / music-quality helpers для Renardo.

ADR-0134, Фаза 1 (#3014): шесть проходов очистки Renardo-кода, бывших
приватными методами ``MusicManager``, вынесены в :class:`MusicCodeFilter`.
``core.renardo_sanitizer`` остаётся публичным API для ``arranger`` и
тестов; этот класс — seam под ``#3014 Фаза 6``, где обе копии сольются.
"""
import ast as _ast
import re as _re
from typing import Dict, List, Optional, Tuple

# Compiled patterns — единая компиляция при импорте (как в renardo_sanitizer).
_BLOCKED_TOKENS = _re.compile(
    r"\b(import|os|sys|subprocess|shutil|socket|requests|urllib|http|ftplib|"
    r"importlib|builtins|__import__|__builtins__|__class__|__subclasses__|"
    r"open|exec|eval|compile|globals|locals|vars|delattr)\b"
)
# Reflection builtins, легитимные в Renardo, но только со строковым литералом
# в качестве имени атрибута (см. Clock.future(... setattr(Clock, "bpm", ...))).
_LITERAL_ATTR_BUILTINS: frozenset = frozenset({"getattr", "setattr", "hasattr"})

# Issue #1016 — music-quality guardrail.
_ABSOLUTE_FREQ_RE = _re.compile(
    r"\b(?:freq|frequency|hz|midinote|note)\s*=\s*(\d+(?:\.\d+)?)"
)
# Аргументы захватываются ДО КОНЦА СТРОКИ (live 02.09: вложенные скобки
# PGroup обрывали захват, и dur= не попадал в анализ).
_PLAYER_LINE_RE = _re.compile(
    r"^\s*(\w+)\s*>>\s*(\w+)\s*\((.*)\)\s*$", _re.MULTILINE
)
_DEV_PATTERN_RE = _re.compile(
    r"\.every\(|Pvar\(|pvar\(|linvar\(|var\(|Clock\.future|chop=|stutter|shuffle|reverse"
)
# Hard-blocked hardware constraints (live 20.08): chop= → щелчки на 16 kHz
# DAC; spack= → сырые глитчевые сэмплы pitchglitch-пака.
_CHOP_RE = _re.compile(r"\bchop\s*=\s*(?!0\b)")
_SPACK_NONZERO_RE = _re.compile(r"\bspack\s*=\s*[1-9]")

# Issue #1804 — физически смонтированы только d1-d3/p1-p3.
_ALLOWED_PLAYER_SLOTS: Tuple[str, ...] = ("d1", "d2", "d3", "p1", "p2", "p3")
_ALLOWED_PLAYER_SLOTS_SET: frozenset = frozenset(_ALLOWED_PLAYER_SLOTS)
_PLAYER_ASSIGN_RE = _re.compile(
    r"^(?P<indent>[ \t]*)(?P<name>[dpsl]\d+)(?P<arrow>\s*>>\s*)(?P<synth>\w+)\s*\(",
    _re.MULTILINE,
)

# Issue #1803 — длина play("...") задаёт период.
_PLAY_PATTERN_LEN_RE = _re.compile(r"play\((\s*)(['\"])([^'\"]*)\2")


def _is_dunder(name: str) -> bool:
    """True для ``__name__``-подобных имён."""
    return name.startswith("__") and name.endswith("__")


def _check_reflection_call(node: _ast.Call) -> Optional[str]:
    """Error для ``getattr``/``setattr``/``hasattr`` с нелитеральным
    атрибутом или dunder; иначе None. Вынесено для снижения CC."""
    if not isinstance(node.func, _ast.Name):
        return None
    if node.func.id not in _LITERAL_ATTR_BUILTINS or len(node.args) < 2:
        return None
    attr_arg = node.args[1]
    if not isinstance(attr_arg, _ast.Constant) or not isinstance(attr_arg.value, str):
        return (
            f"'{node.func.id}' допустим только со строковым литералом "
            "в качестве имени атрибута"
        )
    if _is_dunder(attr_arg.value):
        return f"Запрещённый доступ к dunder-атрибуту: '{attr_arg.value}'"
    return None


class MusicCodeFilter:
    """Шесть проходов очистки Renardo-кода из MusicManager. Чистые методы
    (без I/O). Инстанс создаётся в ``MusicManager.__init__`` и переиспользуется
    на каждый ``execute_code``. ``max_amp`` нужен только ``_cap_amp``."""

    __slots__ = ("_max_amp",)

    def __init__(self, max_amp: float) -> None:
        self._max_amp: float = max(0.0, min(1.0, float(max_amp)))

    # --- Security filter -----------------------------------------------------

    def _filter_code(self, code: str) -> Tuple[bool, str]:
        """Двухслойная проверка: blocklist + AST-проход (см. ``_filter_code_ast``)."""
        match = _BLOCKED_TOKENS.search(code)
        if match:
            return False, f"Запрещённый токен в коде: '{match.group()}'"
        return self._filter_code_ast(code)

    @staticmethod
    def _filter_code_ast(code: str) -> Tuple[bool, str]:
        """AST-слой фильтра безопасности (см. :meth:`_filter_code`)."""
        try:
            tree = _ast.parse(code)
        except SyntaxError as exc:
            return False, f"Синтаксическая ошибка в коде: {exc.msg}"

        for node in _ast.walk(tree):
            if isinstance(node, _ast.Attribute) and _is_dunder(node.attr):
                return False, f"Запрещённый доступ к dunder-атрибуту: '{node.attr}'"
            if isinstance(node, _ast.Name) and _is_dunder(node.id):
                return False, f"Запрещённое dunder-имя: '{node.id}'"
            if isinstance(node, _ast.Call):
                err = _check_reflection_call(node)
                if err is not None:
                    return False, err
        return True, ""

    # --- Issue #1016: dramaturgy validator -----------------------------------

    def _validate_music_code(self, code: str) -> Tuple[List[str], List[str]]:
        """Guardrail качества: HARD errors (абсолютные частоты, chop=, spack=)
        блокируют, WARNINGs (нет dur=; нет развивающих паттернов) идут в LLM."""
        errors: List[str] = []
        warnings: List[str] = []

        freq_match = _ABSOLUTE_FREQ_RE.search(code)
        if freq_match:
            errors.append(
                "Абсолютные частоты запрещены (Renardo ожидает ступени): "
                f"'{freq_match.group(0)}'. Используй степени, например "
                "p1 >> pluck([0,4,7]) или p1 >> pluck([0,4,7], oct=3)."
            )

        if _CHOP_RE.search(code):
            errors.append(
                "chop= запрещён — на 16 kHz DAC даёт щелчки, а не sidechain. "
                "Для дакинга используй amplify=var([1,0.3],[0.5,0.5])."
            )

        if _SPACK_NONZERO_RE.search(code):
            errors.append(
                "spack= не выбирает пак в этой сборке Renardo (параметр "
                "игнорируется — прозвучал бы пак 0 под видом пака 1). Убери "
                "spack= и бери вариант через sample=; жанровый луп — через "
                "loop('<имя>') из каталога лупов."
            )

        players = list(_PLAYER_LINE_RE.finditer(code))
        for m in players:
            player_name, synth, args = m.group(1), m.group(2), m.group(3)
            if synth == "play":
                continue
            if "dur" not in args:
                warnings.append(
                    f"У '{player_name} >> {synth}(...)' нет dur= — паттерн "
                    "будет звучать как staccato-щелчки. Добавь dur "
                    "(например dur=0.5 или dur=[0.5,0.25])."
                )

        if len(players) >= 2 and not _DEV_PATTERN_RE.search(code):
            warnings.append(
                "В коде нет развивающих паттернов (.every/Pvar/linvar/"
                "Clock.future) — трек будет звучать как статичный луп из "
                "4-8 нот. Добавь хотя бы один: .every(4, 'stutter'), "
                "lpf=linvar([500,4000], 16) или Pvar-гармонию."
            )

        return errors, warnings

    # --- Issue #1804: d4+/p4+ remap -----------------------------------------

    def _remap_illegal_slots(self, code: str) -> Tuple[str, Optional[str]]:
        """Переставить d4+/p4+/s*/l* в свободный d1-d3/p1-p3. play → d-слот,
        прочее → p-слот. Один недопустимый токен → один новый слот."""
        occupied: set = {
            m.group("name")
            for m in _PLAYER_ASSIGN_RE.finditer(code)
            if m.group("name") in _ALLOWED_PLAYER_SLOTS_SET
        }
        remapped: Dict[str, str] = {}
        errors: List[str] = []

        def _remap(m: _re.Match) -> str:
            name = m.group("name")
            synth = m.group("synth")
            if name in _ALLOWED_PLAYER_SLOTS_SET:
                return m.group(0)
            if name in remapped:
                new_name = remapped[name]
            else:
                preferred = (
                    _ALLOWED_PLAYER_SLOTS
                    if synth == "play"
                    else _ALLOWED_PLAYER_SLOTS[3:] + _ALLOWED_PLAYER_SLOTS[:3]
                )
                free = next((s for s in preferred if s not in occupied), None)
                if free is None:
                    errors.append(
                        f"'{name} >> {synth}(...)' вне d1-d3/p1-p3, а все "
                        "6 слотов уже заняты — слой некуда переставить. "
                        "Убери один из существующих слоёв или объедини "
                        "паттерны."
                    )
                    return m.group(0)
                occupied.add(free)
                remapped[name] = free
                new_name = free
            return f"{m.group('indent')}{new_name}{m.group('arrow')}{synth}("

        fixed_code = _PLAYER_ASSIGN_RE.sub(_remap, code)
        if errors:
            return code, "⛔ Недопустимые слоты плееров: " + " ".join(errors)
        return fixed_code, None

    # --- Issue #1803: play-pattern length ------------------------------------

    def _fix_pattern_length(self, code: str) -> str:
        """Достроить play("...") до ближайшей СВЕРХУ степени двойки. Хвостовые
        ``.`` снимаются до первой степени двойки; добивка только ``.``."""

        def _pow2_at_least(value: int) -> int:
            target = 1
            while target < value:
                target *= 2
            return target

        def _pad(m: _re.Match) -> str:
            ws, quote, pattern = m.group(1), m.group(2), m.group(3)
            if len(pattern) <= 1:
                return m.group(0)

            trimmed = pattern
            while (
                len(trimmed) > 1
                and _pow2_at_least(len(trimmed)) != len(trimmed)
                and trimmed[-1] == "."
            ):
                trimmed = trimmed[:-1]

            target = _pow2_at_least(len(trimmed))
            if target == len(trimmed):
                if trimmed == pattern:
                    return m.group(0)
                return f"play({ws}{quote}{trimmed}{quote}"

            padded = trimmed + "." * (target - len(trimmed))
            return f"play({ws}{quote}{padded}{quote}"

        return _PLAY_PATTERN_LEN_RE.sub(_pad, code)

    # --- Issue #1000: amp / oct caps ----------------------------------------

    def _cap_amp(self, code: str) -> str:
        """Ограничить ``amp=``/``amplify=`` до ``self._max_amp`` и ``oct=`` до 5."""
        max_amp = self._max_amp
        max_oct = 5

        def _cap_p(m: _re.Match) -> str:
            def _cap_num(n: _re.Match) -> str:
                return f"{min(float(n.group()), max_amp):.3g}"
            return "amp=P[" + _re.sub(r"\b\d+(?:\.\d*)?\b", _cap_num, m.group(1)) + "]"

        code = _re.sub(r"amp\s*=\s*P\[([^\]]+)\]", _cap_p, code)

        def _cap_n(m: _re.Match) -> str:
            return f"amp={min(float(m.group(1)), max_amp):.3g}"

        code = _re.sub(r"amp\s*=\s*(\d+(?:\.\d*)?)", _cap_n, code)

        def _cap_amplify_var(m: _re.Match) -> str:
            inner = m.group(1)

            def _cap_num(n: _re.Match) -> str:
                return f"{min(float(n.group()), max_amp):.3g}"

            inner = _re.sub(r"\b\d+(?:\.\d*)?\b", _cap_num, inner)
            return f"amplify=var({inner})"

        code = _re.sub(r"amplify\s*=\s*var\(([^)]+)\)", _cap_amplify_var, code)

        def _cap_amplify_n(m: _re.Match) -> str:
            return f"amplify={min(float(m.group(1)), max_amp):.3g}"

        code = _re.sub(r"amplify\s*=\s*(\d+(?:\.\d*)?)", _cap_amplify_n, code)

        def _cap_oct(m: _re.Match) -> str:
            return f"oct={min(int(m.group(1)), max_oct)}"

        code = _re.sub(r"oct\s*=\s*(\d+)", _cap_oct, code)
        return code


__all__ = ["MusicCodeFilter"]