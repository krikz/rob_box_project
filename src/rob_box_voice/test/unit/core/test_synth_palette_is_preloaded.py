"""
test_synth_palette_is_preloaded.py — промпт не должен предлагать синты,
которых нет в scsynth.

🔴 Живой инцидент 30.08.2026: робот «играл» музыку, которой не было слышно.
Лог scsynth за один вечер:

    745  *** ERROR: SynthDef rhpiano not found
     11  *** ERROR: SynthDef subbass not found
    760  FAILURE IN SERVER /s_new SynthDef not found

Цепочка отказа. Юзер просит «сыграй тему на рояле» → модель берёт ``rhpiano``
из палитры, которую ей предлагает мастер-промпт → в ``foxdot_init.sc`` этого
имени нет, значит SynthDef никогда не грузился → scsynth отбивает ``/s_new``
→ в цепочке ноты остаётся только обвязка без инструмента::

    group 26797 (5 нод)                 group 26827 (4 ноды)
      26798 startSound                    26828 startSound
      26799 warmpad     ← инструмент           ← 26829 ОТСУТСТВУЕТ
      26800 lowPassFilter                 26830 lowPassFilter
      26801 volume                        26831 volume
      26802 makeSound                     26832 makeSound

Renardo раздаёт id подряд, поэтому пропуск ровно одного номера перед
``lowPassFilter`` — это и есть отбитый ``/s_new``.

Коварство в том, что отказ МОЛЧАЛИВЫЙ на всех уровнях: ноды создаются
(замер во время прогона — 36-65 синтов, CPU 1.75%), ``execute_music_code``
возвращает «Код выполнен успешно» (``exec()`` Python-кода действительно
прошёл, про ответ scsynth он ничего не знает), и робот честно говорит
«играю». Не слышно только ушами.

Промпты предлагали 48 существующих синтов, прелоад покрывал 29.
``test_music_runtime_assets.py::test_foxdot_init_preloads_pianovel...``
появился ровно из-за такого же случая с ОДНИМ синтом — но инвариант тогда
не закрепили, и списки разошлись снова.

Не требует ROS2, SuperCollider и робота — только файлы репозитория.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest


def _repo_root(start: Path) -> Path:
    for parent in [start, *start.parents]:
        if (parent / "docker").is_dir() and (parent / "src").is_dir():
            return parent
    return start.parents[5]


REPO_ROOT = _repo_root(Path(__file__).resolve())
FOXDOT_INIT = REPO_ROOT / "docker" / "vision" / "voice_assistant" / "foxdot_init.sc"
MASTER_PROMPT = REPO_ROOT / "src" / "rob_box_voice" / "prompts" / "master_prompt_compact.txt"
MUSIC_SKILL_PROMPT = (
    REPO_ROOT / "src" / "rob_box_voice" / "prompts" / "skills" / "music_skill_prompt.txt"
)

#: ``d1 >> play("X..o")`` — не SynthDef, а сэмплер (внутри play1/play2).
NOT_A_SYNTHDEF = frozenset({"play"})

#: Слова-роли из строк палитры («melody: blip, arpy…»), не имена синтов.
ROLE_WORDS = frozenset({
    "melody", "bass", "pads", "brass", "glitch", "lead", "perc", "keys_role",
    "palette", "synth", "and", "the", "for", "use",
})


def _preloaded() -> set:
    """Имена из ``startupSynths`` + ``customSynths`` в foxdot_init.sc."""
    content = FOXDOT_INIT.read_text(encoding="utf-8")
    names: set = set()
    for var in ("startupSynths", "customSynths"):
        match = re.search(r"var " + var + r" = \[(.*?)\];", content, re.S)
        assert match, var + " не найден в foxdot_init.sc"
        names |= set(re.findall(r'"([^"]+)"', match.group(1)))
    return names


def _advertised() -> dict:
    """Синты, которые промпты предлагают модели → где именно.

    Берём только однозначное:

    * примеры кода ``p1 >> synthname(...)`` — их модель копирует буквально;
    * блок ``Synth palette — melody: … Bass: …`` в мастер-промпте.

    Свободный текст не парсим: там слишком много слов, похожих на имена.
    """
    found: dict = {}
    for path in (MASTER_PROMPT, MUSIC_SKILL_PROMPT):
        if not path.exists():
            continue
        text = path.read_text(encoding="utf-8")
        for m in re.finditer(r"[pdsl]\d\s*>>\s*([a-z][a-z0-9_]*)\s*\(", text):
            found.setdefault(m.group(1), set()).add(path.name + " (пример кода)")
        palette = re.search(r"Synth palette[^\n]*\n(?:\s{2}\S[^\n]*\n)*", text)
        if palette:
            for name in re.findall(r"\b([a-z][a-z0-9]{2,15})\b", palette.group(0)):
                found.setdefault(name, set()).add(path.name + " (Synth palette)")
    for word in ROLE_WORDS | NOT_A_SYNTHDEF:
        found.pop(word, None)
    return found


def test_prompts_advertise_a_palette_at_all() -> None:
    """Smoke: если извлечение сломается, основной тест станет пустым."""
    advertised = _advertised()
    assert len(advertised) >= 20, (
        "извлечено подозрительно мало имён (" + str(len(advertised)) + ") — "
        "проверь, не поменялся ли формат палитры в промптах"
    )
    for anchor in ("blip", "dub", "pianovel"):
        assert anchor in advertised, anchor + " должен извлекаться из промптов"


def test_every_advertised_synth_is_preloaded() -> None:
    """Всё, что промпт предлагает модели, обязано грузиться в scsynth.

    Незагруженный синт — не деградация звука, а ПОЛНАЯ тишина для своего
    слоя, и притом молчаливая: тул рапортует успех, робот говорит «играю».
    """
    preloaded = _preloaded()
    advertised = _advertised()
    missing = sorted(set(advertised) - preloaded)
    assert not missing, (
        "промпт предлагает модели синты, которых нет в прелоаде "
        "foxdot_init.sc — scsynth отобьёт /s_new, слой будет молчать:\n  "
        + "\n  ".join(
            name + " ← " + ", ".join(sorted(advertised[name])) for name in missing
        )
    )


def test_preload_waits_for_scsynth_after_each_load() -> None:
    """После каждого ``path.load`` в прелоаде обязан идти ``Server.default.sync``.

    ``path.load`` лишь компилирует определение в sclang; ``/d_recv`` уходит в
    scsynth по UDP без подтверждения. Без ожидания часть залпа терялась, а лог
    всё равно писал «preload ok» — из-за чего и ``validate_music_stack.py``,
    который грепает этот лог, давал ложный PASS. Со списком, выросшим с 29 до
    53, ожидание тем более обязательно.
    """
    # Считаем ТОЛЬКО исполняемые строки: и шапка файла, и комментарии ниже
    # описывают пайплайн словами «path.load», и наивный count() их ловит.
    code = "\n".join(
        line for line in FOXDOT_INIT.read_text(encoding="utf-8").splitlines()
        if not line.strip().startswith("//")
    )
    loads = code.count("path.load")
    syncs = code.count("Server.default.sync")
    # минус один load — обработчик OSCdef /foxdot, он вне Routine и грузит
    # то, что renardo прислал сам, по одному файлу за сообщение
    assert loads - 1 <= syncs, (
        "исполняемых path.load: " + str(loads) + ", Server.default.sync: "
        + str(syncs) + " — прелоад снова шлёт залпом без подтверждения"
    )
    assert syncs >= 2, "sync должен стоять и в startupSynths, и в customSynths"


def test_master_limiter_is_armed_after_preload_not_on_a_timer() -> None:
    """Лимитер ставится в конце прелоада, а не по фиксированной задержке.

    ``masterlimiter`` сам лежит в ``customSynths``: по таймеру его могло не
    оказаться в scsynth к моменту ``/s_new``.
    """
    content = FOXDOT_INIT.read_text(encoding="utf-8")
    assert "SystemClock.sched(2.0" not in content, (
        "лимитер снова ставится по таймеру — верни его в конец fork-блока"
    )
    assert "fork {" in content, "прелоад должен идти в Routine (fork), иначе sync невозможен"


@pytest.mark.parametrize("synth", ["rhpiano", "subbass", "arpy", "jbass", "keys"])
def test_synths_from_the_live_failure_are_preloaded(synth: str) -> None:
    """Поимённо — те, что реально отбивались в логе scsynth 30.08."""
    assert synth in _preloaded(), (
        synth + " снова не грузится; именно на нём робот молчал 30.08"
    )
