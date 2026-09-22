"""
test_tts_provider_selection.py — выбор TTS-провайдера для синтеза КОМАНД e2e.

Речь про провайдера, которым БИЛД-МАШИНА озвучивает команду в колонку, а не
про tts_node на роботе. Исторически харнесс умел только Yandex, и когда доступ
к папке Yandex Cloud отвалился (PERMISSION_DENIED), каждый шаг каждого прогона
падал "FAIL synth" — робота при этом никто не спрашивал (run 35533542706).

Проверяют:
  1. map_tts_voice() — перевод голоса сценария в каталог провайдера
  2. Инвариант различимости: anton/ermil/zahar/filipp остаются ЧЕТЫРЬМЯ
     разными голосами у каждого провайдера (иначе ломается диаризация в
     night_marathon act2/act3, где эти четыре голоса живут в одном сценарии)
  3. Голоса, названные каталогом провайдера, не переводятся повторно
  4. resolve_tts_provider() — явный провайдер берётся без пробы,
     auto идёт по очереди и останавливается на первом живом
  5. Contract: run_step синтезирует через synth_command (а не synth_yandex),
     --tts-provider разбирается, YANDEX_API_KEY больше не безусловный фатал

Run:
  python3 -m pytest tests/unit/e2e_scripts/test_tts_provider_selection.py -v --no-cov
"""

import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
E2E_SCRIPT = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"
E2E_LIB = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_lib.sh"
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "L-E2E Voice Test.yml"

# Голоса, которые реально встречаются в .github/e2e/scenarios/**.json.
SCENARIO_VOICES = ("anton", "ermil", "zahar", "filipp")

pytestmark = pytest.mark.skipif(shutil.which("bash") is None, reason="bash недоступен")


# Библиотека подмешивается в тело скрипта, а не source'ится по пути: на
# Windows-хосте найденный bash — это WSL, который не видит C:\... путей.
# read_text() заодно нормализует CRLF Windows-чекаута (core.autocrlf=true) —
# с хвостовым CR bash спотыкается о ";;" и о терминатор heredoc.
LIB_SRC = E2E_LIB.read_text(encoding="utf-8") if E2E_LIB.exists() else ""


def run_bash(body: str) -> tuple[int, str, str]:
    """Гоняет body в bash поверх функций из e2e_voice_lib.sh.

    Скрипт идёт через stdin (``bash -s``), а не через ``-c``: в теле есть
    кавычки и обратные слэши, и на Windows они не переживают сборку
    командной строки для WSL.
    """
    script = "\n".join([LIB_SRC, body, ""])
    # Кормим БАЙТАМИ, а не str: в текстовом режиме Python на Windows
    # переводит "\n" в CRLF прямо в трубе, и bash спотыкается о `$'\r'`.
    # utf-8 задан явно — в скрипте кириллица, а дефолтная локаль тут cp1252.
    proc = subprocess.run(
        ["bash", "-s"],
        input=script.encode("utf-8"),
        capture_output=True,
        timeout=30,
    )
    decode = lambda b: b.decode("utf-8", errors="replace").strip()  # noqa: E731
    return proc.returncode, decode(proc.stdout), decode(proc.stderr)


def map_voice(provider: str, voice: str) -> str:
    rc, out, err = run_bash('map_tts_voice "%s" "%s"' % (provider, voice))
    assert rc == 0, "map_tts_voice упал: %s" % err
    return out


# --- map_tts_voice ----------------------------------------------------------


class TestMapTtsVoice:
    """Голос сценария → голос выбранного провайдера."""

    def test_yandex_is_identity(self):
        """Yandex — исходный каталог сценариев, переводить нечего."""
        for voice in SCENARIO_VOICES:
            assert map_voice("yandex", voice) == voice

    # Таблица ИЗМЕРЕНА 22.09.2026 (4b3c4a8cf, PR #2743): старая пара
    # zahar/filipp = baya/xenia давала max-cos 0.722 — ровно на границе
    # IDENTIFY_THRESHOLD=0.72. Методика и сырые данные:
    # scripts/e2e/measure_tts_voice_distinctness.py,
    # evidence/tts-voice-distinctness-2026-09-22/.
    @pytest.mark.parametrize(
        "voice,expected",
        [
            ("anton", "aidar"),
            ("ermil", "eugene"),
            ("zahar", "kseniya"),
            ("filipp", "xenia"),
        ],
    )
    def test_silero_mapping(self, voice, expected):
        assert map_voice("silero", voice) == expected

    # Тоже измерено 22.09.2026: старая пара anton/ermil
    # (ReliableMan/HandsomeChildhoodFriend) = 0.739, а на живом роботе 0.846 —
    # акт 2 склеил двух дикторов в один профиль (run 35667281570).
    @pytest.mark.parametrize(
        "voice,expected",
        [
            ("anton", "Russian_ReliableMan"),
            ("ermil", "Russian_PessimisticGirl"),
            ("zahar", "Russian_CrazyQueen"),
            ("filipp", "Russian_AttractiveGuy"),
        ],
    )
    def test_minimax_mapping(self, voice, expected):
        assert map_voice("minimax", voice) == expected

    @pytest.mark.parametrize("provider", ["silero", "minimax"])
    def test_scenario_voices_stay_distinct(self, provider):
        """Инвариант диаризации.

        night_marathon act2/act3 проверяют speaker_tag «голос A vs голос B» и
        держат все четыре яндексовских голоса в одном сценарии. Если перевод
        схлопнет их в один-два speaker'а, сценарий станет зелёным, проверяя
        не то, что задумано, — поэтому различимость важнее совпадения пола.
        """
        mapped = [map_voice(provider, v) for v in SCENARIO_VOICES]
        assert len(set(mapped)) == len(SCENARIO_VOICES), mapped

    @pytest.mark.parametrize(
        "provider,voice",
        [
            ("silero", "aidar"),
            ("silero", "eugene"),
            ("minimax", "Russian_CrazyQueen"),
            ("minimax", "male-qn-qingse"),
        ],
    )
    def test_native_voice_passthrough(self, provider, voice):
        """--voice в каталоге провайдера не переводится повторно."""
        assert map_voice(provider, voice) == voice

    @pytest.mark.parametrize(
        "provider,default",
        [("yandex", "anton"), ("silero", "aidar"), ("minimax", "Russian_ReliableMan")],
    )
    def test_empty_voice_falls_back_to_default(self, provider, default):
        assert map_voice(provider, "") == default

    def test_mapped_voices_exist_in_registry(self):
        """Целевые голоса обязаны быть в PROVIDER_VOICES, иначе провайдер
        молча возьмёт свой дефолт и все шаги зазвучат одинаково."""
        registry = (
            REPO_ROOT
            / "src"
            / "rob_box_voice"
            / "rob_box_voice"
            / "tts_voice_registry.py"
        ).read_text(encoding="utf-8")
        for provider in ("silero", "minimax"):
            for voice in SCENARIO_VOICES:
                mapped = map_voice(provider, voice)
                assert '"%s"' % mapped in registry, (provider, voice, mapped)


# --- resolve_tts_provider ---------------------------------------------------


def extract_function(name: str) -> str:
    """Вырезает одну функцию из харнесса по закрывающей скобке в 1-й колонке.

    sed-диапазон ``/^name() {/,/^}$/`` здесь не годится для ВСЕХ функций:
    внутри synth_* живут python-heredoc'и, у которых ``}`` бывает в первой
    колонке (dict literal), и диапазон обрывается на середине. Для
    resolve_tts_provider (чистый bash, без heredoc) первая ``}`` — своя.
    """
    lines = E2E_SCRIPT.read_text(encoding="utf-8").split("\n")
    start = next(
        (i for i, l in enumerate(lines) if l.startswith("%s() {" % name)), None
    )
    assert start is not None, "функция %s не найдена в %s" % (name, E2E_SCRIPT)
    end = next((i for i in range(start + 1, len(lines)) if lines[i] == "}"), None)
    assert end is not None, "не найден конец функции %s" % name
    return "\n".join(lines[start : end + 1])


# Стаб пробы: вместо реального синтеза запоминает, кого потрогали.
RESOLVER_PRELUDE = """
log() { echo ">>> $*"; }
PROBED=""
tts_probe_provider() {
    PROBED="${PROBED}$1,"
    [ "$1" = "$ALIVE" ]
}
E2E_TTS_PROVIDER_RESOLVED=""
"""


def run_resolver(requested: str, alive: str, order: str = "yandex,minimax,silero"):
    body = "".join(
        [
            RESOLVER_PRELUDE,
            'E2E_TTS_PROVIDER="%s"\n' % requested,
            'ALIVE="%s"\n' % alive,
            'E2E_TTS_PROVIDER_ORDER="%s"\n' % order,
            extract_function("resolve_tts_provider"),
            "\nresolve_tts_provider; rc=$?\n",
            'echo "RESOLVED=[$E2E_TTS_PROVIDER_RESOLVED] PROBED=[$PROBED] RC=[$rc]"\n',
        ]
    )
    rc, out, err = run_bash(body)
    assert "syntax error" not in err, err
    return out


class TestResolveTtsProvider:
    """Кого харнесс выберет и кого при этом потрогает пробой."""

    def test_explicit_provider_is_not_probed(self):
        """Явный выбор не подменяется живым соседом.

        Иначе прогон с --tts-provider yandex уехал бы на silero и был бы
        «зелёным», проверив не то, что просили.
        """
        out = run_resolver("yandex", alive="silero")
        assert "RESOLVED=[yandex]" in out
        assert "PROBED=[]" in out
        assert "RC=[0]" in out

    def test_auto_takes_first_alive(self):
        out = run_resolver("auto", alive="minimax")
        assert "RESOLVED=[minimax]" in out
        assert "PROBED=[yandex,minimax,]" in out

    def test_auto_falls_through_to_silero(self):
        """Мёртвые облака больше не красят прогон: silero локальный."""
        out = run_resolver("auto", alive="silero")
        assert "RESOLVED=[silero]" in out
        assert "PROBED=[yandex,minimax,silero,]" in out

    def test_auto_reports_none_when_nothing_answers(self):
        out = run_resolver("auto", alive="nobody")
        assert "RESOLVED=[]" in out
        assert "RC=[1]" in out
        assert "E2E_TTS_PROVIDER none auto" in out

    def test_custom_order_is_honoured(self):
        out = run_resolver("auto", alive="silero", order="silero,yandex")
        assert "PROBED=[silero,]" in out
        assert "RESOLVED=[silero]" in out


# --- contract: харнесс и workflow -------------------------------------------


class TestHarnessContract:
    """Развязка обязана быть подключена, иначе она мёртвый код."""

    def test_run_step_uses_synth_command(self):
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        calls = re.findall(r'^\s*(?:if ! )?(synth_\w+) "\$text" "\$voice"', text, re.M)
        assert calls, "не найдено ни одного call-site синтеза шага"
        assert set(calls) == {"synth_command"}, calls

    @pytest.mark.parametrize(
        "fn", ["synth_yandex", "synth_minimax", "synth_silero", "synth_command"]
    )
    def test_provider_functions_exist(self, fn):
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        assert re.search(r"^%s\(\) \{" % fn, text, re.M), fn

    def test_tts_provider_flag_parsed(self):
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        assert "--tts-provider) E2E_TTS_PROVIDER=" in text

    def test_yandex_key_no_longer_unconditional_fatal(self):
        """Без ключа Yandex прогон обязан оставаться возможным (silero)."""
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        assert 'E2E_FATAL: YANDEX_API_KEY не задан"' not in text
        assert (
            '[ "$E2E_TTS_PROVIDER" = "yandex" ] && [ -z "${YANDEX_API_KEY:-}" ]' in text
        )

    def test_provider_recorded_as_artifact(self):
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        assert "OUT_DIR/tts_provider.json" in text

    def test_workflow_exposes_input_and_passes_it(self):
        wf = WORKFLOW.read_text(encoding="utf-8")
        assert "tts_provider:" in wf
        assert "--tts-provider $TTS_PROVIDER_CLEAN" in wf
        assert "export MINIMAX_API_KEY=" in wf
