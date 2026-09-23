"""test_issue_2809_identity_confirmation_dialogue.py

Issue #2809 (продолжение, после ревью координатора 23.09) -- переспрос
про tentative-личность.

Контекст: первая часть фикса (test_issue_2809_low_confidence_speaker_
does_not_leak_name.py) убрала риск назвать НЕЗНАКОМЦА чужим именем, но
как побочный эффект живой хозяин с confidence 0.757/0.771 (без
конкурента с другим именем в базе) перестал называться по имени вообще
-- регрессия для легитимного случая. Товарищ Шифу дал добро на
переспрос: PR #2822 (develop 8c7f0d61b) снял прежнее препятствие --
``<system_context>`` больше не отдельное system-сообщение, поэтому
подсказку-гипотезу можно класть в него без конфликта с тем PR.

Реализация (``speaker_id_node.py`` + ``dialogue_node.py``):

* ``speaker_id_node.classify_name_confidence`` возвращает не просто
  да/нет, а один из трёх исходов: ``NAME_CONFIDENT``,
  ``NAME_TENTATIVE_SINGLE`` (один похожий кандидат, конкурента с другим
  именем нет -- живой "Дэнчик"), ``NAME_TENTATIVE_CONTESTED`` (несколько
  похожих кандидатов с РАЗНЫМИ именами, разрыв мал -- n210: Борис vs
  Саша). ``_publish_result`` кладёт ``tentative_name``/``tentative_conf``/
  ``tentative_kind`` в payload; ``tentative_name`` есть ТОЛЬКО при
  ``single`` -- при ``contested`` имени нет НИГДЕ, даже как гипотезы
  (в n210 само слово "Борис" запрещено must_not_say -- вопрос "Борис,
  это ты?" завалил бы ту же приёмку).
* ``DialogueNode._handle_tentative_speaker`` (вызывается из
  ``_apply_speaker_identity``) один раз за сессию (якорь непрерывности --
  тот же ``..._session_gap_sec``, что у роста галереи в PR #2757) кладёт
  подсказку-гипотезу в ``_pending_identity_hint``, которую
  ``_build_dynamic_system_context`` превращает в ``<name_hypothesis>``
  (``single``) или нейтральный ``<identity_question_rule>``
  (``contested``, БЕЗ единого имени). Ответ на следующей реплике читает
  ``classify_identity_confirmation`` -- простая эвристика по целым словам
  ("да"/"нет" и т.п.), не NLU; неоднозначный ответ = отказ.
* "Да" -> ``_confirm_tentative_speaker`` мутирует ЛОКАЛЬНЫЙ снимок
  ``_current_speaker`` (не публикует наружу -- vision_face_node,
  ADR-0123 п.6/issue #2771, гипотезу не увидит) и переиспользует
  confident-путь ``[Spkr:...]`` целиком; плюс один раз просит рост
  галереи тем же путём, что PR #2757 (``/voice/speaker/register``).
* "Нет"/неоднозначно -> остаётся tentative до конца сессии, больше не
  спрашиваем. Не переспрос к PR #2798 (``_ask_identity_if_ambiguous``) --
  тот реагирует на ack регистрации, здесь такого ack нет.

Тест не поднимает ROS2 -- тот же приём, что в остальных тестах этого
пакета (``object.__new__``).
"""

from __future__ import annotations

import asyncio
import sys
import threading
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

import rob_box_voice.dialogue_node as dialogue_node_module  # noqa: E402
from rob_box_voice.dialogue_node import (  # noqa: E402
    DialogueNode,
    classify_identity_confirmation,
)

SPEAKER_ID = "1ae4b0ac-0000-0000-0000-000000000000"
BORIS_ID = "boris-id-0000-0000-0000-000000000000"


def _run(coro):
    return asyncio.run(coro)


@pytest.fixture()
def node():
    n = object.__new__(DialogueNode)
    n.get_logger = MagicMock(return_value=MagicMock())
    n._speaker_lock = threading.Lock()
    n._publish_speaker_observation = MagicMock()
    n._speaker_register_pub = MagicMock()
    n._identity_confirmations = {}
    n._pending_identity_hint = None
    # Прод-значения из config/dialogue_node.yaml.
    n._identity_question_session_gap_sec = 120.0
    n._identity_answer_window_sec = 180.0
    return n


class _FakeClock:
    """Подменяет ``time`` модуля dialogue_node: ``monotonic`` -- ручные
    часы, остальное -- настоящий ``time``. Глобальный ``time.monotonic``
    не трогаем -- на нём живёт event loop ``asyncio.run``."""

    def __init__(self, now: float = 0.0) -> None:
        self.now = now

    def monotonic(self) -> float:
        return self.now

    def __getattr__(self, name):
        import time as _time

        return getattr(_time, name)


@pytest.fixture()
def clock(monkeypatch):
    fake = _FakeClock(now=1000.0)
    monkeypatch.setattr(dialogue_node_module, "time", fake)
    return fake


def _logged(node) -> str:
    return "\n".join(
        str(c.args[0]) for c in node.get_logger.return_value.info.call_args_list
    )


def _tentative_single(speaker_id=SPEAKER_ID, name="Denchik", conf=0.771):
    return {
        "is_known": True,
        "speaker_id": speaker_id,
        "name": None,
        "tentative_name": name,
        "tentative_conf": conf,
        "tentative_kind": "single",
        "confidence": conf,
        "epithet": "Sobesednik",
    }


def _tentative_contested(speaker_id=BORIS_ID, conf=0.780):
    return {
        "is_known": True,
        "speaker_id": speaker_id,
        "name": None,
        # Issue #2809 -- speaker_id_node НЕ кладёт tentative_name для
        # contested вообще (см. test_publish_result в другом файле);
        # payload здесь воспроизводит это буквально.
        "tentative_conf": conf,
        "tentative_kind": "contested",
        "confidence": conf,
    }


# ─────────────────────────────────────────────────────────────────────────────
#  classify_identity_confirmation -- эвристика да/нет по целым словам
# ─────────────────────────────────────────────────────────────────────────────


class TestClassifyIdentityConfirmation:
    @pytest.mark.parametrize(
        "text",
        ["да", "Да, это я", "ага", "точно, это я", "конечно"],
    )
    def test_yes_variants(self, text):
        assert classify_identity_confirmation(text) is True

    @pytest.mark.parametrize(
        "text",
        ["нет", "не я", "нет, я другой человек", "обознался"],
    )
    def test_no_variants(self, text):
        assert classify_identity_confirmation(text) is False

    @pytest.mark.parametrize(
        "text",
        ["как дела", "что нового", "давай, пожалуй", "а что там по погоде"],
    )
    def test_unrelated_text_is_ambiguous(self, text):
        # "давай" не должно матчить "да" как подстроку -- сравнение по
        # целым словам.
        assert classify_identity_confirmation(text) is None

    def test_both_yes_and_no_signal_is_ambiguous(self):
        assert classify_identity_confirmation("да нет, не знаю") is None


# ─────────────────────────────────────────────────────────────────────────────
#  single -- вопрос с именем, максимум один раз за сессию
# ─────────────────────────────────────────────────────────────────────────────


class TestSingleHypothesis:
    def test_asks_once_with_name_then_stays_silent_on_ambiguous_reply(self, node):
        node._current_speaker = _tentative_single()

        result1 = _run(node._apply_speaker_identity("privet", speaker_context=None))
        assert "Denchik" not in result1  # имя никогда не идёт в user_input
        assert "[Speaker:tentative]" in result1
        assert node._pending_identity_hint == {
            "kind": "single",
            "name": "Denchik",
            "confidence": pytest.approx(0.771),
        }

        hint_lines = node._pending_identity_hint_lines()
        assert any("Denchik" in line for line in hint_lines)
        assert any("name_hypothesis" in line for line in hint_lines)
        # Одноразовая: второй вызов подряд (как если бы система собрала
        # system_context дважды по ошибке) ничего не возвращает.
        assert node._pending_identity_hint_lines() == []

        # Следующая реплика, ответ неоднозначный -- считается отказом,
        # больше не переспрашиваем, hint больше не появляется.
        result2 = _run(
            node._apply_speaker_identity("kak dela", speaker_context=None)
        )
        assert "[Speaker:tentative]" in result2
        assert "Denchik" not in result2
        assert node._pending_identity_hint is None

        result3 = _run(
            node._apply_speaker_identity("chto novogo", speaker_context=None)
        )
        assert "[Speaker:tentative]" in result3
        assert node._pending_identity_hint is None

    def test_confirmed_yes_uses_name_until_end_of_session(self, node):
        node._current_speaker = _tentative_single()
        _run(node._apply_speaker_identity("privet", speaker_context=None))

        node._current_speaker = _tentative_single()  # тот же speaker_id
        result = _run(
            node._apply_speaker_identity("да, это я", speaker_context=None)
        )
        assert "[Spkr:Denchik]" in result
        assert node._current_speaker["name"] == "Denchik"
        assert "tentative_name" not in node._current_speaker

        # Рост галереи запрошен ровно один раз (issue #2757 путь).
        assert node._speaker_register_pub.publish.call_count == 1
        import json as _json

        published = _json.loads(
            node._speaker_register_pub.publish.call_args.args[0].data
        )
        assert published == {"name": "Denchik", "speaker_id": SPEAKER_ID}

        # Следующая реплика той же сессии -- биометрия всё ещё шлёт
        # tentative (confidence не изменился), но подтверждение уже есть:
        # ведём себя как confident, БЕЗ повторной регистрации.
        node._current_speaker = _tentative_single()
        result2 = _run(
            node._apply_speaker_identity("kak pogoda", speaker_context=None)
        )
        assert "[Spkr:Denchik]" in result2
        assert node._speaker_register_pub.publish.call_count == 1  # не выросло

    def test_confirmed_no_stays_silent_until_end_of_session(self, node):
        node._current_speaker = _tentative_single()
        _run(node._apply_speaker_identity("privet", speaker_context=None))
        node._pending_identity_hint_lines()  # как будто system_context уже собрали

        node._current_speaker = _tentative_single()
        result = _run(
            node._apply_speaker_identity("нет, не я", speaker_context=None)
        )
        assert "[Speaker:tentative]" in result
        assert "Denchik" not in result
        assert node._pending_identity_hint is None
        assert node._speaker_register_pub.publish.call_count == 0

        # Ещё одна реплика той же сессии -- по-прежнему тихо, вопрос не
        # повторяем.
        node._current_speaker = _tentative_single()
        result2 = _run(
            node._apply_speaker_identity("chto tam", speaker_context=None)
        )
        assert "[Speaker:tentative]" in result2
        assert node._pending_identity_hint is None

    def test_new_session_after_gap_allows_asking_again(self, node):
        node._current_speaker = _tentative_single()
        _run(node._apply_speaker_identity("privet", speaker_context=None))
        state = node._identity_confirmations[SPEAKER_ID]
        assert state["asked"] is True

        # Отказали, сессия истекла (пауза больше gap).
        state["confirmed"] = False
        state["last_seen_at"] -= 999.0

        node._current_speaker = _tentative_single()
        _run(node._apply_speaker_identity("privet snova", speaker_context=None))
        new_state = node._identity_confirmations[SPEAKER_ID]
        assert new_state["asked"] is True
        assert new_state["confirmed"] is None  # новая сессия -- решения ещё нет
        assert node._pending_identity_hint is not None  # спросили заново


# ─────────────────────────────────────────────────────────────────────────────
#  contested -- ни одно имя кандидата не должно попасть в LLM-контекст
# ─────────────────────────────────────────────────────────────────────────────


class TestContestedHypothesis:
    def test_never_carries_any_candidate_name(self, node):
        """n210: голос похож на Бориса и Сашу одновременно. Ни в
        user_input, ни в подсказке-гипотезе не должно быть ни одного из
        двух имён -- must_not_say в n210 запрещает само слово "Борис"."""
        node._current_speaker = _tentative_contested()

        result = _run(
            node._apply_speaker_identity(
                "ya mimo shel, imya svoe ne nazovu", speaker_context=None
            )
        )
        assert "Boris" not in result
        assert "Sasha" not in result
        assert "[Speaker:tentative]" in result

        hint_lines = node._pending_identity_hint_lines()
        assert hint_lines  # вопрос всё же есть -- нейтральный
        joined = " ".join(hint_lines)
        assert "Boris" not in joined
        assert "Sasha" not in joined
        assert "identity_question_rule" in joined
        assert "зовут" in joined  # нейтральный "как тебя зовут?"

    def test_asks_at_most_once_per_session(self, node):
        node._current_speaker = _tentative_contested()
        _run(node._apply_speaker_identity("privet", speaker_context=None))
        assert node._identity_confirmations[BORIS_ID]["asked"] is True

        node._pending_identity_hint = None  # как будто уже собрали контекст
        node._current_speaker = _tentative_contested()
        _run(node._apply_speaker_identity("chto delaesh", speaker_context=None))
        # Второй раз подсказка не появляется -- заданный вопрос уже
        # интерпретируется как (неоднозначный) ответ, не новый повод спросить.
        assert node._pending_identity_hint is None


# ─────────────────────────────────────────────────────────────────────────────
#  Окна по часам -- живой прогон 35857257981 (act2b_identity_question)
# ─────────────────────────────────────────────────────────────────────────────


class TestAnswerWindowClock:
    """В живом прогоне "да, это я" пришло через 84s после "Саша, это ты?"
    (шаги харнесса ~60s + переспрос STT). При едином окне 30s состояние
    пересоздавалось с asked=False, ответ не читался, 'Speaker confirmed
    verbally' в логе не было, а n703 зеленел только потому, что LLM
    повторила имя из истории."""

    def test_answer_window_is_independent_of_session_gap(self, node, clock):
        """Даже при прежнем session gap 30s ответ через 84s читается --
        его держит identity_answer_window_sec, а не якорь сессии."""
        node._identity_question_session_gap_sec = 30.0
        node._current_speaker = _tentative_single()
        _run(node._apply_speaker_identity("privet", speaker_context=None))
        node._pending_identity_hint_lines()

        clock.now += 84.13
        node._current_speaker = _tentative_single()
        result = _run(node._apply_speaker_identity(
            "да, это я.", speaker_context=None))
        assert "[Spkr:Denchik]" in result
        assert node._identity_confirmations[SPEAKER_ID]["confirmed"] is True
        assert node._pending_identity_hint is None

    def test_answer_after_84s_is_consumed_live_timeline(self, node, clock):
        clock.now = 716.42
        node._current_speaker = _tentative_single()
        _run(node._apply_speaker_identity(
            "ты меня узнал по голосу", speaker_context=None))
        assert node._pending_identity_hint is not None
        node._pending_identity_hint_lines()  # system_context собран

        clock.now = 800.55  # +84.13s -- больше прежних 30s
        node._current_speaker = _tentative_single()
        result = _run(node._apply_speaker_identity(
            "да, это я.", speaker_context=None))
        assert "[Spkr:Denchik]" in result
        assert node._identity_confirmations[SPEAKER_ID]["confirmed"] is True
        assert node._speaker_register_pub.publish.call_count == 1
        assert "Speaker confirmed verbally: 'Denchik'" in _logged(node)

        clock.now = 866.33  # +65.78s -- n704, тоже больше 30s
        node._current_speaker = _tentative_single()
        result2 = _run(node._apply_speaker_identity(
            "а что ты обо мне запомнил", speaker_context=None))
        assert "[Spkr:Denchik]" in result2
        assert node._pending_identity_hint is None  # не переспрашиваем
        assert node._speaker_register_pub.publish.call_count == 1
        assert "Speaker verbal confirmation reused: 'Denchik'" in _logged(node)

    def test_pending_question_expires_after_answer_window(self, node, clock):
        node._current_speaker = _tentative_single()
        _run(node._apply_speaker_identity("privet", speaker_context=None))
        node._pending_identity_hint_lines()

        clock.now += 180.5  # дольше identity_answer_window_sec
        node._current_speaker = _tentative_single()
        result = _run(node._apply_speaker_identity(
            "да, это я", speaker_context=None))
        # Вопрос протух: "да" не считается ответом, вопрос задаётся заново.
        assert "[Spkr:Denchik]" not in result
        assert node._identity_confirmations[SPEAKER_ID]["confirmed"] is None
        assert node._pending_identity_hint is not None
        assert node._speaker_register_pub.publish.call_count == 0

    def test_declined_survives_harness_step_pause(self, node, clock):
        """n706 "нет" -> n707 через ~65s: не переспрашиваем, имени нет."""
        node._current_speaker = _tentative_single()
        _run(node._apply_speaker_identity("privet", speaker_context=None))
        node._pending_identity_hint_lines()

        clock.now += 65.0
        node._current_speaker = _tentative_single()
        _run(node._apply_speaker_identity("нет, не я", speaker_context=None))
        assert node._identity_confirmations[SPEAKER_ID]["confirmed"] is False

        clock.now += 65.0
        node._current_speaker = _tentative_single()
        result = _run(node._apply_speaker_identity(
            "как меня зовут", speaker_context=None))
        assert "Denchik" not in result
        assert node._pending_identity_hint is None

    def test_settled_answer_resets_after_session_gap(self, node, clock):
        """n705: пауза дольше identity_question_session_gap_sec после
        решённого "да" -- новая сессия, вопрос задаётся снова."""
        node._current_speaker = _tentative_single()
        _run(node._apply_speaker_identity("privet", speaker_context=None))
        node._pending_identity_hint_lines()
        clock.now += 60.0
        node._current_speaker = _tentative_single()
        _run(node._apply_speaker_identity("да, это я", speaker_context=None))

        clock.now += 120.5
        node._current_speaker = _tentative_single()
        result = _run(node._apply_speaker_identity(
            "ты ещё здесь", speaker_context=None))
        assert "[Spkr:Denchik]" not in result
        assert node._pending_identity_hint is not None
