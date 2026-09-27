#!/usr/bin/env python3
"""
test_register_publishes_result.py — обвязка speaker_id_node поверх
issue #2747 (растущая галерея) и issue #2748 (имя доезжает до лица).

До этой пары исправлений:
  * профиль диктора оставался с ОДНИМ эмбеддингом навсегда (issue #2747);
  * ``_do_register()`` публиковал ТОЛЬКО служебный ack
    (``{"event": "registered", ...}``) — vision_face_node ждёт
    ``is_known=true`` в ``/voice/speaker/result`` и в момент регистрации
    его никогда не получал (issue #2748).

ВАЖНО про механизм роста галереи (изменилось в PR #2757 после ревью):
первая версия растила галерею по акустике (успешный identify() при мягком
адаптивном пороге). Она была ОТКЛОНЕНА после проверки на реальных данных
робота — same-voice/cross-voice cosine пересекаются целиком, любой
акустический порог либо не узнаёт хозяина, либо принимает ~90% чужих (см.
docstring test_gallery_warmup.py). Теперь галерея растёт по НЕПРЕРЫВНОСТИ
СЕССИИ: ``_do_register()`` открывает ``_growth_session`` (после явной
регистрации — «человек только что представился»), и
``_apply_growth_session()`` дописывает эмбеддинг в КАЖДУЮ следующую реплику,
пока сессия не прервалась разрывом (``gallery_growth_session_gap_sec``),
не упёрлась в потолок (``GALLERY_WARMUP_SIZE``) или не встретила
уверенное (обычный identify(), калиброванный порог) опознание ДРУГОГО,
уже известного спикера (вето поверх якоря).

Тесты здесь — про НОДУ (в отличие от test_gallery_warmup.py, который
проверяет чистую логику SpeakerDatabase):
    1. ``_do_register()`` публикует ДВА сообщения: ack (не тронут) и новый
       SpeakerMatch с ``is_known=true``, ``source="register"``, и открывает
       growth-сессию.
    2. ``_process_utterance()`` растит галерею по сессии НЕЗАВИСИМО от
       исхода identify() (в т.ч. когда голос НЕ узнан обычным путём).
    3. Growth-сессия закрывается по таймауту разрыва, по потолку галереи
       и по вето (уверенное опознание другого человека).
    4. ``_process_utterance()`` в ветке ``pending_name`` НЕ публикует
       второе, потенциально расходящееся сообщение — доверяет
       ``_do_register()``.

Тот же приём, что в test_epithet_wiring.py: ``SpeakerIdNode.__init__`` не
вызывается (там ROS-параметры, ThreadPool, resemblyzer warmup) — нода
собирается через ``object.__new__`` и получает только те поля, которые
нужны проверяемому коду. ``embed_audio_ex`` подменяется на фиксированный
вектор с явной длительностью речи (issue #2747) — resemblyzer недоступен
на dev-машине / в CI.
"""

from __future__ import annotations

import collections
import json
import sys
import threading
import types
from pathlib import Path
from typing import Deque, Optional, Tuple
from unittest.mock import MagicMock

import numpy as np
import pytest

# ``audio_common_msgs`` не покрыт общим conftest (его тянет только
# speaker_id_node) — доставляем заглушку до импорта ноды (тот же приём,
# что в test_epithet_wiring.py).
_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice import speaker_id_node as sid_node  # noqa: E402


class _Embed:
    """Заглушка ``EmbedResult`` из ``speaker_embeddings`` (issue #2747).

    Нода теперь берёт у БД не голый вектор, а вектор ВМЕСТЕ с
    длительностью РЕЧИ после VAD: гейт регистрации обязан мерить речь, а
    не длину окна записи. Свой мини-класс, а не импорт настоящего, — по
    той же причине, что и остальные заглушки в этом файле: тянуть модуль
    ради трёх полей значит тянуть resemblyzer, которого на dev-машине и
    в CI нет.
    """

    def __init__(self, embedding, voiced_sec: float = 5.0) -> None:
        self.embedding = embedding
        self.voiced_sec = voiced_sec
        self.raw_sec = max(voiced_sec, 0.0)

    @property
    def voiced_ratio(self) -> float:
        return (self.voiced_sec / self.raw_sec) if self.raw_sec > 0 else 0.0


from rob_box_voice.utils.speaker_embeddings import SpeakerDatabase  # noqa: E402


def _embedding(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


def _degraded(base: np.ndarray, alpha: float, noise_seed: int) -> np.ndarray:
    noise = _embedding(noise_seed)
    v = base + alpha * noise
    return v / np.linalg.norm(v)


class _FakePublisher:
    """Собирает опубликованные ``String``-сообщения как декодированный JSON."""

    def __init__(self) -> None:
        self.messages: list[dict] = []

    def publish(self, msg) -> None:
        self.messages.append(json.loads(msg.data))


@pytest.fixture()
def node(tmp_path, monkeypatch):
    """SpeakerIdNode без ROS-инициализации, с реальной БД во временном файле.

    Калиброванные модульные константы фиксируются monkeypatch'ем — тест не
    должен зависеть от того, что происходит в других test_*.py, которые
    патчат те же атрибуты speaker_embeddings (общий модуль в sys.modules).
    """
    import rob_box_voice.utils.speaker_embeddings as se_mod

    monkeypatch.setattr(se_mod, "IDENTIFY_THRESHOLD", 0.72)
    monkeypatch.setattr(se_mod, "REGISTER_MATCH_THRESHOLD", 0.75)
    monkeypatch.setattr(se_mod, "GALLERY_WARMUP_SIZE", 5)
    # Issue #2769 — фиксируем явно, чтобы тест не зависел от того, что
    # значение по умолчанию когда-нибудь изменится.
    monkeypatch.setattr(se_mod, "MIN_REGISTER_AUDIO_DURATION_SEC", 3.0)

    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._db = SpeakerDatabase(str(tmp_path / "speakers.db"))
    instance._speech_log = {}
    instance._speech_log_lock = threading.Lock()
    instance._sample_rate = 16000
    instance._recent_embeddings: Deque[Tuple[float, np.ndarray, float]] = collections.deque(
        maxlen=20
    )
    instance._MAX_EMBED_AGE_SEC = 30.0
    instance._pending_register_name: Optional[str] = None
    instance._pending_register_lock = threading.Lock()
    instance._result_pub = _FakePublisher()
    instance.get_logger = MagicMock(return_value=MagicMock())
    # epithet_request publisher — _ensure_epithet -> _assign_epithet ->
    # _request_llm_epithet его читает через getattr(..., None).
    instance._epithet_request_pub = None
    # Issue #2747 — growth-сессия (session-anchor рост галереи).
    instance._growth_session_gap_sec = 30.0
    instance._growth_session = None
    # Issue #2829 (ADR-0131 PR-2, координатор-ревью) -- нижний порог
    # + переиспользуемый gap-порог #2809 для growth-гейта.
    instance._growth_owner_min_score = 0.65
    instance._name_confidence_min_gap = 0.15
    yield instance
    instance._db.close()


# ---------------------------------------------------------------------------
# 1. _do_register публикует ack + is_known=true source="register"
# ---------------------------------------------------------------------------


def test_do_register_publishes_ack_and_speaker_match(node):
    emb = _embedding(1)
    node._do_register("Деньчик", emb, speaker_id=None)

    kinds = [m.get("event") for m in node._result_pub.messages]
    assert "registered" in kinds, "старый ack не должен пропасть (обратная совместимость)"

    # Issue #2748 — второе сообщение: полноценный SpeakerMatch.
    match_msgs = [m for m in node._result_pub.messages if m.get("is_known") is True]
    assert len(match_msgs) == 1, (
        f"ожидалось ровно одно is_known=true сообщение, получено "
        f"{len(match_msgs)}: {node._result_pub.messages!r}"
    )
    payload = match_msgs[0]
    assert payload["name"] == "Деньчик"
    assert payload["source"] == "register"
    assert payload["speaker_id"]
    assert payload["confidence"] > 0.99, "self-similarity только что записанного эмбеддинга ~1.0"


def test_do_register_ack_message_unchanged_shape(node):
    """dialogue_node/mcp_server/voice_adapter завязаны на форму ack —
    регрессия здесь тихо сломает их обработку 'event'=='registered'."""
    emb = _embedding(2)
    node._do_register("Саша", emb, speaker_id=None)

    ack = next(m for m in node._result_pub.messages if m.get("event") == "registered")
    assert set(ack) >= {"event", "name", "speaker_id", "reused_profile"}
    assert ack["name"] == "Саша"
    assert ack["reused_profile"] is False


def test_do_register_opens_growth_session(node):
    """Issue #2747 — регистрация открывает growth-сессию для этого speaker_id."""
    emb = _embedding(3)
    node._do_register("Шифу", emb, speaker_id=None)

    session = node._growth_session
    assert session is not None
    assert session["name"] == "Шифу"
    assert session["count"] == 0
    match_msgs = [m for m in node._result_pub.messages if m.get("is_known") is True]
    assert session["speaker_id"] == match_msgs[0]["speaker_id"]


def test_do_register_on_name_conflict_still_publishes_own_match(node):
    """ADR-0127: конфликт имён заводит ОТДЕЛЬНЫЙ профиль — is_known=true
    обязан указывать на НОВЫЙ профиль (Борис), а не на чужой (Саша)."""
    base = _embedding(700)
    similar = _degraded(base, alpha=0.6, noise_seed=701)  # cos ~0.86 > REGISTER_MATCH_THRESHOLD

    node._do_register("Саша", base, speaker_id=None)
    node._result_pub.messages.clear()

    node._do_register("Борис", similar, speaker_id=None)

    match_msgs = [m for m in node._result_pub.messages if m.get("is_known") is True]
    assert len(match_msgs) == 1
    assert match_msgs[0]["name"] == "Борис"
    ack = next(m for m in node._result_pub.messages if m.get("event") == "registered")
    assert "voice_conflict" in ack


# ---------------------------------------------------------------------------
# 1b. issue #2769 — _do_register отклоняет реплику короче
#     MIN_REGISTER_AUDIO_DURATION_SEC: профиль не создаётся, вместо
#     обычного ack публикуется event="register_error".
# ---------------------------------------------------------------------------


def test_do_register_publishes_name_twin_in_ack(node):
    """issue #2747 — середина цепочки: нода обязана пробросить повод наружу.

    Концы цепочки проверены отдельно (БД находит тёзку —
    ``test_issue_2747_name_twin.py``; диалог переспрашивает —
    ``test_dialogue_node.py::TestIdentityClarification``). Без этого теста
    середина оставалась бы непокрытой: если поле не доедет до ack, обе
    половины останутся зелёными, а робот молча заведёт второго «Дэнчика» —
    ровно то, что наблюдалось живьём 22.09.2026.
    """
    base = _embedding(900)
    # Голос заметно НИЖЕ порога слияния, имя то же — это тёзка, а не
    # обычное слияние. alpha=1.8 даёт cos ~= 1/sqrt(1+alpha^2) ~= 0.49:
    # больший alpha — БОЛЬШЕ шума и МЕНЬШЕ косинус (см. _degraded выше),
    # поэтому здесь он больше, чем 0.6 в тесте voice_conflict, а не меньше.
    far = _degraded(base, alpha=1.8, noise_seed=901)

    node._do_register("Дэнчик", base, speaker_id=None)
    first = node._db.list_speakers()[0]["id"]
    node._result_pub.messages.clear()

    node._do_register("Дэнчик", far, speaker_id=None)

    assert len(node._db.list_speakers()) == 2, (
        "профиль заводится отдельный — данные целы (инвариант ADR-0127)"
    )
    ack = next(m for m in node._result_pub.messages if m.get("event") == "registered")
    assert "name_twin" in ack, "повод переспросить обязан доехать до dialogue_node"
    assert ack["name_twin"]["speaker_id"] == first
    assert ack["name_twin"]["name"] == "Дэнчик"
    assert isinstance(ack["name_twin"]["score"], float), (
        "в поводе стоит число — оператор должен видеть, насколько близко "
        "было решение"
    )


def test_do_register_plain_registration_has_no_twin_in_ack(node):
    """Незнакомое имя — ack чистый, переспрашивать не о чем."""
    node._do_register("Шифу", _embedding(902), speaker_id=None)

    ack = next(m for m in node._result_pub.messages if m.get("event") == "registered")
    assert "name_twin" not in ack
    assert "voice_conflict" not in ack


def test_do_register_rejects_audio_shorter_than_register_floor(node):
    emb = _embedding(800)

    result = node._do_register("Шифу", emb, speaker_id=None, duration_sec=1.0)

    assert result is False, "_do_register должен сообщить о неудаче вызывающему коду"
    assert node._db.list_speakers() == [], "короткая реплика не должна создать профиль"
    assert node._growth_session is None, "growth-сессия не должна открыться на отказе"

    error_acks = [m for m in node._result_pub.messages if m.get("event") == "register_error"]
    assert len(error_acks) == 1
    ack = error_acks[0]
    assert ack["error"] == "too_short"
    assert ack["name"] == "Шифу"
    assert ack["duration_s"] == pytest.approx(1.0)
    assert ack["min_required_s"] == pytest.approx(3.0)

    # Никакого is_known=true — эталон не был создан, self-match невозможен.
    assert [m for m in node._result_pub.messages if m.get("is_known") is True] == []


def test_do_register_accepts_audio_at_or_above_register_floor(node):
    """Регрессия: обычный (долгий) путь не должен был сломаться правкой."""
    emb = _embedding(801)

    result = node._do_register("Шифу", emb, speaker_id=None, duration_sec=5.0)

    assert result is True
    assert len(node._db.list_speakers()) == 1
    assert node._growth_session is not None


def test_do_register_without_duration_is_not_gated(node):
    """Вызовы без duration_sec (например, из _on_register_request, когда
    в буфере почему-то не оказалось значения) сохраняют старое поведение —
    не начинают внезапно отказывать."""
    emb = _embedding(802)

    result = node._do_register("Шифу", emb, speaker_id=None)

    assert result is True
    assert len(node._db.list_speakers()) == 1


# ---------------------------------------------------------------------------
# 2. _process_utterance / _apply_growth_session: рост галереи по якорю
#    непрерывности сессии, НЕ по акустике (issue #2747, PR #2757 ревью)
# ---------------------------------------------------------------------------


def test_growth_session_grows_gallery_even_when_identify_fails(node):
    """КЛЮЧЕВОЙ тест новой версии: рост НЕ зависит от исхода identify().

    Калиброванный порог 0.72 НЕ пропускает cos~0.68 -- обычная
    идентификация даёт unknown. Но раз growth-сессия открыта (после явной
    регистрации) И похожесть на владельца всё же выше нижнего
    growth-порога (0.65, issue #2829 координатор-ревью), эмбеддинг всё равно
    дописывается в галерею -- личность подтверждена НЕПРЕРЫВНОСТЬЮ
    сессии, а не полной похожестью голоса.

    Issue #2829 (координатор-ревью PR-2): раньше здесь стоял cos~0.523 --
    то самое измерение issue #2747, взятое ДО того, как у growth-гейта
    появился нижний порог 0.65. Теперь 0.523 < 0.65, и рост на нём
    ЗАКОНОМЕРНО не произошёл бы (главный сценарий #2829 п.3 -- чужой
    голос с похожим на владельца score не должен дописываться). Новое
    число 0.68 лежит РОВНО в валидной полосе [growth_owner_min_score,
    identify_threshold) = [0.65, 0.72) -- тест по-прежнему проверяет то же самое
    утверждение ("growth работает даже когда identify() честно
    вернул None"), просто на числе, которое остаётся в силе после
    добавления нижнего порога."""
    base = _embedding(10)
    node._do_register("Деньчик", base, speaker_id=None)
    sid = node._growth_session["speaker_id"]
    assert node._db.gallery_size(sid) == 1
    node._result_pub.messages.clear()

    alpha = float((1.0 / 0.68 ** 2 - 1.0) ** 0.5)
    second = _degraded(base, alpha=alpha, noise_seed=11)
    node._db.embed_audio_ex = MagicMock(return_value=_Embed(second))

    node._process_utterance(bytes(2000))

    assert node._db.gallery_size(sid) == 2, (
        "growth-сессия обязана дописать эмбеддинг НЕЗАВИСИМО от identify(), "
        "пока похожесть на владельца выше growth_owner_min_score"
    )
    # identify() на калиброванном пороге реплику не узнал -- is_known=false,
    # рост галереи никак не подделывает результат обычной идентификации.
    assert node._result_pub.messages == [{"is_known": False}]
    assert node._growth_session["count"] == 1


def test_growth_session_stops_at_warmup_size_cap(node):
    base = _embedding(20)
    node._do_register("Деньчик", base, speaker_id=None)
    sid = node._growth_session["speaker_id"]
    for i in range(1, 5):
        node._db.register("Деньчик", _degraded(base, 0.1, 2000 + i), speaker_id=sid)
    assert node._db.gallery_size(sid) == 5

    node._db.embed_audio_ex = MagicMock(return_value=_Embed(_degraded(base, 0.1, 2999)))
    node._process_utterance(b"\x00\x00" * 1000)

    assert node._db.gallery_size(sid) == 5, "галерея не должна расти после потолка"
    assert node._growth_session is None, "сессия обязана закрыться на потолке"


def test_growth_session_closes_after_gap_timeout(node, monkeypatch):
    """Разрыв дольше gallery_growth_session_gap_sec — сессия закрывается,
    следующая реплика в галерею НЕ дописывается."""
    base = _embedding(25)
    node._do_register("Деньчик", base, speaker_id=None)
    sid = node._growth_session["speaker_id"]
    assert node._db.gallery_size(sid) == 1

    # Отматываем "последнюю реплику сессии" далеко в прошлое.
    node._growth_session["last_utterance_at"] -= node._growth_session_gap_sec + 1.0

    node._db.embed_audio_ex = MagicMock(return_value=_Embed(_degraded(base, 0.1, 2500)))
    node._process_utterance(b"\x00\x00" * 1000)

    assert node._db.gallery_size(sid) == 1, "разрыв больше окна — рост не должен случиться"
    assert node._growth_session is None, "сессия обязана закрыться по таймауту"


def test_tts_finished_resets_gap_so_robot_monologue_is_not_idle(node):
    """Речь робота НЕ должна засчитываться человеку в простой (issue #2747).

    Регрессия на прогон 35781881888: галереи Саши и Бориса остались по
    одному вектору при потолке 5, потому что разрыв мерился от прошлой
    реплики ЧЕЛОВЕКА и включал время, пока робот произносил ответ
    («закрыта по таймауту 33.7s > 30.0s» и «61.7s > 30.0s»).
    """
    base = _embedding(26)
    node._do_register("Деньчик", base, speaker_id=None)
    sid = node._growth_session["speaker_id"]

    # Робот говорил дольше, чем всё окно разрыва, и только потом договорил.
    node._growth_session["last_utterance_at"] -= node._growth_session_gap_sec + 1.0
    node._on_tts_finished(types.SimpleNamespace(data="speech-id-не-важен"))

    node._db.embed_audio_ex = MagicMock(return_value=_Embed(_degraded(base, 0.1, 2500)))
    node._process_utterance(b"\x00\x00" * 1000)

    assert node._growth_session is not None, "сессия не должна закрыться из-за речи робота"
    assert node._db.gallery_size(sid) == 2, "после ответа робота реплика обязана дописаться"


def test_tts_finished_without_session_is_noop(node):
    """Речь робота сама по себе не открывает growth-сессию: она ничего не
    говорит о том, КТО стоит у микрофона."""
    assert node._growth_session is None
    node._on_tts_finished(types.SimpleNamespace(data="speech-id"))
    assert node._growth_session is None


def test_growth_session_vetoed_by_confident_different_speaker(node):
    """Реплика уверенно (калиброванный порог) опознана как ДРУГОЙ, уже
    известный спикер — сильное прямое свидетельство против якоря. Рост не
    должен случиться, сессия должна закрыться (issue #2747, PR #2757
    ревью: «высокий косинус — дополнительное условие ПОВЕРХ якоря, вето,
    а не самостоятельный порог доверия»)."""
    anchor_base = _embedding(30)
    node._do_register("Деньчик", anchor_base, speaker_id=None)
    sid_anchor = node._growth_session["speaker_id"]

    other_base = _embedding(31)  # ортогональный голос — другой человек
    sid_other = node._db.register("Пётр", other_base)
    # Достаточно эмбеддингов, чтобы обычный identify() уверенно узнал Петра
    # по калиброванному порогу на его собственный (почти идентичный) голос.
    node._result_pub.messages.clear()

    node._db.embed_audio_ex = MagicMock(return_value=_Embed(other_base))
    node._process_utterance(b"\x00\x00" * 1000)

    assert node._db.gallery_size(sid_anchor) == 1, "чужая реплика не должна попасть в Деньчика"
    assert node._db.gallery_size(sid_other) == 1, "вето не дописывает и в профиль Петра"
    assert node._growth_session is None, "сессия обязана закрыться при опровержении якоря"
    match_msgs = [m for m in node._result_pub.messages if m.get("is_known") is True]
    assert match_msgs[0]["name"] == "Пётр", "обычная идентификация продолжает работать как раньше"


def test_process_utterance_publishes_unknown_for_unmatched_voice(node):
    node._db.register("Саша", _embedding(40))
    # Issue #2863 — мокать надо embed_audio_ex (его зовёт нода с #2747):
    # мок голого embed_audio не срабатывал, и тест на самом деле гонял
    # ветку «эмбеддинга нет», которая теперь честно помечена inconclusive.
    # 5с речи — фразу реально оценили и не узнали: чистый is_known=false.
    node._db.embed_audio_ex = MagicMock(return_value=_Embed(_embedding(41)))  # ортогональный голос

    node._process_utterance(b"\x00\x00" * 1000)

    assert node._result_pub.messages == [{"is_known": False}]


def test_no_growth_without_active_session(node):
    """Без предшествующей регистрации (нет growth-сессии) обычные реплики
    никогда не растят чужую/случайную галерею."""
    sid = node._db.register("Саша", _embedding(50))
    assert node._growth_session is None

    node._db.embed_audio_ex = MagicMock(return_value=_Embed(_degraded(_embedding(50), 0.1, 51)))
    node._process_utterance(b"\x00\x00" * 1000)

    assert node._db.gallery_size(sid) == 1, "без активной сессии рост не должен случиться"


# ---------------------------------------------------------------------------
# 3. _process_utterance: pending_name-ветка не дублирует публикацию
# ---------------------------------------------------------------------------


def test_pending_name_registration_publishes_exactly_one_known_result(node):
    emb = _embedding(40)
    node._db.embed_audio_ex = MagicMock(return_value=_Embed(emb))
    node._pending_register_name = "Эйджик"

    # Issue #2769 — pending_name идёт через _do_register(duration_sec=...),
    # буфер должен быть >= MIN_REGISTER_AUDIO_DURATION_SEC (3.0s), иначе
    # регистрация отклонится (см. test_pending_name_registration_rejected_
    # when_audio_too_short ниже). 100000 bytes / 16000 Hz / 2 bytes = 3.125s.
    node._process_utterance(b"\x00\x00" * 50000)

    match_msgs = [m for m in node._result_pub.messages if m.get("is_known") is True]
    assert len(match_msgs) == 1, (
        f"ожидалось ровно одно is_known=true сообщение (от _do_register), "
        f"получено {len(match_msgs)}: {node._result_pub.messages!r}"
    )
    assert match_msgs[0]["source"] == "register"
    assert match_msgs[0]["name"] == "Эйджик"
    # pending сброшен, чтобы следующая реплика не перерегистрировалась.
    assert node._pending_register_name is None


def test_long_window_with_short_speech_is_rejected(node):
    """issue #2747 — гейт мерит РЕЧЬ, а не длину окна записи.

    Регресс, который этим чинится: окно 4.6с, в котором человек говорил
    полсекунды, проходило трёхсекундный порог (проверялась длина буфера,
    а не результат VAD) и становилось эталоном профиля. Замер на роботе
    22.09.2026: скоры одного человека против его же свежесозданного
    профиля скакали 0.908 / 0.550 / 0.820 / 0.556, при том что
    сохранённые эталоны внутри одной сессии держались 0.65–0.91 —
    нестабилен был не эмбеддер, а то, что в него попадало.
    """
    emb = _embedding(77)
    # Буфер длинный — 4.6с, как типичная реплика на роботе; речи в нём
    # всего 0.8с. Раньше решала первая цифра, теперь вторая.
    node._db.embed_audio_ex = MagicMock(return_value=_Embed(emb, voiced_sec=0.8))
    node._pending_register_name = "Эйджик"

    node._process_utterance(bytes(2 * 36800))  # окно 4.6с окна

    assert node._db.list_speakers() == [], (
        'окно длинное, но речи в нём меньше порога — профиль создавать нельзя'
    )
    error_acks = [m for m in node._result_pub.messages if m.get("event") == "register_error"]
    assert len(error_acks) == 1
    assert error_acks[0]["error"] == "too_short"


def test_pending_name_registration_rejected_when_audio_too_short(node):
    """Issue #2769 — end-to-end путь ``_process_utterance`` (не только
    ``_do_register`` напрямую): реплика короче 3.0с в pending_name-ветке
    не создаёт профиль и не публикует is_known=true."""
    emb = _embedding(41)
    node._db.embed_audio_ex = MagicMock(return_value=_Embed(emb, voiced_sec=0.0625))
    node._pending_register_name = "Эйджик"

    # 2000 bytes / 16000 Hz / 2 bytes = 0.0625s — заведомо короче порога.
    node._process_utterance(b"\x00\x00" * 1000)

    assert node._db.list_speakers() == [], "короткая реплика не должна создать профиль"
    assert [m for m in node._result_pub.messages if m.get("is_known") is True] == []
    error_acks = [m for m in node._result_pub.messages if m.get("event") == "register_error"]
    assert len(error_acks) == 1
    assert error_acks[0]["error"] == "too_short"
    assert error_acks[0]["name"] == "Эйджик"
    # pending сброшен ДАЖЕ на отказе — иначе следующая обычная реплика
    # (без вызова register_speaker) тоже попыталась бы зарегистрироваться.
    assert node._pending_register_name is None


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
