"""Кличка лица при рождении записи (ADR-0164 §4.1, этап 6). Без сети: vision-LLM — подставная функция.

Хранилище — настоящий ``FaceStore`` на отдельной папке ``tmp_path`` (боевую ``/data/faces`` не трогаем).
Фоновый поток заменён синхронным ``runner``, чтобы тест видел результат сразу.
"""

from __future__ import annotations

import importlib
import io
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import pytest

_PKG_ROOT = Path(__file__).resolve().parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

fs = importlib.import_module('rob_box_perception.face_store')
fe = importlib.import_module('rob_box_perception.face_epithet')
vl = importlib.import_module('rob_box_perception.vision_llm')
persona = importlib.import_module('rob_box_core.persona')

JPEG = b'\xff\xd8fake-jpeg'
LANDMARKS = [0.4, 0.4, 0.6, 0.4, 0.5, 0.5, 0.42, 0.6, 0.58, 0.6]
GOOD_ANSWER = json.dumps(
    {'appearance': 'Борода, синий свитер, очки', 'speech': '', 'epithet': 'Бородач в синем свитере'},
    ensure_ascii=False,
)


def _vec(i: int) -> np.ndarray:
    v = np.zeros(8, dtype=np.float32)
    v[i % 8] = 1.0
    return v


class Describer:
    def __init__(self, answer: Any = GOOD_ANSWER) -> None:
        self.answer = answer
        self.calls: List[Dict[str, Any]] = []

    def __call__(self, jpeg: bytes, prompt: str, schema: Dict[str, Any]) -> Any:
        self.calls.append({'jpeg': jpeg, 'prompt': prompt, 'schema': schema})
        if isinstance(self.answer, BaseException):
            raise self.answer
        return self.answer


class Log:
    def __init__(self) -> None:
        self.lines: List[tuple] = []

    def __call__(self, level: str, msg: str) -> None:
        self.lines.append((level, msg))

    def text(self) -> str:
        return '\n'.join(m for _, m in self.lines)


def _store(tmp_path, mode=fs.MODE_WORKSHOP):
    return fs.FaceStore(root=str(tmp_path), mode=mode)


def _namer(store, describer=None, **kw):
    log = kw.pop('log', Log())
    namer = fe.FaceEpithetNamer(
        store, describer=describer, runner=lambda fn: fn(), clock=lambda: 1000.0,
        session_id='s1', log_fn=log, **kw,
    )
    return namer, log


def _new_face(store, namer, i=0, *, snapshot=JPEG, landmarks=LANDMARKS, face_px=120.0):
    match = store.record_encounter(_vec(i), snapshot=snapshot)
    assert match.is_new
    label = namer.on_new_face(match.person_id, snapshot=snapshot, landmarks=landmarks, face_px=face_px)
    return match.person_id, label


def _ep(store, pid):
    return {p['person_id']: p['epithet'] for p in store.people()}.get(pid)


def _meta(tmp_path, pid) -> Dict[str, Any]:
    return json.loads((Path(tmp_path) / pid / 'meta.json').read_text(encoding='utf-8'))


# ── FaceStore: поля и режимы ─────────────────────────────────────────────────


@pytest.mark.parametrize('mode, policy', [
    (fs.MODE_WORKSHOP, fs.EPITHET_POLICY_VISION),
    (fs.MODE_EXHIBITION, fs.EPITHET_POLICY_DICTIONARY),
    (fs.MODE_STRICT, fs.EPITHET_POLICY_NONE),
])
def test_policy_is_decided_by_store_mode(tmp_path, mode, policy):
    assert _store(tmp_path, mode).epithet_policy == policy


def test_set_epithet_persists_history_and_survives_restart(tmp_path):
    store = _store(tmp_path)
    pid = store.record_encounter(_vec(0)).person_id
    assert store.set_epithet(pid, 'Странник', 'first_seen')
    assert store.set_epithet(pid, 'Бородач в синем', 'persona', persona_meta={'appearance': 'борода'})
    meta = _meta(tmp_path, pid)
    assert meta['epithet'] == 'Бородач в синем'
    assert [h['reason'] for h in meta['epithet_history']] == ['first_seen', 'persona']
    assert meta['epithet_history'][1]['old'] == 'Странник'
    assert meta['persona'] == {'appearance': 'борода'}

    again = _store(tmp_path)
    assert _ep(again, pid) == 'Бородач в синем'
    assert again.identify(_vec(0)).epithet == 'Бородач в синем'
    assert [p['epithet'] for p in again.people()] == ['Бородач в синем']


def test_old_meta_without_epithet_fields_loads(tmp_path):
    store = _store(tmp_path)
    pid = store.record_encounter(_vec(0)).person_id
    path = Path(tmp_path) / pid / 'meta.json'
    meta = json.loads(path.read_text(encoding='utf-8'))
    for key in ('epithet', 'epithet_history', 'persona'):
        meta.pop(key)
    path.write_text(json.dumps(meta), encoding='utf-8')
    again = _store(tmp_path)
    assert _ep(again, pid) is None
    assert again.set_epithet(pid, 'Странник', 'first_seen')


def test_reason_none_updates_description_only(tmp_path):
    store = _store(tmp_path)
    pid = store.record_encounter(_vec(0)).person_id
    store.set_epithet(pid, 'Странник', 'first_seen')
    store.set_epithet(pid, 'Странник', None, persona_meta={'appearance': 'очки'})
    meta = _meta(tmp_path, pid)
    assert len(meta['epithet_history']) == 1 and meta['persona'] == {'appearance': 'очки'}


def test_forgotten_record_is_not_resurrected(tmp_path):
    store = _store(tmp_path)
    pid = store.record_encounter(_vec(0)).person_id
    store.forget(pid)
    assert not store.set_epithet(pid, 'Странник', 'first_seen')
    assert not (Path(tmp_path) / pid).exists()


def test_merge_keeps_epithet_of_dropped_record(tmp_path):
    store = _store(tmp_path)
    a = store.record_encounter(_vec(0)).person_id
    b = store.record_encounter(_vec(1)).person_id
    store.set_epithet(b, 'Электрик', 'first_seen')
    assert store.merge(a, b)
    assert _ep(store, a) == 'Электрик'


# ── Рождение: словарь сразу, внешность в фоне ────────────────────────────────


def test_workshop_new_face_gets_dictionary_then_appearance_epithet(tmp_path):
    store, describer = _store(tmp_path), Describer()
    namer, log = _namer(store, describer)
    pid, first = _new_face(store, namer)

    assert first and len(first.split()) == 1  # сразу — словарная
    assert len(describer.calls) == 1 and describer.calls[0]['jpeg'] == JPEG
    assert describer.calls[0]['schema'] is persona.PROPOSAL_SCHEMA
    meta = _meta(tmp_path, pid)
    assert meta['epithet'] == 'Бородач в синем свитере'
    assert [h['reason'] for h in meta['epithet_history']] == ['first_seen', 'persona']
    assert meta['persona']['appearance'] == 'Борода, синий свитер, очки'
    assert meta['persona']['behavior'] == 'Встречен впервые.'
    assert f'[лицо] кличка {pid[:8]}: «{first}» → «Бородач в синем свитере»' in log.text()
    assert namer.stats()['epithet_vision_accepted'] == 1


def test_without_cloud_dictionary_epithet_and_honest_reason(tmp_path):
    store = _store(tmp_path)
    namer, log = _namer(store, describer=None)
    pid, label = _new_face(store, namer)
    assert _ep(store, pid) == label
    assert 'unconfigured' in log.text()
    assert namer.stats()['epithet_skip_unconfigured'] == 1


def test_exhibition_never_sends_snapshot_to_cloud(tmp_path):
    store, describer = _store(tmp_path, fs.MODE_EXHIBITION), Describer()
    namer, _ = _namer(store, describer)
    pid, label = _new_face(store, namer)
    assert describer.calls == []
    assert label and _ep(store, pid) == label  # словарная, в памяти
    assert namer.stats()['epithet_skip_mode'] == 1


def test_strict_gives_no_epithet(tmp_path):
    store, describer = _store(tmp_path, fs.MODE_STRICT), Describer()
    namer, _ = _namer(store, describer)
    match = store.record_encounter(_vec(0), snapshot=JPEG)
    assert namer.on_new_face(match.person_id, snapshot=JPEG, landmarks=LANDMARKS, face_px=120.0) is None
    assert describer.calls == []


@pytest.mark.parametrize('kw', [
    {'landmarks': None},           # AprilTag / не лицо — точек нет
    {'landmarks': [0.5] * 4},      # неполные точки
    {'snapshot': None},            # пустой кадр
    {'face_px': 20.0},             # слишком мелко
])
def test_quality_filter_blocks_vision_call(tmp_path, kw):
    store, describer = _store(tmp_path), Describer()
    namer, _ = _namer(store, describer)
    pid, label = _new_face(store, namer, **kw)
    assert describer.calls == []
    assert label and _ep(store, pid) == label
    assert namer.stats()['epithet_skip_quality'] == 1


def test_hourly_ceiling_limits_vision_calls(tmp_path):
    store, describer = _store(tmp_path), Describer()
    namer, _ = _namer(store, describer, budget=persona.CallBudget(per_window=1))
    _new_face(store, namer, 0)
    pid2, label2 = _new_face(store, namer, 1)
    assert len(describer.calls) == 1
    assert _ep(store, pid2) == label2
    assert namer.stats()['epithet_skip_budget'] == 1


def test_cloud_error_keeps_dictionary_epithet(tmp_path):
    store = _store(tmp_path)
    namer, log = _namer(store, Describer(TimeoutError('timed out')))
    pid, label = _new_face(store, namer)
    assert _ep(store, pid) == label
    assert any(lvl == 'warn' and 'vision-LLM не ответила' in m for lvl, m in log.lines)
    assert namer.stats()['epithet_vision_errors'] == 1


def test_rejected_candidate_keeps_dictionary_but_saves_description(tmp_path):
    answer = json.dumps({'appearance': 'Куртка', 'speech': '', 'epithet': 'Незнакомец в куртке'},
                        ensure_ascii=False)
    store = _store(tmp_path)
    namer, log = _namer(store, Describer(answer))
    pid, label = _new_face(store, namer)
    meta = _meta(tmp_path, pid)
    assert meta['epithet'] == label
    assert meta['persona']['appearance'] == 'Куртка'
    assert 'stranger' in log.text()
    assert namer.stats()['epithet_vision_rejected'] == 1


def test_made_up_speech_is_rejected(tmp_path):
    answer = json.dumps({'appearance': 'Очки', 'speech': 'любит футбол', 'epithet': 'Очкарик-футболист в кепке'},
                        ensure_ascii=False)
    store = _store(tmp_path)
    namer, _ = _namer(store, Describer(answer))
    pid, label = _new_face(store, namer)
    assert _ep(store, pid) == label


def test_voice_epithets_count_as_taken(tmp_path):
    store = _store(tmp_path)
    namer, log = _namer(store, Describer(), foreign_taken=lambda: ['бородач в синем свитере'])
    pid, label = _new_face(store, namer)
    assert _ep(store, pid) == label
    assert 'taken' in log.text()


def test_two_faces_get_different_epithets(tmp_path):
    store, describer = _store(tmp_path), Describer()
    namer, _ = _namer(store, describer)
    a, _ = _new_face(store, namer, 0)
    b, _ = _new_face(store, namer, 1)
    assert _ep(store, a) == 'Бородач в синем свитере'
    assert _ep(store, b) != _ep(store, a)


# ── Конвейер: маркер Встречи несёт кличку ────────────────────────────────────


def test_recognizer_puts_epithet_into_encounter_marker(tmp_path):
    fr = importlib.import_module('rob_box_perception.face_recognition')
    ft = importlib.import_module('rob_box_perception.face_tracker')
    store = _store(tmp_path)
    namer, _ = _namer(store, describer=None)

    class Emb:
        def embed(self, crops):
            return [np.array([1.0, 0.0, 0.0], dtype=np.float32) for _ in crops]

    rng = np.random.RandomState(20260922)
    frame = rng.randint(0, 256, size=(200, 200, 3)).astype(np.uint8)
    rec = fr.FaceRecognizer(
        embedder=Emb(), store=store, namer=namer,
        tracker=ft.FaceTracker(min_track_sec=1.0, min_face_px=48.0, max_gap_sec=5.0),
    )
    frontal = ((38.2946, 51.6963), (73.5318, 51.5014), (56.0252, 71.7366), (41.5493, 92.3655), (70.7299, 92.2041))
    lms = [c for x, y in frontal for c in (0.25 + 0.5 * x / 112.0, 0.25 + 0.5 * y / 112.0)]
    out: List[Dict[str, Any]] = []
    for t in (0.0, 0.5, 1.0):
        det = {'bbox_cx': 0.5, 'bbox_cy': 0.5, 'bbox_w': 0.5, 'bbox_h': 0.5, 'confidence': 0.9,
               'source_camera': 'main_camera', 'landmarks': lms}
        out = rec.process([det], frame, now=t)

    markers = [json.loads(d['attributes_json']) for d in out if 'attributes_json' in d]
    assert len(markers) == 1
    pid = markers[0]['person_id']
    assert markers[0]['epithet'] and markers[0]['epithet'] == _ep(store, pid)
    assert markers[0]['name'] == ''  # кличка не подменяет имя
    assert rec.stats()['epithet']['epithet_dictionary_total'] == 1


def test_namer_error_does_not_break_encounter(tmp_path):
    store = _store(tmp_path)
    namer, log = _namer(store)
    match = store.record_encounter(_vec(0))

    class Enc:
        best = []
        max_face_px = 'не число'

    assert namer.on_encounter(match, Enc(), JPEG) == ''
    assert any(lvl == 'error' for lvl, _ in log.lines)


def test_known_face_marker_gets_its_epithet(tmp_path):
    store = _store(tmp_path)
    namer, _ = _namer(store)
    pid = store.record_encounter(_vec(0)).person_id
    store.set_epithet(pid, 'Странник', 'first_seen')
    known = store.record_encounter(_vec(0))
    assert not known.is_new
    assert namer.on_encounter(known, None, None) == 'Странник'
    assert fe.NO_NAMER.on_encounter(known, None, None) == 'Странник'


def test_recognizer_without_namer_keeps_old_marker_shape(tmp_path):
    fr = importlib.import_module('rob_box_perception.face_recognition')
    rec = fr.FaceRecognizer(embedder=None, store=_store(tmp_path))
    assert rec.stats()['epithet'] == {}


# ── Клиент vision-LLM: без сети ──────────────────────────────────────────────


class _Resp(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


def test_describer_from_env_without_key_is_dictionary_only_with_honest_reason():
    for env in ({}, {vl.ENV_PROVIDER: 'deepseek'}, {'DEEPSEEK_API_KEY': 'k', vl.ENV_PROVIDER: 'minimax'}):
        describer, note = vl.describer_from_env(env)
        assert 'словарные клички' in note
    assert 'MINIMAX_API_KEY' in vl.describer_from_env({})[1]


def test_unknown_provider_is_reported():
    describer, note = vl.describer_from_env({vl.ENV_PROVIDER: 'foo', vl.ENV_API_KEY: 'k'})
    assert describer is None and 'foo' in note


def _capture(env):
    seen = {}

    def opener(request, timeout):
        seen['url'] = request.full_url
        seen['auth'] = request.get_header('Authorization')
        seen['body'] = json.loads(request.data.decode('utf-8'))
        return _Resp(json.dumps({'choices': [{'message': {'content': GOOD_ANSWER}}]}).encode('utf-8'))

    describer, note = vl.describer_from_env(env, opener=opener)
    assert describer is not None, note
    describer(JPEG, 'опиши', persona.PROPOSAL_SCHEMA)
    return seen, note


def test_default_provider_is_minimax_m3_with_minimax_key():
    seen, note = _capture({'MINIMAX_API_KEY': 'mm-secret'})
    assert seen['url'] == 'https://api.minimax.io/v1/chat/completions'
    assert seen['auth'] == 'Bearer mm-secret'
    assert seen['body']['model'] == 'MiniMax-M3'
    assert seen['body']['messages'][0]['content'][1]['image_url']['url'].startswith('data:image/jpeg;base64,')
    assert 'mm-secret' not in note


def test_deepseek_preset_uses_deepseek_key_and_vision_model():
    seen, note = _capture({vl.ENV_PROVIDER: 'deepseek', 'DEEPSEEK_API_KEY': 'ds-secret',
                           'MINIMAX_API_KEY': 'mm-secret'})
    assert seen['url'] == 'https://api.deepseek.com/chat/completions'
    assert seen['auth'] == 'Bearer ds-secret'
    assert seen['body']['model'] == 'deepseek-v4-flash-vision-exp'
    assert 'ds-secret' not in note


def test_deepseek_model_is_overridable_by_env():
    seen, _ = _capture({vl.ENV_PROVIDER: 'deepseek', 'DEEPSEEK_API_KEY': 'k', vl.ENV_MODEL: 'deepseek-flash'})
    assert seen['body']['model'] == 'deepseek-flash'


def test_explicit_vlm_env_overrides_preset():
    seen, _ = _capture({vl.ENV_API_KEY: 'x', vl.ENV_BASE_URL: 'https://example.test/v1/',
                        vl.ENV_MODEL: 'm', 'MINIMAX_API_KEY': 'mm'})
    assert seen['url'] == 'https://example.test/v1/chat/completions'
    assert seen['auth'] == 'Bearer x' and seen['body']['model'] == 'm'


# ── Новая Встреча известного лица: один вызов на Встречу ─────────────────────


class _Enc:
    class _Obs:
        landmarks = LANDMARKS

    best = [_Obs()]
    max_face_px = 120.0


def _meeting(namer, store, i=0):
    match = store.record_encounter(_vec(i), snapshot=JPEG)
    return match, namer.on_encounter(match, _Enc(), JPEG)


def test_new_meeting_of_known_face_calls_model_once_per_meeting(tmp_path):
    store, describer = _store(tmp_path), Describer()
    namer, _ = _namer(store, describer)
    pid, _ = _new_face(store, namer)
    assert len(describer.calls) == 1  # рождение

    match, label = _meeting(namer, store)  # новая Встреча известного
    assert not match.is_new and match.person_id == pid
    assert len(describer.calls) == 2
    assert label == 'Бородач в синем свитере'  # принятая кличка держится

    _meeting(namer, store)  # ещё одна Встреча — ещё один вызов
    assert len(describer.calls) == 3


def test_recognition_inside_same_meeting_does_not_call_model(tmp_path):
    """Кадры и узнавания внутри одной Встречи моделью не обслуживаются: зовёт только on_encounter."""
    store, describer = _store(tmp_path), Describer()
    namer, _ = _namer(store, describer)
    _new_face(store, namer)
    before = len(describer.calls)
    for _ in range(5):
        assert store.identify(_vec(0)).person_id  # узнавание без события Встречи
    assert len(describer.calls) == before


def test_known_meeting_without_key_stays_dictionary(tmp_path):
    store = _store(tmp_path)
    namer, _ = _namer(store, describer=None)
    pid, first = _new_face(store, namer)
    match, label = _meeting(namer, store)
    assert label == first == _ep(store, pid)
    assert namer.stats()['epithet_skip_unconfigured'] >= 2


def test_known_meeting_exhibition_and_bad_snapshot_do_not_call_model(tmp_path):
    describer = Describer()
    store = _store(tmp_path, fs.MODE_EXHIBITION)
    namer, _ = _namer(store, describer)
    match = store.record_encounter(_vec(0), snapshot=JPEG)
    namer.on_encounter(match, _Enc(), JPEG)
    assert describer.calls == []

    store2 = _store(tmp_path / 'w')
    namer2, _ = _namer(store2, describer)
    pid = store2.record_encounter(_vec(0)).person_id
    store2.set_epithet(pid, 'Странник', 'first_seen')
    known = store2.record_encounter(_vec(0))
    assert namer2.on_known_meeting(known.person_id, snapshot=None, landmarks=LANDMARKS, face_px=120.0) is False
    assert describer.calls == []


def test_known_meeting_rejected_candidate_keeps_previous_epithet(tmp_path):
    store, describer = _store(tmp_path), Describer()
    namer, _ = _namer(store, describer)
    pid, _ = _new_face(store, namer)
    describer.answer = json.dumps({'appearance': 'Другая одежда', 'speech': '', 'epithet': ''},
                                  ensure_ascii=False)
    _meeting(namer, store)
    assert _ep(store, pid) == 'Бородач в синем свитере'
    assert _meta(tmp_path, pid)['persona']['appearance'] == 'Другая одежда'


@pytest.mark.parametrize('raw, expected', [
    ('', persona.VISION_CALLS_PER_HOUR), ('5', 5), ('мусор', persona.VISION_CALLS_PER_HOUR), ('-3', 0),
])
def test_budget_from_env(raw, expected):
    assert vl.budget_from_env({vl.ENV_MAX_PER_HOUR: raw}).per_window == expected
