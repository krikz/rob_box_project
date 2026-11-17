"""Issue #4115 -- ``SpeakerDatabase.register``: профиль виден читателям только вместе с эмбеддингом.

Корень флака #4093. ``register`` вставлял строку профиля и строку его эмбеддинга на ОБЩЕМ для потоков соединении
(``check_same_thread=False``): соединение видит свои неподтверждённые записи, поэтому читатель из другого потока в окне
между двумя ``INSERT`` находил профиль с 0 эмбеддингов, и ``identify`` возвращал ``None``. Теперь транзакция записи
идёт на своём соединении (``_isolated_write``): читатели общего соединения видят профиль целиком после ``commit`` или
не видят вовсе.

Детерминированно, без ``sleep``: запись останавливается на событии внутри окна (замедленная сериализация эмбеддинга --
она вызывается уже после вставки профиля), читатель смотрит БД именно в этот момент, затем запись отпускается.
"""

from __future__ import annotations

import importlib.util
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

# Грузим модуль по пути, минуя пакет `rob_box_voice`, чей `__init__` тянет pyaudio -- как в соседних тестах каталога.
_SPEAKER_EMB = Path(__file__).resolve().parents[3] / "rob_box_voice" / "utils" / "speaker_embeddings.py"
_spec = importlib.util.spec_from_file_location("rob_box_voice.utils.speaker_embeddings", _SPEAKER_EMB)
se = importlib.util.module_from_spec(_spec)
sys.modules["rob_box_voice.utils.speaker_embeddings"] = se
_spec.loader.exec_module(se)

SpeakerDatabase = se.SpeakerDatabase
WAIT_S = 10.0  # потолок ожидания события: тест не виснет, если запись не дошла до окна


def _embedding(seed: int) -> np.ndarray:
    v = np.random.default_rng(seed).standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


@pytest.fixture()
def db(tmp_path):
    database = SpeakerDatabase(str(tmp_path / "speakers.db"))
    yield database
    database.close()


class _Window:
    """Окно записи: ``_ndarray_to_blob`` встаёт на ``entered`` и ждёт ``release`` -- читатель смотрит БД между ними."""

    def __init__(self, database) -> None:
        self.entered, self.release = threading.Event(), threading.Event()
        self._real = database._ndarray_to_blob
        database._ndarray_to_blob = self._blob

    def _blob(self, arr):
        self.entered.set()
        assert self.release.wait(WAIT_S), "читатель не отпустил запись"
        return self._real(arr)


def _in_thread(fn, *args, **kwargs):
    """Запустить ``fn`` в потоке: ``(поток, словарь с результатом или ошибкой)``."""
    out = {}

    def run():
        try:
            out["value"] = fn(*args, **kwargs)
        except BaseException as exc:  # noqa: BLE001 -- тест докладывает ошибку записи, а не глотает
            out["error"] = exc

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread, out


def _snapshot(database, embedding):
    """Что видит читатель: ``(профили (имя, число эмбеддингов), найден ли голос identify)``."""
    return [(s["name"], s["embeddings"]) for s in database.list_speakers()], database.identify(embedding, 0.0)


def _finish(window, thread, out):
    window.release.set()
    thread.join(WAIT_S)
    assert not thread.is_alive() and "error" not in out, out.get("error")


def test_reader_never_sees_a_new_profile_without_its_embedding(db):
    """Окно между вставкой профиля и вставкой его эмбеддинга: читатель другого потока не видит профиль (и ``identify``
    не находит голос); после записи -- профиль с одним эмбеддингом, ``identify`` находит."""
    emb, window = _embedding(1), _Window(db)
    writer, out = _in_thread(db.register, "Борис", emb)
    assert window.entered.wait(WAIT_S), "запись не дошла до окна"

    during = _snapshot(db, emb)
    _finish(window, writer, out)

    assert during == ([], None), "в окне записи читатель увидел недописанный профиль"
    after, match = _snapshot(db, emb)
    assert after == [("Борис", 1)] and match is not None and match.name == "Борис"
    assert match.speaker_id == out["value"]


def test_reader_sees_known_profile_unchanged_while_another_embedding_is_written(db):
    """Явный ``speaker_id`` существующего профиля: пока пишется второй эмбеддинг, читатель видит профиль как был (с
    одним), а не наполовину обновлённым; после записи -- с двумя."""
    sid = db.register("Нина", _embedding(2))
    emb, window = _embedding(3), _Window(db)
    writer, out = _in_thread(db.register, "Нина", emb, speaker_id=sid)
    assert window.entered.wait(WAIT_S)

    during = [(s["name"], s["embeddings"]) for s in db.list_speakers()]
    _finish(window, writer, out)

    assert during == [("Нина", 1)]
    assert [(s["name"], s["embeddings"]) for s in db.list_speakers()] == [("Нина", 2)]


def test_rename_by_explicit_id_is_published_together_with_the_embedding(db):
    """Переименование по явному id (rename-поток) идёт той же транзакцией: в окне записи читатель видит старое имя, после
    -- новое и новый эмбеддинг."""
    sid = db.register("Нина", _embedding(4))
    emb, window = _embedding(5), _Window(db)
    writer, out = _in_thread(db.register, "Тамара", emb, speaker_id=sid)
    assert window.entered.wait(WAIT_S)

    during = [(s["name"], s["embeddings"]) for s in db.list_speakers()]
    _finish(window, writer, out)

    assert during == [("Нина", 1)]
    assert [(s["name"], s["embeddings"]) for s in db.list_speakers()] == [("Тамара", 2)]


def test_unnamed_profile_is_published_atomically_too(db):
    """Профиль без имени (``db.unnamed.register``, #3858) -- тот же шаблон «профиль, потом эмбеддинг»: читатель не видит
    безымянный профиль без эмбеддинга."""
    emb, window = _embedding(6), _Window(db)
    writer, out = _in_thread(db.unnamed.register, emb, 4.0)
    assert window.entered.wait(WAIT_S)

    during = (db.unnamed.count(), [(s["name"], s["embeddings"]) for s in db.list_speakers()])
    _finish(window, writer, out)

    assert during == (0, [])
    assert db.unnamed.count() == 1 and [s["embeddings"] for s in db.list_speakers()] == [1]


def test_failed_write_leaves_no_half_profile(db, monkeypatch):
    """Запись упала между профилем и эмбеддингом: транзакция откатывается целиком -- ни профиля без эмбеддинга, ни
    «хвоста», который закоммитил бы чужой ``commit`` на общем соединении."""
    def broken(_arr):
        raise RuntimeError("диск")

    monkeypatch.setattr(db, "_ndarray_to_blob", broken)
    with pytest.raises(RuntimeError):
        db.register("Борис", _embedding(7))
    db._conn.commit()  # чужой commit на общем соединении не должен вытащить недописанное

    assert db.list_speakers() == []
    monkeypatch.undo()
    assert db.register("Борис", _embedding(7)) and [(s["name"], s["embeddings"]) for s in db.list_speakers()] == [
        ("Борис", 1)
    ]


def test_claimed_unnamed_profile_gets_name_and_embedding(db):
    """``register_or_merge`` присваивает имя безымянному профилю (``claim``): обновление имени и запись эмбеддинга не
    блокируют друг друга (два соединения), итог -- один профиль, два эмбеддинга."""
    base = _embedding(8)
    sid = db.unnamed.register(base, 4.0).speaker_id

    outcome = db.register_or_merge("Олег", base, duration_sec=4.0)

    assert outcome.claimed_unnamed and outcome.speaker_id == sid
    assert [(s["name"], s["embeddings"]) for s in db.list_speakers()] == [("Олег", 2)]
