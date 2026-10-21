"""Issue #3762: длинная речь режется на куски, а не выбрасывается."""
import array
import math

from rob_box_dialog.backlog import Backlog, Heard

from rob_box_voice.core.speech_split import split_long_speech

SR = 16000


def _tone(seconds: float, amp: int = 8000) -> bytes:
    n = int(SR * seconds)
    return array.array("h", (int(amp * math.sin(i / 7.0)) for i in range(n))).tobytes()


def _silence(seconds: float) -> bytes:
    return b"\x00\x00" * int(SR * seconds)


def test_short_speech_is_returned_whole():
    buf = _tone(5.0)
    assert split_long_speech(buf, SR, 15.0, 0.3) == [buf]


def test_20s_speech_is_split_without_loss_and_within_max():
    buf = _tone(20.0)
    chunks = split_long_speech(buf, SR, 15.0, 0.3)
    assert len(chunks) >= 2
    assert b"".join(chunks) == buf
    assert all(len(c) <= 15 * SR * 2 for c in chunks)
    assert all(len(c) >= int(0.3 * SR) * 2 for c in chunks)


def test_cut_goes_into_internal_pause():
    buf = _tone(12.5) + _silence(0.4) + _tone(8.0)
    first = split_long_speech(buf, SR, 15.0, 0.3)[0]
    assert 12.5 * SR * 2 <= len(first) <= 12.9 * SR * 2


def test_tail_shorter_than_min_is_never_produced():
    buf = _tone(15.1)
    chunks = split_long_speech(buf, SR, 15.0, 0.3)
    assert all(len(c) >= int(0.3 * SR) * 2 for c in chunks)
    assert b"".join(chunks) == buf


def test_bare_wake_5s_after_long_speech_picks_up_a_chunk():
    """20 с речи -> куски в бэклоге; «робот» через 5 с подхватывает фразу."""
    chunks = split_long_speech(_tone(20.0), SR, 15.0, 0.3)
    backlog = Backlog()
    for i, _ in enumerate(chunks):
        backlog.add(Heard(text=f"кусок {i}", t=100.0 + i, doa_deg=90.0))
    assert backlog.background_phrases(105.0) >= 1
    entry, channels = backlog.pickup(Heard(text="робот", t=105.0, doa_deg=90.0, faces=frozenset({"f1"})))
    assert entry is not None and channels
