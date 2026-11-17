"""Слот одежды на треке: память строки, не детектор (ADR-0148)."""

from rob_box_perception.tracking.tracker import Detection, Tracker, TrackerParams

DT = 0.1


def test_faceless_frame_keeps_clothing_new_track_is_empty():
    """Трек без лица хранит ранее записанную строку; новый трек — пустую."""
    tracker = Tracker(TrackerParams(confirm_hits=1))
    born = tracker.step(0.0, [
        Detection('person', 1.0, 1.0),
        Detection('face', 1.05, 1.0),
    ])
    (track,) = born.tracks
    assert track.clothing == ''
    assert tracker.remember_clothing(track.track_id, 'синяя куртка') is True

    turned = tracker.step(DT, [Detection('person', 1.05, 1.0)])
    (still,) = turned.tracks
    assert still.track_id == track.track_id
    assert still.clothing == 'синяя куртка'

    tracker.step(DT, [Detection('person', 4.0, 1.0)])
    other = next(s for s in tracker.tracks() if s.track_id != track.track_id)
    assert other.clothing == ''
    assert next(s for s in tracker.tracks()
                if s.track_id == track.track_id).clothing == 'синяя куртка'
    assert tracker.remember_clothing(track.track_id, '   ') is False
    assert next(s for s in tracker.tracks()
                if s.track_id == track.track_id).clothing == 'синяя куртка'
