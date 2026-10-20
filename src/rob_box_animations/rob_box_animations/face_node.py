"""face_node — the robot's procedural LED face in real time (ADR-0150 §5.5, PR-3a #3689).

Runs in the ``led-matrix`` container on Vision Pi next to the compositor
(decision В14): frames never cross Zenoh. A 25 Hz timer asks ``FaceDirector``
for a frame and publishes the panels that changed to ``/panel_image``
(``sensor_msgs/Image`` rgb8, ``frame_id`` = logical group), one publisher.

Inputs: ``/voice/speech/envelope`` + ``/voice/speech/event`` (mouth by the real
voice), ``/voice/tts/state`` (transitional «speaking»), ``/audio/direction`` +
``/audio/vad`` (look at the speaker), ``/voice/music/state`` (music scenes by
tempo), ``/voice/animation/request`` (v1 ``play_animation``, until PR-3b),
``/voice/expression/request`` (``ExpressionRequest``; writers come in PR-3b),
``~/scene`` (debug: ``name`` or ``name:seconds``). No voice owner for
``voice_timeout_s`` (no messages, no voice publishers in
the graph) -> ``sleep``; it is back -> ``wakeup`` -> idle.
"""
from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rob_box_dialog_msgs.msg import ExpressionRequest, SpeechEnvelope, SpeechEvent
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String

from .face.face import FPS, GROUP_OF, GROUP_SIZE, IDLE_MA, SCENE_LIT_LIMIT_MA, frame_ma, to_wire
from .face_director import FaceDirector, GazeTrack, parse_animation_request

VOICE_OWNER_TOPIC = '/voice/speech/event'
#: Publishers in the graph that mean «voice-assistant is up» (tts_node, audio_node).
VOICE_PRESENCE_TOPICS = (VOICE_OWNER_TOPIC, '/voice/tts/state', '/audio/direction')


class PanelPublisher:
    """Frame -> one ``Image`` per logical group; only panels that changed (all of them once a second)."""

    def __init__(self, node: Node, full_refresh_s: float = 1.0) -> None:
        self._node = node
        self._pub = node.create_publisher(Image, '/panel_image', 10)
        self._last: dict = {}
        self._full_refresh_s = full_refresh_s
        self._full_at = 0.0
        self.sent = 0

    def publish(self, frame, now: float) -> None:
        full = now - self._full_at >= self._full_refresh_s
        if full:
            self._full_at = now
        stamp = self._node.get_clock().now().to_msg()
        for panel, arr in frame.panels().items():
            group = GROUP_OF[panel]
            data = to_wire(arr).tobytes()
            if not full and self._last.get(group) == data:
                continue
            self._last[group] = data
            w, h = GROUP_SIZE[group]
            msg = Image()
            msg.header.stamp, msg.header.frame_id = stamp, group
            msg.width, msg.height, msg.encoding, msg.is_bigendian, msg.step = w, h, 'rgb8', 0, w * 3
            msg.data = data
            self._pub.publish(msg)
            self.sent += 1


class FaceNode(Node):
    def __init__(self) -> None:
        super().__init__('face_node')
        p = {
            'fps': float(FPS), 'mouth_design': 'jaw', 'thinking_design': 'thinking_teeth',
            'idle_bored_every_s': 60.0, 'voice_timeout_s': 7.0, 'test_pattern': False,
            'doa_front_deg': 0.0, 'doa_ccw': True, 'rear_theme': 'default', 'metrics_period_s': 10.0,
        }
        for name, default in p.items():
            self.declare_parameter(name, default)
        g = {name: self.get_parameter(name).value for name in p}
        self.director = FaceDirector(
            time.time(), mouth_design=g['mouth_design'], thinking_design=g['thinking_design'],
            idle_bored_every_s=g['idle_bored_every_s'], voice_timeout_s=g['voice_timeout_s'],
            rear_theme=g['rear_theme'], test_pattern=g['test_pattern'],
            gaze=GazeTrack(front_deg=g['doa_front_deg'], ccw=g['doa_ccw']))
        self.panels = PanelPublisher(self)
        self._graph_present: Optional[bool] = None
        self._subscribe()
        self._metrics_period = g['metrics_period_s']
        self._reset_metrics(time.time())
        self.create_timer(1.0 / g['fps'], self._on_frame)
        self.create_timer(1.0, self._check_voice_owner)
        self.get_logger().info(
            f"face_node: {g['fps']:.0f} fps, mouth={g['mouth_design']}, thinking={g['thinking_design']}, "
            f"test_pattern={g['test_pattern']}, voice_timeout={g['voice_timeout_s']:.0f}s")

    def _subscribe(self) -> None:
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(SpeechEnvelope, '/voice/speech/envelope', self._on_envelope, 10)
        self.create_subscription(SpeechEvent, VOICE_OWNER_TOPIC, self._on_speech_event, 10)
        self.create_subscription(String, '/voice/tts/state', self._on_tts_state, 10)
        self.create_subscription(Int32, '/audio/direction', self._on_direction, 10)
        self.create_subscription(Bool, '/audio/vad', self._on_vad, 10)
        self.create_subscription(String, '/voice/music/state', self._on_music_state, latched)
        self.create_subscription(String, '/voice/animation/request', self._on_animation_request, 10)
        self.create_subscription(ExpressionRequest, '/voice/expression/request', self._on_expression, 10)
        self.create_subscription(String, '~/scene', self._on_animation_request, 10)

    # -- inputs ---------------------------------------------------------------
    def _on_envelope(self, msg) -> None:
        t0 = msg.t0.sec + msg.t0.nanosec * 1e-9
        self.director.on_envelope(msg.speech_id, t0, msg.frame_ms, msg.levels, msg.vowel_hint, time.time())

    def _on_speech_event(self, msg) -> None:
        now = time.time()
        if msg.kind == SpeechEvent.KIND_CANCELLED:
            self.director.on_speech_cancelled(msg.speech_id, now)
        else:
            self.director.voice_seen(now)

    def _on_tts_state(self, msg) -> None:
        self.director.on_tts_state(msg.data, time.time())

    def _on_direction(self, msg) -> None:
        now = time.time()
        self.director.voice_seen(now)
        self.director.gaze.on_direction(float(msg.data), now)

    def _on_vad(self, msg) -> None:
        self.director.gaze.on_vad(bool(msg.data), time.time())

    def _on_music_state(self, msg) -> None:
        if not self.director.on_music_state(msg.data, time.time()):
            self.get_logger().warn(f'/voice/music/state: не JSON-снимок плеера: {msg.data[:120]!r}')

    def _on_animation_request(self, msg) -> None:
        name, seconds = parse_animation_request(msg.data)
        self._show(name, seconds)

    def _on_expression(self, msg) -> None:
        if msg.face_json:
            self.get_logger().warn('ExpressionRequest.face_json пока не исполняется (DSL сцены — PR-3c)')
        for name in (msg.gesture, msg.emotion):
            if name:
                self._show(name, msg.ttl_ms / 1000.0 if msg.ttl_ms else None)
                return

    def _show(self, name: str, seconds) -> None:
        scene = self.director.request(name, seconds, time.time())
        if scene is None:
            self.get_logger().warn(f'сцены «{name}» нет — запрос не исполнен')
        else:
            self.get_logger().info(f'сцена по запросу: {name} -> {scene} ({seconds or "по умолчанию"} с)')

    def _check_voice_owner(self) -> None:
        counts = {t: self.count_publishers(t) for t in VOICE_PRESENCE_TOPICS}
        present = any(counts.values())
        if present:
            self.director.voice_seen(time.time())
        if present != self._graph_present:
            self._graph_present = present
            self.get_logger().info(f'голосовой стек в графе: {"есть" if present else "нет"} {counts}')

    # -- output ---------------------------------------------------------------
    def _on_frame(self) -> None:
        now = time.time()
        t0 = time.perf_counter()
        prev = self.director.key
        frame = self.director.tick(now)
        self.panels.publish(frame, now)
        ms = (time.perf_counter() - t0) * 1000.0
        if self.director.key != prev:
            self.get_logger().info(f'лицо: {prev or "—"} -> {self.director.key}')
        self._frames += 1
        self._render_ms = max(self._render_ms, ms)
        self._render_sum += ms
        self._peak_ma = max(self._peak_ma, frame_ma(frame))
        if now - self._metrics_at >= self._metrics_period:
            self._log_metrics(now)

    def _reset_metrics(self, now: float) -> None:
        self._metrics_at, self._frames, self._render_ms, self._render_sum, self._peak_ma = now, 0, 0.0, 0.0, 0.0
        self._sent_at = self.panels.sent

    def _log_metrics(self, now: float) -> None:
        dt = now - self._metrics_at
        n = max(1, self._frames)
        level = self.get_logger().warn if self._peak_ma - IDLE_MA > SCENE_LIT_LIMIT_MA else self.get_logger().info
        level(f'face.metrics scene={self.director.key} fps={self._frames / dt:.1f} '
              f'render_ms_avg={self._render_sum / n:.2f} render_ms_max={self._render_ms:.2f} '
              f'panels_per_s={(self.panels.sent - self._sent_at) / dt:.1f} face.peak_ma={self._peak_ma:.0f}')
        self._reset_metrics(now)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
