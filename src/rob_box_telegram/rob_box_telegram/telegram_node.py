#!/usr/bin/env python3
import asyncio, json, logging, os, re, threading, time, uuid
from typing import Optional
import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from telegram.ext import Application, CallbackQueryHandler, CommandHandler, MessageHandler, filters
from .camera_cache import CameraCache
from .handlers import commands as _cmds
from .handlers.callbacks import callback_handler
from .handlers.messages import text_message_handler, voice_message_handler
# Issue #1160 — Prometheus metrics (этап 1 observability). Telegram-bot —
# отдельный контейнер, поэтому у него свой лёгкий observability-модуль
# (не тянет rob_box_voice).
from .observability import (
    is_metrics_enabled,
    record_telegram_message,
    start_metrics_server,
)
_BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
_RE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
_TL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
class TelegramNode(Node):
    """Telegram Bot API ↔ /voice/stt/result + /voice/dialogue/response bridge."""
    def __init__(self):
        super().__init__("telegram_node")
        self.declare_parameter("camera_topic", "/camera/camera/color/image_raw/compressed")
        self.declare_parameter("camera_depth_topic", "/camera/camera/depth/image_rect_raw/compressedDepth")
        self.declare_parameter("camera_up_topic", "/ceiling_camera/image_raw/compressed")
        self.declare_parameter("camera_cache_ttl", 5.0)
        # Issue #1160 — Prometheus metrics endpoint. 9101 — telegram-bot.
        self.declare_parameter("metrics_port", 9101)
        p = self.get_parameter
        self.camera_topic, self.camera_depth_topic, self.camera_up_topic = p("camera_topic").value, p("camera_depth_topic").value, p("camera_up_topic").value
        self.camera_cache = CameraCache(ttl=p("camera_cache_ttl").value)
        self.latest_map_grid = self._active_chat_id = self._telegram_app = None
        # Issue #1195 — echo path (LLM replies back into the chat).
        # ``_telegram_loop`` is the asyncio loop owned by the telegram
        # thread; ``_response_queue`` is consumed by ``_chat_echo_worker``
        # inside that loop. The ROS executor thread only does
        # ``call_soon_threadsafe`` — no ``run_coroutine_threadsafe``
        # (t_aad8e224: loop closed/None → swallowed AttributeError).
        self._telegram_loop: Optional[asyncio.AbstractEventLoop] = None
        self._response_queue: Optional[asyncio.Queue] = None
        self._echo_task: Optional[asyncio.Task] = None
        g = ReentrantCallbackGroup()
        for topic, cb in ((self.camera_topic, self._on_camera_front), (self.camera_depth_topic, self._on_camera_depth), (self.camera_up_topic, self._on_camera_up)):
            self.create_subscription(CompressedImage, topic, cb, _BE, callback_group=g)
        self.create_subscription(OccupancyGrid, "/rtabmap/grid_prob_map", self._on_map, _TL, callback_group=g)
        self._stt_pub = self.create_publisher(String, "/voice/stt/result", _RE)
        self._response_pub = self.create_publisher(String, "/voice/dialogue/response", _RE)
        # Issue #1195 — restore the echo path: dialogue/TTS output is
        # duplicated into the active Telegram chat so the operator sees
        # what the robot says (removed in 88cecc91 because of the
        # asyncio-loop bug; reimplemented queue-based, see _on_response).
        self._response_sub = self.create_subscription(
            String, "/voice/dialogue/response", self._on_response, _RE,
            callback_group=g,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_web", _RE)
        self.tts_pub = self.create_publisher(String, "/voice/tts/request", _RE)
        # Issue #1160 — Prometheus metrics server (этап 1).
        # Порт 9101 — стандартный для telegram-bot (см. observability).
        metrics_port = int(p("metrics_port").value or 0)
        if metrics_port > 0 and is_metrics_enabled():
            if start_metrics_server(metrics_port):
                self.get_logger().info(
                    f"📊 Telegram metrics server listening on :{metrics_port}/metrics"
                )
            else:
                self.get_logger().warning(
                    f"📊 Telegram metrics port {metrics_port} not bound "
                    "(busy or prometheus_client missing)"
                )
        token = os.getenv("TELEGRAM_BOT_TOKEN", "")
        if not token: self.get_logger().error("TELEGRAM_BOT_TOKEN not set"); return
        self._start_telegram_bot(token)
        self.get_logger().info("TelegramNode: thin ROS 2 bridge (W8)")
    def _on_camera_front(self, m): self.camera_cache.update(self.camera_topic, bytes(m.data))
    def _on_camera_depth(self, m): self.camera_cache.update(self.camera_depth_topic, bytes(m.data))
    def _on_camera_up(self, m): self.camera_cache.update(self.camera_up_topic, bytes(m.data))
    def _on_map(self, m): self.latest_map_grid = m
    def set_active_chat(self, chat_id: int) -> None: self._active_chat_id = chat_id
    def forward_to_stt(self, text: str, chat_id: Optional[int] = None) -> None:
        if not text: return
        # Issue #1195 — source marker: [TG:chat_id] text. dialogue_node
        # parses it to skip the wake-word gate (chat messages are explicit
        # address), remembers the chat for echo routing and does NOT attach
        # the voice-biometry speaker tag. chat_id also becomes the "active
        # chat" so voice-initiated replies echo into the same chat.
        if chat_id is not None:
            self.set_active_chat(chat_id)
            text = f"[TG:{chat_id}] {text}"
        # Issue #1160 — Prometheus metrics: входящее сообщение (текст/команда).
        if is_metrics_enabled():
            record_telegram_message("in", message_type="text")
        m = String(); m.data = text; self._stt_pub.publish(m)
    def publish_tts(self, text: str) -> None:
        # Issue #1160 — Prometheus metrics: исходящая озвучка.
        if is_metrics_enabled():
            record_telegram_message("out", message_type="voice")
        m = String(); m.data = json.dumps(
            {"ssml": f"<speak>{text}</speak>", "speech_id": str(uuid.uuid4()), "emotion": "neutral"},
            ensure_ascii=False); self._response_pub.publish(m)
    def _on_response(self, msg: String) -> None:
        """Echo dialogue/TTS output back into the active Telegram chat.

        Issue #1195 — runs on the ROS 2 executor thread, so the actual
        ``bot.send_message`` must happen on the telegram asyncio loop.
        We push ``(chat_id, text)`` into ``_response_queue``; the telegram
        loop's ``_chat_echo_worker`` consumes it and sends. No
        ``run_coroutine_threadsafe`` from this thread — that was the
        t_aad8e224 bug (loop captured wrong / closed → AttributeError
        silently swallowed every TG reply).
        """
        try:
            payload = json.loads(msg.data or "")
        except (json.JSONDecodeError, TypeError):
            payload = {}
        ssml = payload.get("ssml")
        if ssml:
            text = re.sub(r"<[^>]+>", "", ssml).strip()
        else:
            text = (msg.data or "").strip()
        if not text:
            return
        # Prefer the chat_id dialogue_node routed for this turn; fall back
        # to the active chat (last chat that wrote to the robot).
        chat_id = payload.get("tg_chat_id") or self._active_chat_id
        loop, queue = self._telegram_loop, self._response_queue
        if not (loop and queue and chat_id):
            self.get_logger().warning(
                "Dropping dialogue echo, bot not ready / no active chat "
                f"(app={bool(self._telegram_app)}, loop={bool(loop)}, "
                f"chat_id={chat_id!r}, text_len={len(text)})"
            )
            return
        try:
            loop.call_soon_threadsafe(queue.put_nowait, (int(chat_id), text))
        except RuntimeError as exc:
            # Loop closed underneath us (bot crashed, outer loop is between
            # attempts). Drop rather than crash the ROS executor — the next
            # valid response will be picked up after the loop is restored.
            self.get_logger().error(
                f"Failed to schedule TG echo (loop closed?): {exc!r}"
            )
    async def _chat_echo_worker(self) -> None:
        """Consume dialogue responses and send them into the chat.

        Runs inside the telegram asyncio loop (started by
        ``_run_telegram``); the queue is only fed from the ROS thread via
        ``call_soon_threadsafe``.
        """
        queue = self._response_queue
        if queue is None:
            return
        try:
            while True:
                chat_id, text = await queue.get()
                app = self._telegram_app
                if app is None:
                    continue
                try:
                    await app.bot.send_message(chat_id=chat_id, text=text)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warning(
                        f"Failed to echo LLM reply to chat {chat_id}: {exc!r}"
                    )
        except asyncio.CancelledError:
            pass
    def _start_telegram_bot(self, token: str) -> None:
        threading.Thread(target=self._run_telegram_loop, args=(token,), daemon=True,
                         name="telegram-bot").start()
    def _run_telegram_loop(self, token: str) -> None:
        attempt, delay = 0, 5.0
        while rclpy.ok():
            loop = asyncio.new_event_loop(); asyncio.set_event_loop(loop)
            try: loop.run_until_complete(self._run_telegram(token)); loop.close(); return
            except Exception as e:
                attempt += 1; d = min(delay * attempt, 60.0)
                self.get_logger().error(f"Bot crashed ({attempt}): {e}. Retry in {d:.0f}s")
                loop.close(); time.sleep(d)
    async def _run_telegram(self, token: str) -> None:
        app = Application.builder().token(token).build()
        app.bot_data["node"] = self; self._telegram_app = app
        # Issue #1195 — echo path: capture the loop we run on and create
        # the queue the ROS thread feeds via call_soon_threadsafe. The
        # worker task lives inside this loop, so no cross-loop coroutine
        # scheduling (t_aad8e224).
        self._telegram_loop = asyncio.get_running_loop()
        self._response_queue = asyncio.Queue()
        self._echo_task = asyncio.create_task(self._chat_echo_worker())
        for name in dir(_cmds):
            if name.endswith("_handler"):
                app.add_handler(CommandHandler(name[:-8], getattr(_cmds, name)))
        app.add_handler(CallbackQueryHandler(callback_handler))
        app.add_handler(MessageHandler(filters.VOICE, voice_message_handler))
        app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, text_message_handler))
        await app.initialize(); await app.start()
        try:
            await app.updater.start_polling(poll_interval=1.0, timeout=30)
        except Exception as exc:
            self.get_logger().warning(
                f"start_polling failed: {exc!r}; outer loop will retry"
            )
            raise
        try:
            while rclpy.ok(): await asyncio.sleep(1.0)
        finally:
            if self._echo_task is not None:
                self._echo_task.cancel()
                self._echo_task = None
            await app.updater.stop(); await app.stop(); await app.shutdown()


def main(args=None):
    rclpy.init(args=args); node = TelegramNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__": main()