#!/usr/bin/env python3
import asyncio, json, logging, os, threading, time, uuid
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
        p = self.get_parameter
        self.camera_topic, self.camera_depth_topic, self.camera_up_topic = p("camera_topic").value, p("camera_depth_topic").value, p("camera_up_topic").value
        self.camera_cache = CameraCache(ttl=p("camera_cache_ttl").value)
        self.latest_map_grid = self._active_chat_id = self._telegram_app = None
        # python-telegram-bot's Application runs on its own asyncio loop,
        # which we own via ``_run_telegram_loop``. ``_telegram_app._loop``
        # is private API (see bug t_aad8e224): a future PTB release may
        # rename or drop it, and on a broken init it can be ``None``, which
        # made ``asyncio.run_coroutine_threadsafe`` raise a swallowed
        # AttributeError that silently killed every TG reply. Cache the
        # loop we created explicitly so ``_on_response`` always schedules
        # onto a live, owned loop.
        self._telegram_loop: Optional[asyncio.AbstractEventLoop] = None
        g = ReentrantCallbackGroup()
        for topic, cb in ((self.camera_topic, self._on_camera_front), (self.camera_depth_topic, self._on_camera_depth), (self.camera_up_topic, self._on_camera_up)):
            self.create_subscription(CompressedImage, topic, cb, _BE, callback_group=g)
        self.create_subscription(OccupancyGrid, "/rtabmap/grid_prob_map", self._on_map, _TL, callback_group=g)
        self._stt_pub = self.create_publisher(String, "/voice/stt/result", _RE)
        self._response_pub = self.create_publisher(String, "/voice/dialogue/response", _RE)
        self._response_sub = self.create_subscription(String, "/voice/dialogue/response", self._on_response, _RE)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_web", _RE)
        self.tts_pub = self.create_publisher(String, "/voice/tts/request", _RE)
        token = os.getenv("TELEGRAM_BOT_TOKEN", "")
        if not token: self.get_logger().error("TELEGRAM_BOT_TOKEN not set"); return
        self._start_telegram_bot(token)
        self.get_logger().info("TelegramNode: thin ROS 2 bridge (W8)")
    def _on_camera_front(self, m): self.camera_cache.update(self.camera_topic, bytes(m.data))
    def _on_camera_depth(self, m): self.camera_cache.update(self.camera_depth_topic, bytes(m.data))
    def _on_camera_up(self, m): self.camera_cache.update(self.camera_up_topic, bytes(m.data))
    def _on_map(self, m): self.latest_map_grid = m
    def set_active_chat(self, chat_id: int) -> None: self._active_chat_id = chat_id
    def forward_to_stt(self, text: str) -> None:
        if not text: return
        m = String(); m.data = text; self._stt_pub.publish(m)
    def publish_tts(self, text: str) -> None:
        m = String(); m.data = json.dumps(
            {"ssml": f"<speak>{text}</speak>", "speech_id": str(uuid.uuid4()), "emotion": "neutral"},
            ensure_ascii=False); self._response_pub.publish(m)
    def _on_response(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warning(
                f"Dropping /voice/dialogue/response: invalid JSON ({exc!r}); payload[:200]={msg.data[:200]!r}"
            )
            payload = {}
        text = payload.get("ssml", msg.data).replace("<speak>", "").replace("</speak>", "").strip()
        if not (self._telegram_app and self._active_chat_id and self._telegram_loop and text):
            self.get_logger().warning(
                "Dropping response, bot not ready / no active chat "
                f"(app={bool(self._telegram_app)}, loop={bool(self._telegram_loop)}, "
                f"chat_id={self._active_chat_id}, text_len={len(text) if text else 0})"
            )
            return
        coro = self._telegram_app.bot.send_message(
            chat_id=self._active_chat_id, text=text
        )
        try:
            asyncio.run_coroutine_threadsafe(coro, self._telegram_loop)
        except RuntimeError as exc:
            # The captured loop was closed underneath us (bot crashed and
            # the outer ``_run_telegram_loop`` is between attempts, or the
            # node is being torn down). Log and drop the message rather
            # than crashing the ROS 2 executor — the next valid message
            # will be picked up after the loop is restored.
            self.get_logger().error(
                f"Failed to schedule TG send_message (loop closed?): {exc!r}"
            )
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
        # Capture the loop we are running on BEFORE any awaits; this is
        # the loop python-telegram-bot will use for all bot.send_message
        # coroutines, and the one ``_on_response`` (which fires on the
        # ROS 2 executor thread) must schedule into.
        self._telegram_loop = asyncio.get_event_loop()
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
            await app.updater.stop(); await app.stop(); await app.shutdown()


def main(args=None):
    rclpy.init(args=args); node = TelegramNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__": main()