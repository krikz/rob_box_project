#!/usr/bin/env python3
"""
telegram_node.py — ROS 2 node bridging Telegram Bot API to the robot.

Subscribes to camera topics, MCP results, and publishes TTS/cmd_vel.
Runs python-telegram-bot Application in a background asyncio thread.

Architecture:
    Telegram (async, python-telegram-bot)  <->  TelegramNode (ROS 2)  <->  ROS 2 topics

Topics subscribed:
    /camera/camera/color/image_raw/compressed  (sensor_msgs/CompressedImage) — front camera
    /camera/camera/depth/image_rect_raw/compressedDepth (sensor_msgs/CompressedImage) — depth
    /ceiling_camera/image_raw/compressed (sensor_msgs/CompressedImage) — ceiling camera
    /rtabmap/grid_prob_map (nav_msgs/OccupancyGrid) — 2D SLAM map
    /mcp/result (std_msgs/String) — MCP tool execution results
    /mcp/tools  (std_msgs/String) — available MCP tool definitions

Topics published:
    /voice/tts/request (std_msgs/String) — text-to-speech requests
    /cmd_vel_web (geometry_msgs/Twist) — movement commands (priority 50)
    /mcp/execute (std_msgs/String) — MCP tool execution requests
"""

import asyncio
import json
import logging
import os
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from telegram import Update
from telegram.ext import (
    Application,
    CallbackQueryHandler,
    CommandHandler,
    MessageHandler,
    filters,
)

from .camera_cache import CameraCache
from .handlers.callbacks import callback_handler
from .handlers.commands import (
    animation_handler,
    clear_handler,
    control_handler,
    goto_handler,
    help_handler,
    map_handler,
    menu_handler,
    music_handler,
    myid_handler,
    photo_depth_handler,
    photo_handler,
    photo_map_handler,
    photo_up_handler,
    playvoice_handler,
    pose_handler,
    repl_handler,
    say_handler,
    sound_handler,
    start_handler,
    status_handler,
    stop_handler,
    stopmusic_handler,
    volume_handler,
    waypoints_handler,
)
from .handlers.messages import text_message_handler, voice_message_handler
from .llm_chat import LLMChat
from .mcp_bridge import MCPBridge

logger = logging.getLogger(__name__)


class TelegramNode(Node):
    """ROS 2 node for Telegram bot operator interface.

    Bridges Telegram Bot API ↔ ROS 2 topics for remote robot operation.
    """

    def __init__(self):
        super().__init__("telegram_node")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("max_linear_speed", 0.3)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("move_duration", 0.5)
        self.declare_parameter("camera_topic", "/camera/camera/color/image_raw/compressed")
        self.declare_parameter("camera_depth_topic", "/camera/camera/depth/image_rect_raw/compressedDepth")
        self.declare_parameter("camera_up_topic", "/ceiling_camera/image_raw/compressed")
        self.declare_parameter("camera_cache_ttl", 5.0)
        self.declare_parameter("llm_provider", "deepseek")
        self.declare_parameter("llm_max_history", 20)
        self.declare_parameter("llm_temperature", 0.7)
        self.declare_parameter("voice_stt_method", "yandex")
        self.declare_parameter("voice_stt_language", "ru-RU")
        self.declare_parameter("telegram_poll_timeout", 30)

        self.max_linear_speed: float = self.get_parameter("max_linear_speed").value
        self.max_angular_speed: float = self.get_parameter("max_angular_speed").value
        self.move_duration: float = self.get_parameter("move_duration").value
        self.camera_topic: str = self.get_parameter("camera_topic").value
        self.camera_depth_topic: str = self.get_parameter("camera_depth_topic").value
        self.camera_up_topic: str = self.get_parameter("camera_up_topic").value
        camera_cache_ttl: float = self.get_parameter("camera_cache_ttl").value
        llm_provider: str = self.get_parameter("llm_provider").value
        llm_max_history: int = self.get_parameter("llm_max_history").value
        llm_temperature: float = self.get_parameter("llm_temperature").value
        self.voice_stt_method: str = self.get_parameter("voice_stt_method").value
        self.voice_stt_language: str = self.get_parameter("voice_stt_language").value
        poll_timeout: int = self.get_parameter("telegram_poll_timeout").value

        # ── Camera cache ────────────────────────────────────────────
        self.camera_cache = CameraCache(ttl=camera_cache_ttl)        # Latest SLAM occupancy grid (stored raw for on-demand rendering)
        self.latest_map_grid: Optional[OccupancyGrid] = None
        # ── QoS profiles ────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        transient_local_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        cb_group = ReentrantCallbackGroup()

        # ── Subscribers ─────────────────────────────────────────────
        self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self._on_camera_front,
            best_effort_qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            CompressedImage,
            self.camera_depth_topic,
            self._on_camera_depth,
            best_effort_qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            CompressedImage,
            self.camera_up_topic,
            self._on_camera_up,
            best_effort_qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            OccupancyGrid,
            "/rtabmap/grid_prob_map",
            self._on_map,
            transient_local_qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            String,
            "/mcp/result",
            self._on_mcp_result,
            reliable_qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            String,
            "/mcp/tools",
            self._on_mcp_tools,
            reliable_qos,
            callback_group=cb_group,
        )

        # ── Publishers ──────────────────────────────────────────────
        self.tts_pub = self.create_publisher(String, "/voice/tts/request", reliable_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_web", reliable_qos)
        execute_pub = self.create_publisher(String, "/mcp/execute", reliable_qos)

        # ── MCP Bridge ──────────────────────────────────────────────
        self.mcp_bridge = MCPBridge(execute_pub, ros_logger=self.get_logger())

        # ── LLM Chat ────────────────────────────────────────────────
        self.llm_chat = LLMChat(
            provider=llm_provider,
            max_history=llm_max_history,
            temperature=llm_temperature,
        )

        # ── Telegram Bot ────────────────────────────────────────────
        token = os.getenv("TELEGRAM_BOT_TOKEN", "")
        if not token:
            self.get_logger().error("TELEGRAM_BOT_TOKEN not set! Bot will not start.")
            return

        self._telegram_app: Optional[Application] = None
        self._telegram_thread: Optional[threading.Thread] = None
        self._poll_timeout = poll_timeout

        self._start_telegram_bot(token)

        self.get_logger().info("🤖 TelegramNode initialized")

    # ── ROS 2 Callbacks ─────────────────────────────────────────────

    def _on_camera_front(self, msg: CompressedImage) -> None:
        """Cache latest front camera frame."""
        self.camera_cache.update(self.camera_topic, bytes(msg.data))

    def _on_camera_depth(self, msg: CompressedImage) -> None:
        """Cache latest depth frame (compressedDepth format)."""
        self.camera_cache.update(self.camera_depth_topic, bytes(msg.data))

    def _on_camera_up(self, msg: CompressedImage) -> None:
        """Cache latest ceiling camera frame."""
        self.camera_cache.update(self.camera_up_topic, bytes(msg.data))

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Store latest SLAM occupancy grid for /photo_map rendering."""
        self.latest_map_grid = msg

    def _on_mcp_result(self, msg: String) -> None:
        """Forward MCP result to the bridge for request correlation."""
        self.mcp_bridge.on_result(msg)

    def _on_mcp_tools(self, msg: String) -> None:
        """Update LLM chat with available MCP tool definitions."""
        try:
            tools = json.loads(msg.data)
            if isinstance(tools, list):
                tool_names = sorted(
                    t.get("function", {}).get("name", t.get("name", "?")) for t in tools
                )
                # Log only when the tool set actually changes
                prev_names = getattr(self, "_last_tool_names", None)
                if tool_names != prev_names:
                    self._last_tool_names = tool_names
                    self.llm_chat.update_tools(tools)
                    self.get_logger().info(
                        f"🔧 MCP tools updated ({len(tools)}): {', '.join(tool_names)}"
                    )
                else:
                    self.llm_chat.update_tools(tools)
        except json.JSONDecodeError as e:
            self.get_logger().warning(f"⚠️ Failed to parse MCP tools: {e}")

    # ── Publish helpers ─────────────────────────────────────────────

    def publish_tts(self, text: str) -> None:
        """Publish text for the robot to speak via /voice/tts/request.

        tts_node expects JSON: {"ssml": "<speak>text</speak>", "speech_id": "..."}
        """
        import uuid

        payload = json.dumps(
            {
                "ssml": f"<speak>{text}</speak>",
                "speech_id": str(uuid.uuid4()),
            },
            ensure_ascii=False,
        )
        msg = String()
        msg.data = payload
        self.tts_pub.publish(msg)
        self.get_logger().info(f"TTS request: {text[:80]}")

    # ── Telegram Bot Setup ──────────────────────────────────────────

    def _start_telegram_bot(self, token: str) -> None:
        """Build and start the Telegram bot in a background thread."""
        self._telegram_thread = threading.Thread(
            target=self._run_telegram_loop,
            args=(token,),
            daemon=True,
            name="telegram-bot",
        )
        self._telegram_thread.start()
        self.get_logger().info("Telegram bot thread started")

    def _run_telegram_loop(self, token: str) -> None:
        """Entry point for the Telegram bot background thread. Retries on crash."""
        retry_delay = 5.0
        max_retry_delay = 60.0
        attempt = 0

        while rclpy.ok():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self._run_telegram(token))
                loop.close()
                break  # clean shutdown — no retry needed
            except Exception as e:
                attempt += 1
                delay = min(retry_delay * attempt, max_retry_delay)
                self.get_logger().error(
                    f"Telegram bot crashed (attempt {attempt}): {e}. Restarting in {delay:.0f}s..."
                )
                loop.close()
                time.sleep(delay)

    async def _run_telegram(self, token: str) -> None:
        """Build Telegram Application, register handlers, and start polling."""
        app = Application.builder().token(token).build()
        self._telegram_app = app

        # Pass this node to all handlers via bot_data
        app.bot_data["node"] = self

        # ── Register command handlers ───────────────────────────────
        app.add_handler(CommandHandler("start", start_handler))
        app.add_handler(CommandHandler("myid", myid_handler))
        app.add_handler(CommandHandler("help", help_handler))
        app.add_handler(CommandHandler("menu", menu_handler))
        app.add_handler(CommandHandler("photo", photo_handler))
        app.add_handler(CommandHandler("photo_up", photo_up_handler))
        app.add_handler(CommandHandler("photo_depth", photo_depth_handler))
        app.add_handler(CommandHandler("photo_map", photo_map_handler))
        app.add_handler(CommandHandler("say", say_handler))
        app.add_handler(CommandHandler("playvoice", playvoice_handler))
        app.add_handler(CommandHandler("status", status_handler))
        app.add_handler(CommandHandler("waypoints", waypoints_handler))
        app.add_handler(CommandHandler("goto", goto_handler))
        app.add_handler(CommandHandler("stop", stop_handler))
        app.add_handler(CommandHandler("pose", pose_handler))
        app.add_handler(CommandHandler("control", control_handler))
        app.add_handler(CommandHandler("volume", volume_handler))
        app.add_handler(CommandHandler("animation", animation_handler))
        app.add_handler(CommandHandler("sound", sound_handler))
        app.add_handler(CommandHandler("map", map_handler))
        app.add_handler(CommandHandler("music", music_handler))
        app.add_handler(CommandHandler("repl", repl_handler))
        app.add_handler(CommandHandler("stopmusic", stopmusic_handler))
        app.add_handler(CommandHandler("clear", clear_handler))

        # ── Callback query handler (inline keyboards) ──────────────
        app.add_handler(CallbackQueryHandler(callback_handler))

        # ── Voice message handler ──────────────────────────────────
        app.add_handler(MessageHandler(filters.VOICE, voice_message_handler))

        # ── Text message handler (LLM chat — must be last) ────────
        app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, text_message_handler))

        self.get_logger().info("Telegram bot handlers registered, starting polling...")

        # Initialize and start polling
        await app.initialize()
        await app.start()
        await app.updater.start_polling(poll_interval=1.0, timeout=self._poll_timeout)

        self.get_logger().info("✅ Telegram bot is running")

        # Keep running until shutdown
        try:
            while rclpy.ok():
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            pass
        finally:
            self.get_logger().info("Telegram bot shutting down...")
            await app.updater.stop()
            await app.stop()
            await app.shutdown()

    def destroy_node(self) -> None:
        """Clean shutdown of the Telegram bot."""
        self.get_logger().info("Destroying TelegramNode...")
        super().destroy_node()


def main(args=None):
    """ROS 2 entry point."""
    rclpy.init(args=args)
    node = TelegramNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
