#!/usr/bin/env python3
"""
Robot Control GUI - Интерфейс для управления и мониторинга робота РОББОКС

Функции:
- Отправка готовых TTS команд
- Мониторинг сообщений от пользователя (STT)
- Просмотр ответов DeepSeek и TTS
- Переключение анимаций
- Воспроизведение звуков
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import json
import os
import sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotControlGUI(Node):
    """ROS2 нода с GUI для управления роботом"""

    def __init__(self, root):
        # Сначала сохраняем root
        self.root = root
        self.root.title("🤖 РОББОКС Control Panel")
        self.root.geometry("1200x800")
        
        # Потом инициализируем ROS2 Node
        super().__init__("robot_control_gui")

        # ROS2 publishers
        self.tts_pub = self.create_publisher(String, "/voice/tts/request", 10)
        self.stt_pub = self.create_publisher(String, "/voice/stt/result", 10)
        self.animation_pub = self.create_publisher(String, "/voice/animation/request", 10)
        self.sound_pub = self.create_publisher(String, "/voice/sound/trigger", 10)

        # ROS2 subscribers
        self.create_subscription(String, "/voice/stt/result", self.stt_callback, 10)
        self.create_subscription(String, "/voice/dialogue/response", self.dialogue_callback, 10)
        self.create_subscription(String, "/voice/tts/request", self.tts_callback, 10)

        # GUI setup
        self.setup_gui()
        self.get_logger().info("🎮 Robot Control GUI запущен")

    def setup_gui(self):
        """Создание GUI интерфейса"""

        # Основной контейнер
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Настройка grid
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(3, weight=1)

        # ==================== ЛЕВАЯ КОЛОНКА: Управление ====================
        left_frame = ttk.LabelFrame(main_frame, text="🎛️ Управление", padding="5")
        left_frame.grid(row=0, column=0, rowspan=4, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5)

        # --- TTS Commands ---
        ttk.Label(left_frame, text="📢 Отправить TTS:", font=("Arial", 10, "bold")).pack(pady=5)

        self.tts_templates = {
            "Привет": '{"ssml": "<speak><prosody pitch=\'high\'>Привет!</prosody><break time=\'400ms\'/>Я РОББОКС.<break time=\'300ms\'/></speak>", "emotion": "happy"}',
            "Еду вперед": '{"ssml": "<speak>Еду вперёд.<break time=\'300ms\'/></speak>", "commands": ["move_forward:0.3"], "emotion": "neutral"}',
            "Стоп": '{"ssml": "<speak>Останавливаюсь.<break time=\'300ms\'/></speak>", "commands": ["stop"], "emotion": "neutral"}',
            "Думаю": '{"ssml": "<speak>Хм,<break time=\'200ms\'/> дай подумать.<break time=\'400ms\'/></speak>", "emotion": "thinking"}',
            "Ошибка": '{"ssml": "<speak>Ой,<break time=\'200ms\'/> что-то пошло не так.<break time=\'400ms\'/></speak>", "emotion": "sad"}',
            "Анекдот": '{"ssml": "<speak>Идёт программист в магазин.<break time=\'300ms\'/>Жена говорит купи батон.<break time=\'300ms\'/>Если будут яйца — возьми десяток.<break time=\'400ms\'/>Он вернулся с десятью батонами.<break time=\'300ms\'/><prosody pitch=\'high\'>Яйца были!</prosody><break time=\'500ms\'/></speak>", "emotion": "happy"}',
        }

        for label, ssml in self.tts_templates.items():
            btn = ttk.Button(
                left_frame,
                text=label,
                command=lambda s=ssml: self.send_tts(s),
            )
            btn.pack(fill=tk.X, pady=2)

        ttk.Separator(left_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        # --- Custom TTS ---
        ttk.Label(left_frame, text="✏️ Свой текст:", font=("Arial", 10, "bold")).pack(pady=5)
        self.custom_text = tk.Text(left_frame, height=3, width=30, wrap=tk.WORD)
        self.custom_text.pack(fill=tk.X, pady=5)
        ttk.Button(left_frame, text="Отправить текст", command=self.send_custom_tts).pack(fill=tk.X)

        ttk.Separator(left_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        # --- Animations ---
        ttk.Label(left_frame, text="🎨 Анимации:", font=("Arial", 10, "bold")).pack(pady=5)
        animations = ["idle", "listening", "talking", "thinking", "happy", "sad", "alert"]
        for anim in animations:
            ttk.Button(
                left_frame,
                text=anim.capitalize(),
                command=lambda a=anim: self.send_animation(a),
            ).pack(fill=tk.X, pady=2)

        ttk.Separator(left_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        # --- Sounds ---
        ttk.Label(left_frame, text="🔔 Звуки:", font=("Arial", 10, "bold")).pack(pady=5)
        sounds = ["cute", "confused", "sad", "thinking", "positive"]
        for sound in sounds:
            ttk.Button(
                left_frame,
                text=sound.capitalize(),
                command=lambda s=sound: self.send_sound(s),
            ).pack(fill=tk.X, pady=2)

        # ==================== ПРАВАЯ КОЛОНКА: Мониторинг ====================

        # --- STT Messages ---
        stt_frame = ttk.LabelFrame(main_frame, text="🎤 Сообщения от пользователя (STT)", padding="5")
        stt_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        main_frame.rowconfigure(0, weight=1)

        self.stt_log = scrolledtext.ScrolledText(stt_frame, height=10, wrap=tk.WORD, state=tk.DISABLED)
        self.stt_log.pack(fill=tk.BOTH, expand=True)
        self.stt_log.tag_config("timestamp", foreground="gray")
        self.stt_log.tag_config("user", foreground="blue", font=("Arial", 10, "bold"))

        # --- DeepSeek Responses ---
        deepseek_frame = ttk.LabelFrame(main_frame, text="🤖 Ответы DeepSeek", padding="5")
        deepseek_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        main_frame.rowconfigure(1, weight=1)

        self.deepseek_log = scrolledtext.ScrolledText(deepseek_frame, height=10, wrap=tk.WORD, state=tk.DISABLED)
        self.deepseek_log.pack(fill=tk.BOTH, expand=True)
        self.deepseek_log.tag_config("timestamp", foreground="gray")
        self.deepseek_log.tag_config("json", foreground="green")
        self.deepseek_log.tag_config("ssml", foreground="purple")

        # --- TTS Output ---
        tts_frame = ttk.LabelFrame(main_frame, text="🔊 Вывод TTS (что робот говорит)", padding="5")
        tts_frame.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        main_frame.rowconfigure(2, weight=1)

        self.tts_log = scrolledtext.ScrolledText(tts_frame, height=10, wrap=tk.WORD, state=tk.DISABLED)
        self.tts_log.pack(fill=tk.BOTH, expand=True)
        self.tts_log.tag_config("timestamp", foreground="gray")
        self.tts_log.tag_config("text", foreground="darkblue", font=("Arial", 10))

        # --- System Log ---
        system_frame = ttk.LabelFrame(main_frame, text="📋 Системный лог", padding="5")
        system_frame.grid(row=3, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        main_frame.rowconfigure(3, weight=1)

        self.system_log = scrolledtext.ScrolledText(system_frame, height=5, wrap=tk.WORD, state=tk.DISABLED)
        self.system_log.pack(fill=tk.BOTH, expand=True)
        self.system_log.tag_config("info", foreground="black")
        self.system_log.tag_config("error", foreground="red")

        # --- Buttons ---
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=4, column=0, columnspan=2, pady=10)

        ttk.Button(button_frame, text="🗑️ Очистить логи", command=self.clear_logs).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="❌ Выход", command=self.quit_app).pack(side=tk.LEFT, padx=5)

    # ==================== ROS2 Callbacks ====================

    def stt_callback(self, msg):
        """Получение сообщений от STT (речь пользователя)"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        text = msg.data

        self.stt_log.configure(state=tk.NORMAL)
        self.stt_log.insert(tk.END, f"[{timestamp}] ", "timestamp")
        self.stt_log.insert(tk.END, f"{text}\n", "user")
        self.stt_log.see(tk.END)
        self.stt_log.configure(state=tk.DISABLED)

        self.log_system(f"📥 STT: {text}")

    def dialogue_callback(self, msg):
        """Получение ответов от dialogue_node (DeepSeek)"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        try:
            data = json.loads(msg.data)

            self.deepseek_log.configure(state=tk.NORMAL)
            self.deepseek_log.insert(tk.END, f"[{timestamp}] ", "timestamp")
            self.deepseek_log.insert(tk.END, f"{json.dumps(data, ensure_ascii=False, indent=2)}\n\n", "json")
            self.deepseek_log.see(tk.END)
            self.deepseek_log.configure(state=tk.DISABLED)

            # Извлекаем SSML если есть
            if "ssml" in data:
                self.log_system(f"🤖 DeepSeek SSML: {data['ssml'][:100]}...")

        except json.JSONDecodeError:
            self.log_system(f"⚠️ Не удалось распарсить dialogue response: {msg.data[:100]}", error=True)

    def tts_callback(self, msg):
        """Получение того, что идёт в TTS"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        try:
            data = json.loads(msg.data)

            if data.get("chunk") == "end":
                return  # Пропускаем end chunks

            # Извлекаем чистый текст из SSML
            ssml = data.get("ssml", "")
            import re

            clean_text = re.sub(r"<[^>]+>", "", ssml).strip()

            self.tts_log.configure(state=tk.NORMAL)
            self.tts_log.insert(tk.END, f"[{timestamp}] ", "timestamp")
            self.tts_log.insert(tk.END, f"{clean_text}\n", "text")
            self.tts_log.see(tk.END)
            self.tts_log.configure(state=tk.DISABLED)

            self.log_system(f"🔊 TTS: {clean_text[:50]}...")

        except json.JSONDecodeError:
            pass  # Игнорируем некорректные JSON

    # ==================== Control Methods ====================

    def send_tts(self, ssml_json):
        """Отправить TTS команду"""
        try:
            # Формируем chunk
            msg_data = json.loads(ssml_json)
            chunk = {"chunk": 1, **msg_data}

            msg = String()
            msg.data = json.dumps(chunk)
            self.tts_pub.publish(msg)

            # Отправляем end chunk
            end_msg = String()
            end_msg.data = json.dumps({"chunk": "end"})
            self.tts_pub.publish(end_msg)

            self.log_system(f"✅ TTS отправлен: {list(msg_data.keys())}")

        except Exception as e:
            self.log_system(f"❌ Ошибка отправки TTS: {e}", error=True)

    def send_custom_tts(self):
        """Отправить произвольный текст в TTS"""
        text = self.custom_text.get("1.0", tk.END).strip()
        if not text:
            self.log_system("⚠️ Текст пустой!", error=True)
            return

        # Простой SSML
        ssml = f'<speak>{text}<break time="300ms"/></speak>'
        ssml_json = json.dumps({"ssml": ssml, "emotion": "neutral"})
        self.send_tts(ssml_json)
        self.custom_text.delete("1.0", tk.END)

    def send_animation(self, animation_name):
        """Переключить анимацию"""
        msg = String()
        msg.data = animation_name
        self.animation_pub.publish(msg)
        self.log_system(f"🎨 Анимация: {animation_name}")

    def send_sound(self, sound_name):
        """Воспроизвести звук"""
        msg = String()
        msg.data = sound_name
        self.sound_pub.publish(msg)
        self.log_system(f"🔔 Звук: {sound_name}")

    # ==================== Utility Methods ====================

    def log_system(self, message, error=False):
        """Логирование в системный лог"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        tag = "error" if error else "info"

        self.system_log.configure(state=tk.NORMAL)
        self.system_log.insert(tk.END, f"[{timestamp}] {message}\n", tag)
        self.system_log.see(tk.END)
        self.system_log.configure(state=tk.DISABLED)

        # Дублируем в ROS2 logger
        if error:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def clear_logs(self):
        """Очистить все логи"""
        for log in [self.stt_log, self.deepseek_log, self.tts_log, self.system_log]:
            log.configure(state=tk.NORMAL)
            log.delete("1.0", tk.END)
            log.configure(state=tk.DISABLED)
        self.log_system("🗑️ Логи очищены")

    def quit_app(self):
        """Выход из приложения"""
        self.log_system("👋 Закрытие GUI...")
        self.root.quit()
        rclpy.shutdown()


def main():
    # Сначала инициализируем ROS2
    rclpy.init()
    
    # Создаём Tkinter root
    root = tk.Tk()

    try:
        # Создаём GUI ноду
        gui_node = RobotControlGUI(root)
        
        # ROS2 в отдельном потоке
        import threading
        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(gui_node, timeout_sec=0.1)
        
        ros_thread = threading.Thread(target=ros_spin, daemon=True)
        ros_thread.start()
        
        # GUI в главном потоке
        root.mainloop()
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
