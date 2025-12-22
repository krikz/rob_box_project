#!/usr/bin/env python3
"""
Robot Control GUI - Простая версия без наследования Node

Упрощенный интерфейс для управления роботом
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import json
import threading
from datetime import datetime

# ROS2 будет импортирован позже
rclpy = None
String = None


class RobotControlGUI:
    """GUI для управления роботом (без наследования от Node)"""

    def __init__(self, root):
        self.root = root
        self.root.title("🤖 РОББОКС Control Panel")
        self.root.geometry("1200x800")
        
        # ROS2 components (будут созданы позже)
        self.node = None
        self.publishers = {}
        self.subscriptions = []
        
        self.setup_gui()

    def init_ros(self):
        """Инициализация ROS2 после создания GUI"""
        global rclpy, String
        import os
        import rclpy as rclpy_module
        from std_msgs.msg import String as StringMsg
        from rclpy.node import Node
        
        rclpy = rclpy_module
        String = StringMsg
        
        # НЕ используем namespace в Node - он задан в Zenoh конфиге
        # Создаём простую ноду БЕЗ namespace
        self.node = Node("robot_control_gui")
        
        # Publishers (топики с начальным / - абсолютные пути)
        self.publishers = {
            'tts': self.node.create_publisher(String, "/voice/tts/request", 10),
            'animation': self.node.create_publisher(String, "/voice/animation/request", 10),
            'sound': self.node.create_publisher(String, "/voice/sound/trigger", 10),
        }
        
        # Subscribers (топики с начальным / - абсолютные пути)
        self.subscriptions = [
            self.node.create_subscription(String, "/voice/stt/result", self.stt_callback, 10),
            self.node.create_subscription(String, "/voice/dialogue/response", self.dialogue_callback, 10),
            self.node.create_subscription(String, "/voice/tts/request", self.tts_callback, 10),
        ]
        
        self.log_system("✅ ROS2 инициализирован")

    def setup_gui(self):
        """Создание GUI интерфейса"""
        
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(3, weight=1)
        
        # ===== ЛЕВАЯ ПАНЕЛЬ =====
        left_frame = ttk.LabelFrame(main_frame, text="🎛️ Управление", padding="5")
        left_frame.grid(row=0, column=0, rowspan=4, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5)
        
        # TTS кнопки
        ttk.Label(left_frame, text="📢 TTS команды:", font=("Arial", 10, "bold")).pack(pady=5)
        
        tts_buttons = [
            ("Привет", '{"ssml": "<speak><prosody pitch=\'high\'>Привет!</prosody><break time=\'300ms\'/></speak>", "emotion": "happy"}'),
            ("Вперёд", '{"ssml": "<speak>Еду вперёд.<break time=\'300ms\'/></speak>", "commands": ["move_forward:0.3"]}'),
            ("Стоп", '{"ssml": "<speak>Останавливаюсь.<break time=\'300ms\'/></speak>", "commands": ["stop"]}'),
            ("Анекдот", '{"ssml": "<speak>Идёт программист.<break time=\'300ms\'/></speak>", "emotion": "happy"}'),
        ]
        
        for label, data in tts_buttons:
            ttk.Button(left_frame, text=label, command=lambda d=data: self.send_tts(d)).pack(fill=tk.X, pady=2)
        
        ttk.Separator(left_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        # Свой текст
        ttk.Label(left_frame, text="✏️ Свой текст:", font=("Arial", 10, "bold")).pack(pady=5)
        self.custom_text = tk.Text(left_frame, height=3, width=30, wrap=tk.WORD)
        self.custom_text.pack(fill=tk.X, pady=5)
        ttk.Button(left_frame, text="Отправить", command=self.send_custom_tts).pack(fill=tk.X)
        
        ttk.Separator(left_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        # Анимации (выпадающий список + кнопка)
        ttk.Label(left_frame, text="🎨 Анимации:", font=("Arial", 10, "bold")).pack(pady=5)
        
        animations_list = [
            "idle", "talking", "thinking", "happy", "sad",
            "angry", "surprised", "victory", "sleep", "wakeup",
            "charging", "low_battery", "error",
            "police_lights", "ambulance", "fire_truck", "road_service",
            "accelerating", "braking", "turn_left", "turn_right"
        ]
        
        self.animation_var = tk.StringVar(value="idle")
        animation_combo = ttk.Combobox(
            left_frame, 
            textvariable=self.animation_var,
            values=animations_list,
            state="readonly",
            width=18
        )
        animation_combo.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(
            left_frame, 
            text="▶️ Запустить анимацию", 
            command=self.send_selected_animation
        ).pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Separator(left_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        # Звуки (выпадающий список + кнопка)
        ttk.Label(left_frame, text="🔔 Звуки:", font=("Arial", 10, "bold")).pack(pady=5)
        
        sounds_list = ["cute", "confused", "sad", "thinking", "positive"]
        
        self.sound_var = tk.StringVar(value="cute")
        sound_combo = ttk.Combobox(
            left_frame,
            textvariable=self.sound_var,
            values=sounds_list,
            state="readonly",
            width=18
        )
        sound_combo.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(
            left_frame,
            text="▶️ Воспроизвести звук",
            command=self.send_selected_sound
        ).pack(fill=tk.X, padx=5, pady=5)
        
        # ===== ПРАВАЯ ПАНЕЛЬ =====
        
        # STT Log
        stt_frame = ttk.LabelFrame(main_frame, text="🎤 STT (пользователь)", padding="5")
        stt_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        main_frame.rowconfigure(0, weight=1)
        
        self.stt_log = scrolledtext.ScrolledText(stt_frame, height=10, wrap=tk.WORD, state=tk.DISABLED)
        self.stt_log.pack(fill=tk.BOTH, expand=True)
        self.stt_log.tag_config("time", foreground="gray")
        self.stt_log.tag_config("text", foreground="blue", font=("Arial", 10, "bold"))
        
        # DeepSeek Log
        ds_frame = ttk.LabelFrame(main_frame, text="🤖 DeepSeek", padding="5")
        ds_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        main_frame.rowconfigure(1, weight=1)
        
        self.ds_log = scrolledtext.ScrolledText(ds_frame, height=10, wrap=tk.WORD, state=tk.DISABLED)
        self.ds_log.pack(fill=tk.BOTH, expand=True)
        self.ds_log.tag_config("time", foreground="gray")
        self.ds_log.tag_config("json", foreground="green")
        
        # TTS Log
        tts_frame = ttk.LabelFrame(main_frame, text="🔊 TTS (робот)", padding="5")
        tts_frame.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        main_frame.rowconfigure(2, weight=1)
        
        self.tts_log = scrolledtext.ScrolledText(tts_frame, height=10, wrap=tk.WORD, state=tk.DISABLED)
        self.tts_log.pack(fill=tk.BOTH, expand=True)
        self.tts_log.tag_config("time", foreground="gray")
        self.tts_log.tag_config("text", foreground="darkblue")
        
        # System Log
        sys_frame = ttk.LabelFrame(main_frame, text="📋 Система", padding="5")
        sys_frame.grid(row=3, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        main_frame.rowconfigure(3, weight=1)
        
        self.sys_log = scrolledtext.ScrolledText(sys_frame, height=5, wrap=tk.WORD, state=tk.DISABLED)
        self.sys_log.pack(fill=tk.BOTH, expand=True)
        self.sys_log.tag_config("info", foreground="black")
        self.sys_log.tag_config("error", foreground="red")
        
        # Кнопки
        btn_frame = ttk.Frame(main_frame)
        btn_frame.grid(row=4, column=0, columnspan=2, pady=10)
        ttk.Button(btn_frame, text="🗑️ Очистить", command=self.clear_logs).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="❌ Выход", command=self.quit_app).pack(side=tk.LEFT, padx=5)

    # ===== ROS Callbacks =====
    
    def stt_callback(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.stt_log.configure(state=tk.NORMAL)
        self.stt_log.insert(tk.END, f"[{ts}] ", "time")
        self.stt_log.insert(tk.END, f"{msg.data}\n", "text")
        self.stt_log.see(tk.END)
        self.stt_log.configure(state=tk.DISABLED)
    
    def dialogue_callback(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        try:
            data = json.loads(msg.data)
            self.ds_log.configure(state=tk.NORMAL)
            self.ds_log.insert(tk.END, f"[{ts}] ", "time")
            self.ds_log.insert(tk.END, f"{json.dumps(data, ensure_ascii=False, indent=2)}\n\n", "json")
            self.ds_log.see(tk.END)
            self.ds_log.configure(state=tk.DISABLED)
        except:
            pass
    
    def tts_callback(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        try:
            data = json.loads(msg.data)
            if data.get("chunk") == "end":
                return
            ssml = data.get("ssml", "")
            import re
            text = re.sub(r"<[^>]+>", "", ssml).strip()
            
            self.tts_log.configure(state=tk.NORMAL)
            self.tts_log.insert(tk.END, f"[{ts}] ", "time")
            self.tts_log.insert(tk.END, f"{text}\n", "text")
            self.tts_log.see(tk.END)
            self.tts_log.configure(state=tk.DISABLED)
        except:
            pass
    
    # ===== Control Methods =====
    
    def send_tts(self, ssml_json):
        if not self.node:
            self.log_system("❌ ROS2 не инициализирован", error=True)
            return
        try:
            data = json.loads(ssml_json)
            chunk = {"chunk": 1, **data}
            
            msg = String()
            msg.data = json.dumps(chunk)
            
            topic_name = self.publishers['tts'].topic_name
            print(f"[DEBUG] Публикация TTS в топик: {topic_name}")
            print(f"[DEBUG] Данные chunk: {json.dumps(chunk, ensure_ascii=False)[:200]}")
            self.publishers['tts'].publish(msg)
            
            end = String()
            end.data = json.dumps({"chunk": "end"})
            print(f"[DEBUG] Публикация end chunk в топик: {topic_name}")
            self.publishers['tts'].publish(end)
            
            self.log_system(f"✅ TTS отправлен в {topic_name}")
        except Exception as e:
            print(f"[ERROR] Ошибка TTS: {e}")
            self.log_system(f"❌ Ошибка TTS: {e}", error=True)
    
    def send_custom_tts(self):
        text = self.custom_text.get("1.0", tk.END).strip()
        if not text:
            return
        
        # Проверяем есть ли уже <speak> тег
        if text.startswith('<speak>') and text.endswith('</speak>'):
            # Уже полный SSML - отправляем как есть
            ssml = text
        else:
            # Обычный текст - оборачиваем в <speak>
            ssml = f'<speak>{text}<break time="300ms"/></speak>'
        
        self.send_tts(json.dumps({"ssml": ssml, "emotion": "neutral"}))
        self.custom_text.delete("1.0", tk.END)
    
    def send_animation(self, name):
        if not self.node:
            return
        msg = String()
        msg.data = name
        topic_name = self.publishers['animation'].topic_name
        print(f"[DEBUG] Публикация анимации '{name}' в топик: {topic_name}")
        self.publishers['animation'].publish(msg)
        self.log_system(f"🎨 Анимация: {name} → {topic_name}")
    
    def send_selected_animation(self):
        """Отправить выбранную анимацию из списка"""
        animation = self.animation_var.get()
        self.send_animation(animation)
    
    def send_sound(self, name):
        if not self.node:
            return
        msg = String()
        msg.data = name
        topic_name = self.publishers['sound'].topic_name
        print(f"[DEBUG] Публикация звука '{name}' в топик: {topic_name}")
        self.publishers['sound'].publish(msg)
        self.log_system(f"🔔 Звук: {name} → {topic_name}")
    
    def send_selected_sound(self):
        """Воспроизвести выбранный звук из списка"""
        sound = self.sound_var.get()
        self.send_sound(sound)
    
    def log_system(self, msg, error=False):
        ts = datetime.now().strftime("%H:%M:%S")
        tag = "error" if error else "info"
        self.sys_log.configure(state=tk.NORMAL)
        self.sys_log.insert(tk.END, f"[{ts}] {msg}\n", tag)
        self.sys_log.see(tk.END)
        self.sys_log.configure(state=tk.DISABLED)
    
    def clear_logs(self):
        for log in [self.stt_log, self.ds_log, self.tts_log, self.sys_log]:
            log.configure(state=tk.NORMAL)
            log.delete("1.0", tk.END)
            log.configure(state=tk.DISABLED)
        self.log_system("🗑️ Очищено")
    
    def quit_app(self):
        self.log_system("👋 Выход...")
        if self.node:
            self.node.destroy_node()
        self.root.quit()


def main():
    # Сначала Tkinter
    root = tk.Tk()
    gui = RobotControlGUI(root)
    
    # Потом ROS2
    try:
        import rclpy
        rclpy.init()
        gui.init_ros()
        
        # ROS spin в фоне
        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(gui.node, timeout_sec=0.1)
        
        thread = threading.Thread(target=ros_spin, daemon=True)
        thread.start()
        
        # GUI
        root.mainloop()
        
    except Exception as e:
        print(f"Ошибка: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
