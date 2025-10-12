#!/usr/bin/env python3
"""
Графический интерфейс для настройки и тестирования Silero TTS v4

Возможности:
- Выбор голоса (aidar, baya, kseniya, xenia)
- Настройка pitch через SSML (x-low, low, medium, high, x-high)
- Настройка rate через SSML (x-slow, slow, medium, fast, x-fast)
- Настройка sample_rate (24000, 48000 Hz)
- Настройка threads (1-16)
- Произвольный текст для тестирования
- SSML редактор с примерами
- Измерение производительности (RTF, латентность)
- Сохранение/загрузка конфигурации
- Экспорт в WAV

Требования:
    pip install torch torchaudio tkinter sounddevice numpy
"""

import os
import sys
import json
import time
import wave
import tempfile
from pathlib import Path
from typing import Optional, Dict, Any
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import urllib.request

try:
    import torch
    import sounddevice as sd
    import numpy as np
except ImportError as e:
    print(f"❌ Ошибка импорта: {e}")
    print("\nУстановите зависимости:")
    print("  pip install torch torchaudio sounddevice numpy")
    sys.exit(1)


class SileroTTSGUI:
    """Графический интерфейс для Silero TTS"""
    
    # Константы
    VOICES = {
        "aidar": "Aidar (M) - нейтральный, чёткий, серьёзный ✨",
        "baya": "Baya (M) - спокойный, мягкий, дружелюбный",
        "kseniya": "Kseniya (F) - нейтральный, профессиональный",
        "xenia": "Xenia (F) - энергичный, живой ✨"
    }
    
    PITCH_VALUES = ["x-low", "low", "medium", "high", "x-high"]
    RATE_VALUES = ["x-slow", "slow", "medium", "fast", "x-fast"]
    SAMPLE_RATES = [24000, 48000]
    
    MODEL_URL = "https://models.silero.ai/models/tts/ru/v4_ru.pt"
    
    # Примеры текстов
    EXAMPLE_TEXTS = {
        "Приветствие": "Привет! Я голосовой ассистент робота ROBBOX.",
        "Команда": "Принял команду. Выполняю движение вперёд.",
        "Предупреждение": "Внимание! Обнаружено препятствие впереди.",
        "Вопрос": "Что мне делать дальше?",
        "Ответ": "Дай подумаю. Кажется, я знаю решение.",
        "Ошибка": "Извините, произошла ошибка. Попробуйте ещё раз.",
        "Батарея": "Уровень заряда батареи низкий. Требуется подзарядка.",
        "Успех": "Отлично! Задача выполнена успешно!"
    }
    
    # SSML примеры
    SSML_EXAMPLES = {
        "Базовый": '<speak>{text}</speak>',
        "С pitch": '<speak><prosody pitch="{pitch}">{text}</prosody></speak>',
        "С rate": '<speak><prosody rate="{rate}">{text}</prosody></speak>',
        "С pitch+rate": '<speak><prosody pitch="{pitch}" rate="{rate}">{text}</prosody></speak>',
        "С паузой": '<speak>{text}<break time="1s"/>Продолжение после паузы.</speak>',
        "Эмоции": '<speak><prosody pitch="high" rate="fast">Ура!</prosody><break time="500ms"/><prosody pitch="low" rate="slow">Но есть проблема.</prosody></speak>'
    }
    
    # Имперский марш (Imperial March) - мелодия через SSML
    # Используем только поддерживаемые Silero значения: x-low, low, medium, high, x-high
    # Ноты: G(medium) G(medium) G(medium) | Eb(low) Bb(high) G(medium)
    # Более выраженная мелодия с правильными паузами
    IMPERIAL_MARCH_TEMPLATE = '''<speak>
<prosody rate="{base_rate}">
<prosody pitch="medium">Дан</prosody><break time="600ms"/>
<prosody pitch="medium">Дан</prosody><break time="600ms"/>
<prosody pitch="medium">Дан</prosody><break time="800ms"/>
<prosody pitch="low">Дааа</prosody><break time="350ms"/>
<prosody pitch="high">ан</prosody><break time="150ms"/>
<prosody pitch="medium">Дан</prosody><break time="800ms"/>
<prosody pitch="low">Дааа</prosody><break time="350ms"/>
<prosody pitch="high">ан</prosody><break time="150ms"/>
<prosody pitch="medium">Дан</prosody><break time="1400ms"/>
<prosody pitch="x-high">Дан</prosody><break time="600ms"/>
<prosody pitch="x-high">Дан</prosody><break time="600ms"/>
<prosody pitch="x-high">Дан</prosody><break time="800ms"/>
<prosody pitch="high">Дааа</prosody><break time="350ms"/>
<prosody pitch="x-high">ан</prosody><break time="150ms"/>
<prosody pitch="medium">Дан</prosody><break time="800ms"/>
<prosody pitch="low">Дааа</prosody><break time="350ms"/>
<prosody pitch="high">ан</prosody><break time="150ms"/>
<prosody pitch="medium">Дан</prosody>
</prosody>
</speak>'''
    
    def __init__(self, root: tk.Tk):
        """Инициализация GUI"""
        self.root = root
        self.root.title("Silero TTS v4 - Настройка и тестирование")
        self.root.geometry("1000x800")
        
        # Модель и настройки
        self.model = None
        self.model_path = Path.home() / ".cache/rob_box_voice/tts_models/silero_v4_ru.pt"
        
        # Настройки по умолчанию
        self.config = {
            "speaker": "aidar",
            "pitch": "medium",
            "rate": "medium",
            "sample_rate": 48000,
            "threads": 4,
            "use_ssml": True
        }
        
        # Статистика
        self.last_synthesis_time = 0.0
        self.last_audio_duration = 0.0
        self.last_rtf = 0.0
        
        # Создание интерфейса
        self.create_widgets()
        
        # Загрузка модели после отрисовки GUI
        self.root.after(100, self.load_model)
    
    def create_widgets(self):
        """Создание виджетов интерфейса"""
        
        # Главный контейнер с прокруткой
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # ========== Секция 1: Выбор голоса ==========
        voice_frame = ttk.LabelFrame(main_frame, text="Выбор голоса", padding="10")
        voice_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.voice_var = tk.StringVar(value=self.config["speaker"])
        for idx, (voice_id, voice_name) in enumerate(self.VOICES.items()):
            rb = ttk.Radiobutton(voice_frame, text=voice_name, variable=self.voice_var, 
                                value=voice_id, command=self.on_voice_change)
            rb.grid(row=idx//2, column=idx%2, sticky=tk.W, padx=5, pady=2)
        
        # ========== Секция 2: SSML настройки ==========
        ssml_frame = ttk.LabelFrame(main_frame, text="SSML параметры", padding="10")
        ssml_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Pitch (тон голоса)
        ttk.Label(ssml_frame, text="Pitch (тон):").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.pitch_var = tk.StringVar(value=self.config["pitch"])
        pitch_combo = ttk.Combobox(ssml_frame, textvariable=self.pitch_var, 
                                   values=self.PITCH_VALUES, state="readonly", width=15)
        pitch_combo.grid(row=0, column=1, sticky=tk.W, padx=5)
        pitch_combo.bind("<<ComboboxSelected>>", self.on_pitch_change)
        
        ttk.Label(ssml_frame, text="← x-low (низкий) | x-high (высокий) →").grid(
            row=0, column=2, sticky=tk.W, padx=10)
        
        # Rate (скорость речи)
        ttk.Label(ssml_frame, text="Rate (скорость):").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.rate_var = tk.StringVar(value=self.config["rate"])
        rate_combo = ttk.Combobox(ssml_frame, textvariable=self.rate_var, 
                                 values=self.RATE_VALUES, state="readonly", width=15)
        rate_combo.grid(row=1, column=1, sticky=tk.W, padx=5)
        rate_combo.bind("<<ComboboxSelected>>", self.on_rate_change)
        
        ttk.Label(ssml_frame, text="← x-slow (медленно) | x-fast (быстро) →").grid(
            row=1, column=2, sticky=tk.W, padx=10)
        
        # Use SSML checkbox
        self.use_ssml_var = tk.BooleanVar(value=self.config["use_ssml"])
        ssml_check = ttk.Checkbutton(ssml_frame, text="Использовать SSML", 
                                    variable=self.use_ssml_var, command=self.on_ssml_toggle)
        ssml_check.grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=5)
        
        # ========== Секция 3: Технические параметры ==========
        tech_frame = ttk.LabelFrame(main_frame, text="Технические параметры", padding="10")
        tech_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Sample Rate
        ttk.Label(tech_frame, text="Sample Rate (Hz):").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.sample_rate_var = tk.IntVar(value=self.config["sample_rate"])
        for idx, sr in enumerate(self.SAMPLE_RATES):
            rb = ttk.Radiobutton(tech_frame, text=str(sr), variable=self.sample_rate_var, 
                                value=sr, command=self.on_sample_rate_change)
            rb.grid(row=0, column=idx+1, sticky=tk.W, padx=5)
        
        # Threads
        ttk.Label(tech_frame, text="CPU Threads:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.threads_var = tk.IntVar(value=self.config["threads"])
        threads_spin = ttk.Spinbox(tech_frame, from_=1, to=16, textvariable=self.threads_var, 
                                   width=10, command=self.on_threads_change)
        threads_spin.grid(row=1, column=1, sticky=tk.W, padx=5)
        
        ttk.Label(tech_frame, text="(оптимально 4 для Pi 5)").grid(
            row=1, column=2, sticky=tk.W, padx=10)
        
        # ========== Секция 4: Текст для синтеза ==========
        text_frame = ttk.LabelFrame(main_frame, text="Текст для синтеза", padding="10")
        text_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        # Примеры текстов
        examples_frame = ttk.Frame(text_frame)
        examples_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(examples_frame, text="Примеры:").pack(side=tk.LEFT, padx=5)
        for example_name in self.EXAMPLE_TEXTS.keys():
            btn = ttk.Button(examples_frame, text=example_name, 
                           command=lambda name=example_name: self.load_example_text(name))
            btn.pack(side=tk.LEFT, padx=2)
        
        # Текстовое поле
        self.text_editor = scrolledtext.ScrolledText(text_frame, width=80, height=6, wrap=tk.WORD)
        self.text_editor.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        self.text_editor.insert("1.0", self.EXAMPLE_TEXTS["Приветствие"])
        
        text_frame.rowconfigure(1, weight=1)
        
        # ========== Секция 5: SSML редактор ==========
        ssml_editor_frame = ttk.LabelFrame(main_frame, text="SSML редактор (опционально)", padding="10")
        ssml_editor_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        # SSML примеры
        ssml_examples_frame = ttk.Frame(ssml_editor_frame)
        ssml_examples_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(ssml_examples_frame, text="Шаблоны:").pack(side=tk.LEFT, padx=5)
        for template_name in self.SSML_EXAMPLES.keys():
            btn = ttk.Button(ssml_examples_frame, text=template_name, 
                           command=lambda name=template_name: self.load_ssml_template(name))
            btn.pack(side=tk.LEFT, padx=2)
        
        # SSML поле
        self.ssml_editor = scrolledtext.ScrolledText(ssml_editor_frame, width=80, height=4, wrap=tk.WORD)
        self.ssml_editor.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        ssml_editor_frame.rowconfigure(1, weight=1)
        
        # ========== Секция 6: Кнопки управления ==========
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=5, column=0, columnspan=2, pady=10)
        
        self.load_model_btn = ttk.Button(control_frame, text="📥 Загрузить модель", 
                                         command=self.load_model, width=20)
        self.load_model_btn.pack(side=tk.LEFT, padx=5)
        
        self.synthesize_btn = ttk.Button(control_frame, text="🔊 Синтезировать и проиграть", 
                                        command=self.synthesize_and_play, width=30)
        self.synthesize_btn.pack(side=tk.LEFT, padx=5)
        
        self.export_btn = ttk.Button(control_frame, text="💾 Экспорт в WAV", 
                                    command=self.export_to_wav, width=20)
        self.export_btn.pack(side=tk.LEFT, padx=5)
        
        self.save_config_btn = ttk.Button(control_frame, text="📝 Сохранить конфиг", 
                                         command=self.save_config, width=20)
        self.save_config_btn.pack(side=tk.LEFT, padx=5)
        
        self.load_config_btn = ttk.Button(control_frame, text="📂 Загрузить конфиг", 
                                         command=self.load_config, width=20)
        self.load_config_btn.pack(side=tk.LEFT, padx=5)
        
        # Кнопка для Имперского марша
        control_frame2 = ttk.Frame(main_frame)
        control_frame2.grid(row=6, column=0, columnspan=2, pady=5)
        
        self.imperial_march_btn = ttk.Button(control_frame2, text="🎵 Имперский марш (Star Wars)", 
                                            command=self.play_imperial_march, width=40)
        self.imperial_march_btn.pack(pady=5)
        
        ttk.Label(control_frame2, text="Демонстрация возможностей SSML: мелодия через изменение pitch и паузы", 
                 font=("Arial", 8), foreground="gray").pack()
        
        # ========== Секция 7: Статистика ==========
        stats_frame = ttk.LabelFrame(main_frame, text="Статистика производительности", padding="10")
        stats_frame.grid(row=7, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.stats_label = ttk.Label(stats_frame, text="Нажмите 'Синтезировать' для измерения...", 
                                     font=("Courier", 10))
        self.stats_label.pack(pady=5)
        
        # ========== Секция 8: Статус ==========
        self.status_label = ttk.Label(main_frame, text="Загрузка модели...", 
                                      relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.grid(row=8, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Настройка весов для адаптивности
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(3, weight=2)
        main_frame.rowconfigure(4, weight=1)
    
    def load_model(self):
        """Загрузка модели Silero"""
        self.status_label.config(text="Загрузка модели Silero v4...")
        self.root.update()
        
        try:
            # Проверка и скачивание модели
            if not self.model_path.exists():
                self.status_label.config(text="Скачивание модели (100 MB)...")
                self.root.update()
                
                self.model_path.parent.mkdir(parents=True, exist_ok=True)
                urllib.request.urlretrieve(self.MODEL_URL, self.model_path)
            
            # Загрузка модели (torch.package format для v4)
            self.status_label.config(text="Загрузка модели в память...")
            self.root.update()
            
            self.model = torch.package.PackageImporter(str(self.model_path)).load_pickle("tts_models", "model")
            self.model.to('cpu')
            torch.set_num_threads(self.config["threads"])
            
            self.status_label.config(text="✅ Модель загружена! Готов к работе.")
            messagebox.showinfo("Успех", "Silero TTS v4 загружен успешно!")
            
        except Exception as e:
            error_msg = f"Ошибка загрузки модели: {e}"
            self.status_label.config(text=f"❌ {error_msg}")
            messagebox.showerror("Ошибка", error_msg)
    
    def on_voice_change(self):
        """Обработчик смены голоса"""
        self.config["speaker"] = self.voice_var.get()
        self.status_label.config(text=f"Голос изменён: {self.VOICES[self.config['speaker']]}")
    
    def on_pitch_change(self, event=None):
        """Обработчик смены pitch"""
        self.config["pitch"] = self.pitch_var.get()
        self.status_label.config(text=f"Pitch изменён: {self.config['pitch']}")
    
    def on_rate_change(self, event=None):
        """Обработчик смены rate"""
        self.config["rate"] = self.rate_var.get()
        self.status_label.config(text=f"Rate изменён: {self.config['rate']}")
    
    def on_ssml_toggle(self):
        """Обработчик переключения SSML"""
        self.config["use_ssml"] = self.use_ssml_var.get()
        status = "включен" if self.config["use_ssml"] else "отключен"
        self.status_label.config(text=f"SSML {status}")
    
    def on_sample_rate_change(self):
        """Обработчик смены sample rate"""
        self.config["sample_rate"] = self.sample_rate_var.get()
        self.status_label.config(text=f"Sample Rate: {self.config['sample_rate']} Hz")
    
    def on_threads_change(self):
        """Обработчик смены threads"""
        self.config["threads"] = self.threads_var.get()
        if self.model:
            torch.set_num_threads(self.config["threads"])
        self.status_label.config(text=f"CPU Threads: {self.config['threads']}")
    
    def load_example_text(self, example_name: str):
        """Загрузка примера текста"""
        text = self.EXAMPLE_TEXTS.get(example_name, "")
        self.text_editor.delete("1.0", tk.END)
        self.text_editor.insert("1.0", text)
        self.status_label.config(text=f"Загружен пример: {example_name}")
    
    def load_ssml_template(self, template_name: str):
        """Загрузка SSML шаблона"""
        template = self.SSML_EXAMPLES.get(template_name, "")
        text = self.text_editor.get("1.0", tk.END).strip()
        
        # Подстановка параметров
        ssml = template.format(
            text=text,
            pitch=self.config["pitch"],
            rate=self.config["rate"]
        )
        
        self.ssml_editor.delete("1.0", tk.END)
        self.ssml_editor.insert("1.0", ssml)
        self.status_label.config(text=f"Загружен SSML шаблон: {template_name}")
    
    def synthesize_and_play(self):
        """Синтез и воспроизведение речи"""
        if not self.model:
            messagebox.showerror("Ошибка", "Модель не загружена!")
            return
        
        try:
            # Получение текста
            ssml_text = self.ssml_editor.get("1.0", tk.END).strip()
            plain_text = self.text_editor.get("1.0", tk.END).strip()
            
            if not plain_text:
                messagebox.showwarning("Внимание", "Введите текст для синтеза!")
                return
            
            # Определение текста для синтеза
            if ssml_text and self.config["use_ssml"]:
                synthesis_text = ssml_text
                use_ssml_param = True
            elif self.config["use_ssml"]:
                # Автоматическая генерация SSML
                synthesis_text = f'<speak><prosody pitch="{self.config["pitch"]}" rate="{self.config["rate"]}">{plain_text}</prosody></speak>'
                use_ssml_param = True
            else:
                synthesis_text = plain_text
                use_ssml_param = False
            
            # Статус
            self.status_label.config(text="Синтезирую речь...")
            self.root.update()
            
            # Синтез
            start_time = time.time()
            
            if use_ssml_param:
                audio = self.model.apply_tts(
                    ssml_text=synthesis_text,
                    speaker=self.config["speaker"],
                    sample_rate=self.config["sample_rate"]
                )
            else:
                audio = self.model.apply_tts(
                    text=synthesis_text,
                    speaker=self.config["speaker"],
                    sample_rate=self.config["sample_rate"]
                )
            
            synthesis_time = time.time() - start_time
            
            # Конвертация в numpy array
            if isinstance(audio, torch.Tensor):
                audio_np = audio.numpy()
            else:
                audio_np = np.array(audio)
            
            # Статистика
            audio_duration = len(audio_np) / self.config["sample_rate"]
            rtf = synthesis_time / audio_duration if audio_duration > 0 else 0.0
            
            self.last_synthesis_time = synthesis_time
            self.last_audio_duration = audio_duration
            self.last_rtf = rtf
            
            # Обновление статистики
            self.update_stats()
            
            # Воспроизведение
            self.status_label.config(text="Воспроизведение...")
            self.root.update()
            
            sd.play(audio_np, self.config["sample_rate"])
            sd.wait()
            
            self.status_label.config(text="✅ Готово!")
            
        except Exception as e:
            error_msg = f"Ошибка синтеза: {e}"
            self.status_label.config(text=f"❌ {error_msg}")
            messagebox.showerror("Ошибка", error_msg)
    
    def export_to_wav(self):
        """Экспорт в WAV файл"""
        if not self.model:
            messagebox.showerror("Ошибка", "Модель не загружена!")
            return
        
        try:
            # Диалог сохранения
            filename = filedialog.asksaveasfilename(
                defaultextension=".wav",
                filetypes=[("WAV files", "*.wav"), ("All files", "*.*")]
            )
            
            if not filename:
                return
            
            # Получение текста
            ssml_text = self.ssml_editor.get("1.0", tk.END).strip()
            plain_text = self.text_editor.get("1.0", tk.END).strip()
            
            if not plain_text:
                messagebox.showwarning("Внимание", "Введите текст для синтеза!")
                return
            
            # Синтез (аналогично synthesize_and_play)
            if ssml_text and self.config["use_ssml"]:
                synthesis_text = ssml_text
                use_ssml_param = True
            elif self.config["use_ssml"]:
                synthesis_text = f'<speak><prosody pitch="{self.config["pitch"]}" rate="{self.config["rate"]}">{plain_text}</prosody></speak>'
                use_ssml_param = True
            else:
                synthesis_text = plain_text
                use_ssml_param = False
            
            self.status_label.config(text="Синтезирую для экспорта...")
            self.root.update()
            
            if use_ssml_param:
                audio = self.model.apply_tts(
                    ssml_text=synthesis_text,
                    speaker=self.config["speaker"],
                    sample_rate=self.config["sample_rate"]
                )
            else:
                audio = self.model.apply_tts(
                    text=synthesis_text,
                    speaker=self.config["speaker"],
                    sample_rate=self.config["sample_rate"]
                )
            
            # Конвертация в numpy
            if isinstance(audio, torch.Tensor):
                audio_np = audio.numpy()
            else:
                audio_np = np.array(audio)
            
            # Нормализация и конвертация в int16
            audio_int16 = np.int16(audio_np / np.max(np.abs(audio_np)) * 32767)
            
            # Сохранение WAV
            with wave.open(filename, 'w') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)  # 16-bit
                wav_file.setframerate(self.config["sample_rate"])
                wav_file.writeframes(audio_int16.tobytes())
            
            self.status_label.config(text=f"✅ Сохранено: {filename}")
            messagebox.showinfo("Успех", f"Аудио сохранено:\n{filename}")
            
        except Exception as e:
            error_msg = f"Ошибка экспорта: {e}"
            self.status_label.config(text=f"❌ {error_msg}")
            messagebox.showerror("Ошибка", error_msg)
    
    def save_config(self):
        """Сохранение конфигурации в JSON"""
        try:
            filename = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                initialfile="silero_config.json"
            )
            
            if not filename:
                return
            
            config_data = {
                "config": self.config,
                "text": self.text_editor.get("1.0", tk.END).strip(),
                "ssml": self.ssml_editor.get("1.0", tk.END).strip()
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=2, ensure_ascii=False)
            
            self.status_label.config(text=f"✅ Конфиг сохранён: {filename}")
            messagebox.showinfo("Успех", f"Конфигурация сохранена:\n{filename}")
            
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка сохранения конфига: {e}")
    
    def load_config(self):
        """Загрузка конфигурации из JSON"""
        try:
            filename = filedialog.askopenfilename(
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
            )
            
            if not filename:
                return
            
            with open(filename, 'r', encoding='utf-8') as f:
                config_data = json.load(f)
            
            # Загрузка конфига
            self.config = config_data.get("config", self.config)
            
            # Обновление UI
            self.voice_var.set(self.config["speaker"])
            self.pitch_var.set(self.config["pitch"])
            self.rate_var.set(self.config["rate"])
            self.sample_rate_var.set(self.config["sample_rate"])
            self.threads_var.set(self.config["threads"])
            self.use_ssml_var.set(self.config["use_ssml"])
            
            # Загрузка текстов
            if "text" in config_data:
                self.text_editor.delete("1.0", tk.END)
                self.text_editor.insert("1.0", config_data["text"])
            
            if "ssml" in config_data:
                self.ssml_editor.delete("1.0", tk.END)
                self.ssml_editor.insert("1.0", config_data["ssml"])
            
            # Обновление threads
            if self.model:
                torch.set_num_threads(self.config["threads"])
            
            self.status_label.config(text=f"✅ Конфиг загружен: {filename}")
            messagebox.showinfo("Успех", f"Конфигурация загружена:\n{filename}")
            
        except Exception as e:
            messagebox.showerror("Ошибка", f"Ошибка загрузки конфига: {e}")
    
    def play_imperial_march(self):
        """Воспроизведение Имперского марша через SSML"""
        if not self.model:
            messagebox.showerror("Ошибка", "Модель не загружена! Нажмите '📥 Загрузить модель'")
            return
        
        try:
            self.status_label.config(text="🎵 Синтез Имперского марша...")
            self.root.update()
            
            # Генерируем SSML с текущими настройками rate
            # Pitch управляется внутри мелодии (x-low, low, medium, high, x-high)
            imperial_march_ssml = self.IMPERIAL_MARCH_TEMPLATE.format(
                base_rate=self.config["rate"]
            )
            
            # Показываем информацию (БЕЗ записи в редактор!)
            info_text = f"🎵 Воспроизведение с настройками:\n• Голос: {self.config['speaker']}\n• Base Rate: {self.config['rate']}"
            print(info_text)
            
            # Синтез
            start_time = time.time()
            
            audio = self.model.apply_tts(
                ssml_text=imperial_march_ssml,
                speaker=self.config["speaker"],
                sample_rate=self.config["sample_rate"]
            )
            
            synthesis_time = time.time() - start_time
            
            # Конвертация в numpy
            if isinstance(audio, torch.Tensor):
                audio_np = audio.numpy()
            else:
                audio_np = np.array(audio)
            
            # Статистика
            audio_duration = len(audio_np) / self.config["sample_rate"]
            rtf = synthesis_time / audio_duration
            
            self.last_synthesis_time = synthesis_time
            self.last_audio_duration = audio_duration
            self.last_rtf = rtf
            
            # Обновление статистики
            self.update_stats()
            
            # Воспроизведение
            self.status_label.config(text="🎵 Воспроизведение Имперского марша...")
            self.root.update()
            
            sd.play(audio_np, self.config["sample_rate"])
            sd.wait()
            
            self.status_label.config(text="✅ Имперский марш завершён! May the Force be with you!")
            
            # Показываем SSML в окне (опционально - если пользователь хочет посмотреть)
            show_ssml = messagebox.askyesno("🎵 Имперский марш", 
                              f"Воспроизведение завершено!\n\n"
                              f"Настройки:\n"
                              f"• Голос: {self.VOICES[self.config['speaker']]}\n"
                              f"• Base Rate: {self.config['rate']}\n"
                              f"• Pitch: фиксированный (x-low → x-high в мелодии)\n\n"
                              f"Производительность:\n"
                              f"• Синтез: {synthesis_time:.2f}s\n"
                              f"• Аудио: {audio_duration:.2f}s\n"
                              f"• RTF: {rtf:.2f} ({'⚡ Быстрее realtime!' if rtf < 1.0 else '⏱️'})\n\n"
                              f"Показать SSML код в редакторе?")
            
            if show_ssml:
                # Только если пользователь хочет - показываем SSML
                self.ssml_editor.delete("1.0", tk.END)
                self.ssml_editor.insert("1.0", imperial_march_ssml)
            
        except Exception as e:
            error_msg = f"Ошибка воспроизведения: {e}"
            self.status_label.config(text=f"❌ {error_msg}")
            messagebox.showerror("Ошибка", error_msg)
    
    def update_stats(self):
        """Обновление статистики производительности"""
        rtf_color = "green" if self.last_rtf < 1.0 else "orange"
        rtf_status = "⚡ Быстрее realtime!" if self.last_rtf < 1.0 else "⏱️ Медленнее realtime"
        
        stats_text = (
            f"Синтез: {self.last_synthesis_time:.3f}s | "
            f"Аудио: {self.last_audio_duration:.3f}s | "
            f"RTF: {self.last_rtf:.3f} {rtf_status}\n"
            f"Скорость: {1/self.last_rtf:.1f}x realtime | "
            f"Голос: {self.VOICES[self.config['speaker']]} | "
            f"Sample Rate: {self.config['sample_rate']} Hz"
        )
        
        self.stats_label.config(text=stats_text)


def main():
    """Главная функция"""
    root = tk.Tk()
    app = SileroTTSGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
