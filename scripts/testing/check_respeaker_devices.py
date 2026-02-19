#!/usr/bin/env python3
"""Проверка PyAudio устройств ReSpeaker"""
import pyaudio

pa = pyaudio.PyAudio()
print(f"\n=== Всего устройств: {pa.get_device_count()} ===\n")

for i in range(pa.get_device_count()):
    info = pa.get_device_info_by_index(i)
    if 'respeaker' in info['name'].lower() or 'seeed' in info['name'].lower():
        print(f"ID {i}: {info['name']}")
        print(f"  Входов: {info['maxInputChannels']}")
        print(f"  Выходов: {info['maxOutputChannels']}")
        print(f"  Sample Rate: {info['defaultSampleRate']}")
        print()
