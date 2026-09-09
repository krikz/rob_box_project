#!/usr/bin/env python3
"""
Тест чтения VAD напрямую через USB БЕЗ PyAudio
"""

import usb.core
import usb.util
import struct
import time

VENDOR_ID = 0x2886
PRODUCT_ID = 0x0018
TIMEOUT = 100000

# Найти устройство
dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
if dev is None:
    print("❌ ReSpeaker не найден!")
    exit(1)

print(f"✓ Найдено: {dev}")

# Reset
try:
    dev.reset()
    print("✓ USB reset выполнен")
except usb.core.USBError as e:
    print(f"⚠ USB reset warning: {e}")

time.sleep(5)
print("✓ После sleep(5)")

# VOICEACTIVITY = (19, 32, 'int', 1, 0, 'ro')
param_id = 19
offset = 32
cmd = 0x80 | offset | 0x40  # int type
length = 8

print("\n🎙️ Читаю VAD каждую секунду (Ctrl+C для выхода)...")
print("="*50)

try:
    while True:
        try:
            response = dev.ctrl_transfer(
                usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
                0, cmd, param_id, length, TIMEOUT)
            
            data = struct.unpack(b'ii', response.tobytes())
            vad = data[0]
            
            if vad:
                print(f"🎙️ VAD: {vad} - SPEECH DETECTED!")
            else:
                print(f"   VAD: {vad} - silence")
                
        except usb.core.USBError as e:
            print(f"❌ USB Error: {e}")
            break
            
        time.sleep(1)
        
except KeyboardInterrupt:
    print("\n✓ Stopped")
