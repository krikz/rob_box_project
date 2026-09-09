#!/usr/bin/env python3
"""Проверка конфигурации AEC на ReSpeaker"""

import usb.core
import struct

# Найти ReSpeaker
dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
if dev is None:
    print("❌ ReSpeaker не найден!")
    exit(1)

try:
    product = usb.util.get_string(dev, dev.iProduct) if dev.iProduct else "Unknown"
    print(f"✅ ReSpeaker найден: {product}")
except:
    print("✅ ReSpeaker найден (0x2886:0x0018)")

# Параметры AEC
params = {
    "AECFREEZEONOFF": (18, 7),  # 0=adaptive, 1=frozen
    "ECHOONOFF": (18, 6),        # 1=enabled, 0=disabled
    "NLATTENONOFF": (18, 1),     # 1=enabled, 0=disabled
}

print("\n📊 Текущая конфигурация AEC:")
print("-" * 50)

for name, (param_id, offset) in params.items():
    try:
        cmd = 0x80 | offset | 0x40  # Read int
        response = dev.ctrl_transfer(0xC0, 0, cmd, param_id, 8, timeout=1000)
        value = struct.unpack(b"ii", response.tobytes())[0]
        
        if name == "AECFREEZEONOFF":
            status = "✅ Adaptive" if value == 0 else "⚠️ Frozen" if value == 1 else f"? {value}"
        else:
            status = "✅ Enabled" if value == 1 else "❌ Disabled" if value == 0 else f"? {value}"
        
        print(f"{name:20s}: {status:15s} (value={value})")
    except Exception as e:
        print(f"{name:20s}: ❌ Ошибка - {e}")

print("-" * 50)
print("\n💡 Рекомендуемые значения для STT:")
print("  AECFREEZEONOFF = 0 (Adaptive)")
print("  ECHOONOFF = 1 (Enabled)")
print("  NLATTENONOFF = 1 (Enabled)")
