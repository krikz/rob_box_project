import serial
import matplotlib.pyplot as plt
import numpy as np
import struct
import time

# Настройки последовательного порта
SERIAL_PORT = 'COM14'  # Порт для N10
BAUD_RATE = 230400     # Скорость для N10

# Создаем объект для работы с последовательным портом
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Порт {SERIAL_PORT} открыт успешно")
except Exception as e:
    print(f"Ошибка при открытии порта: {e}")
    exit()

# Параметры для N10
PACKET_SIZE = 58        # Размер пакета для N10
POINTS_PER_PACKET = 16  # Точек в пакете
DATA_START = 7          # Начало данных
ANGLE_START = 5         # Позиция угла
END_ANGLE_POS = 55      # Позиция конечного угла
CRC_POS = PACKET_SIZE - 1

# Инициализация графика
plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_ylim(0, 20)  # Максимальная дистанция для отображения
theta = np.linspace(0.0, 2 * np.pi, 360)
line, = ax.plot([], [], lw=2)

distances = np.zeros(360)

def calculate_crc(data):
    """Расчет CRC8 для N10"""
    crc = 0
    for byte in data[:-1]:
        crc += byte
    return crc & 0xFF

def parse_packet(packet):
    """Функция для парсинга пакета данных согласно структуре N10"""
    if len(packet) != PACKET_SIZE:
        return [], []
    
    # Проверка CRC
    calculated_crc = calculate_crc(packet)
    if calculated_crc != packet[CRC_POS]:
        print(f"CRC mismatch: expected {packet[CRC_POS]}, got {calculated_crc}")
        return [], []
    
    distances = []
    angles = []
    
    # Получаем начальный угол
    start_angle_raw = (packet[ANGLE_START] << 8) | packet[ANGLE_START+1]
    start_angle = start_angle_raw / 100.0
    
    # Получаем конечный угол
    end_angle_raw = (packet[END_ANGLE_POS] << 8) | packet[END_ANGLE_POS+1]
    end_angle = end_angle_raw / 100.0
    
    angle_step = (end_angle - start_angle) / POINTS_PER_PACKET
    
    for i in range(POINTS_PER_PACKET):
        offset = DATA_START + i*3
        distance_raw = (packet[offset] << 8) | packet[offset+1]
        intensity = packet[offset+2]
        
        if distance_raw == 0xFFFF:  # Недействительное значение
            continue
        
        # Преобразуем в метры
        distance_m = distance_raw / 1000.0
        
        # Вычисляем угол
        angle = start_angle + (i * angle_step)
        angle = angle % 360
        
        distances.append(distance_m)
        angles.append(angle)
    
    return angles, distances

try:
    print("Ожидание данных от лидара...")
    buffer = bytearray()
    while True:
        # Чтение данных из последовательного порта
        raw_data = ser.read(1024)
        if not raw_data:
            time.sleep(0.001)
            continue
        
        buffer.extend(raw_data)
        
        while len(buffer) >= PACKET_SIZE:
            # Ищем начало пакета
            start_idx = buffer.find(b'\xA5\x5A')
            if start_idx == -1:
                # Если не нашли заголовок, отбрасываем первый байт
                buffer.pop(0)
                continue
            
            # Извлекаем пакет
            full_packet = buffer[start_idx:start_idx+PACKET_SIZE]
            del buffer[:start_idx+2]  # Удаляем заголовок и начало пакета
            
            if len(full_packet) < PACKET_SIZE:
                # Если пакет неполный, ждем больше данных
                break
            
            # Парсим пакет
            angles, dists = parse_packet(full_packet)
            
            if not angles or not dists:
                print("Пакет не содержит валидных данных")
                continue
            
            # Обновление массива расстояний
            for angle, dist in zip(angles, dists):
                angle_int = int(round(angle))
                if 0 <= angle_int < 360:
                    distances[angle_int] = dist
            
            # Обновление графика
            line.set_xdata(np.deg2rad(range(360)))
            line.set_ydata(distances)
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.001)

except KeyboardInterrupt:
    print("\nПрервано пользователем")

finally:
    ser.close()
    print("Порт закрыт")