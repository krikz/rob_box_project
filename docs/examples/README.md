# Примеры клиентов для Zenoh REST API

Эта директория содержит примеры кода для взаимодействия с роботами через Zenoh REST API.

## ⚠️ Важно: Формат данных

**REST API принимает CDR (Common Data Representation) бинарный формат, НЕ JSON!**

CDR - это стандартный бинарный формат сериализации, используемый в DDS/ROS 2.

## 📄 Файлы

### zenoh_rest_client.ts

TypeScript/JavaScript клиент для отправки команд управления роботом.

**Основные функции:**

- `serializeTwist(linear, angular)` - сериализует Twist сообщение в CDR формат
- `sendRobotCommand(robotId, linear, angular, topic)` - отправляет команду роботу

**Использование:**

```typescript
import { sendRobotCommand } from './zenoh_rest_client';

// Движение вперёд со скоростью 0.1 м/с
await sendRobotCommand('RBXU100001', 0.1, 0.0);

// Поворот на месте со скоростью 0.5 рад/с
await sendRobotCommand('RBXU100001', 0.0, 0.5);

// Остановка
await sendRobotCommand('RBXU100001', 0.0, 0.0);
```

**Установка зависимостей:**

```bash
npm install @foxglove/cdr
```

## 🔧 Формат REST API запроса

### URL

```
http://zenoh.robbox.online/robots/{ROBOT_ID}/{topic}
```

Где:
- `{ROBOT_ID}` - идентификатор робота (например, `RBXU100001`)
- `{topic}` - ROS топик (например, `cmd_vel_voice`, `cmd_vel_web`)

### Заголовки

```
Content-Type: application/octet-stream
```

### Тело запроса

CDR бинарные данные сериализованного Twist сообщения (48 байт):

```
Структура Twist (geometry_msgs/msg/Twist):
  linear:
    x: float64 (8 байт)
    y: float64 (8 байт)
    z: float64 (8 байт)
  angular:
    x: float64 (8 байт)
    y: float64 (8 байт)
    z: float64 (8 байт)
```

### Пример curl (с готовым CDR файлом)

```bash
# Создать CDR файл с помощью ROS инструментов или веб-клиента
curl -X PUT http://zenoh.robbox.online/robots/RBXU100001/cmd_vel_voice \
  -H "Content-Type: application/octet-stream" \
  --data-binary @twist.cdr
```

## 🌐 Интеграция в веб-приложение

### React пример

```typescript
import { sendRobotCommand } from './zenoh_rest_client';

function RobotController({ robotId }: { robotId: string }) {
  const [linear, setLinear] = useState(0);
  const [angular, setAngular] = useState(0);

  const handleSend = async () => {
    try {
      await sendRobotCommand(robotId, linear, angular);
      console.log('Command sent');
    } catch (error) {
      console.error('Failed:', error);
    }
  };

  return (
    <div>
      <input 
        type="number" 
        value={linear} 
        onChange={(e) => setLinear(parseFloat(e.target.value))}
        placeholder="Linear (m/s)"
      />
      <input 
        type="number" 
        value={angular} 
        onChange={(e) => setAngular(parseFloat(e.target.value))}
        placeholder="Angular (rad/s)"
      />
      <button onClick={handleSend}>Send</button>
    </div>
  );
}
```

## 📚 Дополнительная информация

### CDR сериализация

CDR (Common Data Representation) - стандарт OMG для сериализации данных в распределённых системах.

**Ссылки:**
- [CDR Specification](https://www.omg.org/spec/DDSI-RTPS/2.3/PDF)
- [@foxglove/cdr](https://www.npmjs.com/package/@foxglove/cdr) - библиотека для TypeScript/JavaScript

### Zenoh REST API

**Документация:**
- [Zenoh REST plugin](https://github.com/eclipse-zenoh/zenoh/tree/main/plugins/zenoh-plugin-rest)
- [Zenoh DDS plugin](https://github.com/eclipse-zenoh/zenoh-plugin-dds)

### ROS типы сообщений

**geometry_msgs/msg/Twist:**
- [ROS 2 документация](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- Используется для команд управления мобильными роботами

## 🐛 Устранение неполадок

### Ошибка: "Failed to send command: 400"

**Причина:** Неправильный формат CDR данных

**Решение:** Убедитесь что используете правильную сериализацию:
- Little Endian (CDR_LE)
- 6 x float64 (48 байт)
- Порядок: linear.x, linear.y, linear.z, angular.x, angular.y, angular.z

### Команда не доходит до робота

**Проверьте:**
1. Правильный ROBOT_ID в URL
2. DDS плагин включён на облачном роутере
3. Робот подключён к облаку (проверить топики: `https://zenoh.robbox.online/robots/RBXU100001/**`)

---

**Последнее обновление:** 10 ноября 2025
