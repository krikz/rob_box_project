/**
 * Пример клиента для отправки команд через Zenoh REST API
 * 
 * ВАЖНО: REST API принимает CDR (Common Data Representation) бинарный формат,
 * используемый в DDS/ROS, а НЕ JSON!
 * 
 * Этот файл показывает правильную сериализацию Twist сообщений.
 */

// Установка зависимостей:
// npm install @foxglove/cdr

import { CdrWriter } from '@foxglove/cdr';

/**
 * Структура Twist сообщения (geometry_msgs/msg/Twist)
 */
interface Vector3 {
  x: number;
  y: number;
  z: number;
}

interface Twist {
  linear: Vector3;
  angular: Vector3;
}

/**
 * Сериализует Twist сообщение в CDR формат
 * 
 * @param linear - Линейная скорость (м/с)
 * @param angular - Угловая скорость (рад/с)
 * @returns CDR бинарные данные
 */
export function serializeTwist(linear: number, angular: number): Uint8Array {
  const twist: Twist = {
    linear: { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular },
  };

  // Создаём CDR writer
  const writer = new CdrWriter({ kind: 'CDR_LE' }); // Little Endian

  // Сериализуем Twist (6 x float64)
  writer.float64(twist.linear.x);
  writer.float64(twist.linear.y);
  writer.float64(twist.linear.z);
  writer.float64(twist.angular.x);
  writer.float64(twist.angular.y);
  writer.float64(twist.angular.z);

  return new Uint8Array(writer.data);
}

/**
 * Отправляет команду движения роботу через Zenoh REST API
 * 
 * @param robotId - ID робота (например, 'RBXU100001')
 * @param linear - Линейная скорость (м/с)
 * @param angular - Угловая скорость (рад/с)
 * @param topic - Топик для команды (по умолчанию 'cmd_vel_voice')
 */
export async function sendRobotCommand(
  robotId: string,
  linear: number,
  angular: number,
  topic: string = 'cmd_vel_voice'
): Promise<void> {
  // Сериализуем Twist в CDR
  const cdrData = serializeTwist(linear, angular);

  // URL с namespace для выбора робота
  const url = `http://zenoh.robbox.online/robots/${robotId}/${topic}`;

  // Отправляем PUT запрос с бинарными CDR данными
  const response = await fetch(url, {
    method: 'PUT',
    headers: {
      'Content-Type': 'application/octet-stream',
    },
    body: cdrData,
  });

  if (!response.ok) {
    throw new Error(`Failed to send command: ${response.status} ${response.statusText}`);
  }
}

/**
 * Примеры использования
 */
async function examples() {
  // Пример 1: Движение вперёд
  await sendRobotCommand('RBXU100001', 0.1, 0.0);

  // Пример 2: Поворот на месте
  await sendRobotCommand('RBXU100001', 0.0, 0.5);

  // Пример 3: Движение по дуге
  await sendRobotCommand('RBXU100001', 0.1, 0.2);

  // Пример 4: Остановка
  await sendRobotCommand('RBXU100001', 0.0, 0.0);

  // Пример 5: Команда другому роботу
  await sendRobotCommand('RBXU100002', 0.1, 0.0);

  // Пример 6: Использование другого топика
  await sendRobotCommand('RBXU100001', 0.1, 0.0, 'cmd_vel_web');
}
