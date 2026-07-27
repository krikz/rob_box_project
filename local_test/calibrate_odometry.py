#!/usr/bin/env python3
"""
Калибровка одометрии Rob Box — тест перемещения 0.5 м

Скрипт:
1. Записывает начальную позицию /odom
2. Посылает cmd_vel для перемещения на ~0.5 м (0.1 м/с × 5 сек)
3. Записывает конечную позицию /odom
4. Выводит расстояние по одометрии
5. Спрашивает реальное расстояние (линейка)
6. Рассчитывает поправочный коэффициент для wheel_radius

Использование:
    # Линейный тест (0.5 м вперёд)
    python3 local_test/calibrate_odometry.py --linear

    # Угловой тест (360° на месте)
    python3 local_test/calibrate_odometry.py --angular

    # Указать скорость и дистанцию
    python3 local_test/calibrate_odometry.py --linear --speed 0.1 --distance 0.5

Перед запуском:
    source /opt/ros/lyrical/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_SESSION_CONFIG_URI=$(pwd)/local_test/zenoh_local_session.json5
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class OdometryCalibrator(Node):
    """Нода для калибровки одометрии."""

    def __init__(self, test_type: str, speed: float, distance: float):
        super().__init__("odometry_calibrator")

        self.test_type = test_type
        self.speed = speed
        self.target_distance = distance

        # State
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.test_running = False
        self.test_done = False

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.get_logger().info("Калибратор одометрии запущен")
        self.get_logger().info(f"Тест: {test_type}, скорость: {speed}, целевая дистанция: {distance}")
        self.get_logger().info("Ожидаю данные /odom...")

    def odom_callback(self, msg: Odometry):
        """Обработка одометрии."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Quaternion → yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f"✅ /odom получен: x={self.current_x:.4f}, y={self.current_y:.4f}, "
                f"yaw={math.degrees(self.current_yaw):.1f}°"
            )

    def wait_for_odom(self, timeout: float = 10.0) -> bool:
        """Ждём первое сообщение /odom."""
        start = time.time()
        while not self.odom_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self.odom_received

    def record_start(self):
        """Записать стартовую позицию."""
        rclpy.spin_once(self, timeout_sec=0.5)
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw
        self.get_logger().info(
            f"📍 Старт: x={self.start_x:.4f}, y={self.start_y:.4f}, "
            f"yaw={math.degrees(self.start_yaw):.1f}°"
        )

    def send_velocity(self, linear_x: float, angular_z: float, duration: float):
        """Отправить cmd_vel на заданное время."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        rate = 10  # Hz
        count = int(duration * rate)
        self.get_logger().info(
            f"🚗 Отправка cmd_vel: linear={linear_x:.2f} m/s, angular={angular_z:.2f} rad/s, "
            f"длительность={duration:.1f}с ({count} сообщений)"
        )

        for i in range(count):
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(1.0 / rate)

        # Остановка
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            time.sleep(0.1)

        # Дать роботу остановиться и одометрии обновиться
        self.get_logger().info("⏸️  Ожидание остановки (2 сек)...")
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)

    def calculate_results(self):
        """Рассчитать результаты."""
        rclpy.spin_once(self, timeout_sec=0.5)

        if self.test_type == "linear":
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            odom_distance = math.sqrt(dx * dx + dy * dy)

            self.get_logger().info("=" * 60)
            self.get_logger().info("📊 РЕЗУЛЬТАТЫ ЛИНЕЙНОГО ТЕСТА")
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"  Начальная позиция: x={self.start_x:.4f}, y={self.start_y:.4f}")
            self.get_logger().info(f"  Конечная позиция:  x={self.current_x:.4f}, y={self.current_y:.4f}")
            self.get_logger().info(f"  Расстояние по одометрии: {odom_distance:.4f} м")
            self.get_logger().info(f"  Целевое расстояние:      {self.target_distance:.4f} м")
            self.get_logger().info("")

            return odom_distance

        elif self.test_type == "angular":
            dyaw = self.current_yaw - self.start_yaw
            # Нормализация в -π..π
            while dyaw > math.pi:
                dyaw -= 2 * math.pi
            while dyaw < -math.pi:
                dyaw += 2 * math.pi

            odom_degrees = math.degrees(dyaw)
            total_yaw_deg = math.degrees(self.current_yaw)

            self.get_logger().info("=" * 60)
            self.get_logger().info("📊 РЕЗУЛЬТАТЫ УГЛОВОГО ТЕСТА")
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"  Начальный yaw: {math.degrees(self.start_yaw):.1f}°")
            self.get_logger().info(f"  Конечный yaw:  {total_yaw_deg:.1f}°")
            self.get_logger().info(f"  Поворот по одометрии: {odom_degrees:.1f}°")
            self.get_logger().info(f"  Целевой поворот:      360.0°")
            self.get_logger().info("")

            return odom_degrees

    def prompt_correction(self, odom_value: float):
        """Запросить реальное измерение и рассчитать коррекцию."""
        if self.test_type == "linear":
            print("\n" + "=" * 60)
            print("📏 ИЗМЕРЕНИЕ ЛИНЕЙКОЙ")
            print("=" * 60)
            print(f"  Одометрия показала: {odom_value:.4f} м")
            print()

            try:
                real_str = input("  Введите реальное расстояние (м), например 0.48: ")
                real_distance = float(real_str)
            except (ValueError, EOFError):
                print("  ❌ Неверный ввод")
                return

            # Текущий wheel_radius
            current_radius = 0.1143  # из конфига

            if abs(odom_value) < 0.001:
                print("  ❌ Одометрия не показала движение!")
                return

            ratio = real_distance / odom_value
            new_radius = current_radius * ratio
            error_pct = abs(1.0 - ratio) * 100

            print()
            print("=" * 60)
            print("🔧 РЕКОМЕНДАЦИИ")
            print("=" * 60)
            print(f"  Коэффициент коррекции: {ratio:.4f}")
            print(f"  Ошибка одометрии:      {error_pct:.1f}%")
            print(f"  Текущий wheel_radius:  {current_radius}")
            print(f"  Новый wheel_radius:    {new_radius:.6f}")
            print()

            if error_pct > 300:
                gear_est = odom_value / real_distance
                print(f"  ⚠️  Ошибка >300%! Возможно gear_ratio не учтён.")
                print(f"  ⚠️  Оценка gear_ratio: {gear_est:.1f}")
                print(f"  📝  Добавьте gear_ratio: {gear_est:.1f} в vesc_config.yaml")
            elif error_pct > 10:
                print(f"  ⚠️  Ошибка >10%. Нужна корректировка wheel_radius")
                print(f"  📝  Обновите два файла:")
                print(f"       docker/main/config/vesc_nexus/robot_controller.yaml")
                print(f"       src/rob_box_description/urdf/rob_box_ros2_control.xacro")
            else:
                print(f"  ✅ Ошибка <10%. Одометрия в пределах нормы!")

        elif self.test_type == "angular":
            print("\n" + "=" * 60)
            print("📐 ВИЗУАЛЬНОЕ ИЗМЕРЕНИЕ")
            print("=" * 60)
            print(f"  Одометрия показала поворот: {odom_value:.1f}°")
            print()

            try:
                real_str = input("  Введите реальный угол поворота (градусы), например 350: ")
                real_degrees = float(real_str)
            except (ValueError, EOFError):
                print("  ❌ Неверный ввод")
                return

            current_separation = 0.380

            if abs(odom_value) < 0.1:
                print("  ❌ Одометрия не показала поворота!")
                return

            ratio = real_degrees / odom_value
            new_separation = current_separation * ratio
            error_pct = abs(1.0 - ratio) * 100

            print()
            print("=" * 60)
            print("🔧 РЕКОМЕНДАЦИИ")
            print("=" * 60)
            print(f"  Коэффициент коррекции:    {ratio:.4f}")
            print(f"  Ошибка одометрии:         {error_pct:.1f}%")
            print(f"  Текущий wheel_separation: {current_separation}")
            print(f"  Новый wheel_separation:   {new_separation:.6f}")
            print()

            if error_pct > 10:
                print(f"  ⚠️  Нужна корректировка wheel_separation")
                print(f"  📝  Обновите: docker/main/config/vesc_nexus/robot_controller.yaml")
            else:
                print(f"  ✅ Ошибка <10%. Wheel separation в пределах нормы!")


def main():
    parser = argparse.ArgumentParser(description="Калибровка одометрии Rob Box")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--linear", action="store_true", help="Линейный тест (0.5 м вперёд)")
    group.add_argument("--angular", action="store_true", help="Угловой тест (360° на месте)")
    parser.add_argument("--speed", type=float, default=0.1, help="Скорость м/с или рад/с (default: 0.1)")
    parser.add_argument("--distance", type=float, default=0.5, help="Целевая дистанция в м (default: 0.5)")
    parser.add_argument("--duration", type=float, default=None, help="Длительность в секундах (auto-calculated)")
    args = parser.parse_args()

    test_type = "linear" if args.linear else "angular"

    if args.angular:
        args.distance = 360.0  # degrees
        if args.speed == 0.1:
            args.speed = 0.5  # rad/s по умолчанию для поворота

    # Расчёт длительности
    if args.duration is not None:
        duration = args.duration
    elif test_type == "linear":
        duration = args.distance / args.speed
    else:
        duration = (2 * math.pi) / args.speed  # Полный оборот

    rclpy.init()
    node = OdometryCalibrator(test_type, args.speed, args.distance)

    try:
        # 1. Ждём одометрию
        if not node.wait_for_odom(timeout=15.0):
            node.get_logger().error("❌ Таймаут: /odom не получен за 15 сек")
            node.get_logger().error("Проверьте:")
            node.get_logger().error("  - Zenoh роутер запущен (rmw_zenohd)")
            node.get_logger().error("  - ZENOH_SESSION_CONFIG_URI установлен")
            node.get_logger().error("  - Робот включён и vesc_nexus работает")
            return 1

        # 2. Подтверждение
        print("\n" + "=" * 60)
        if test_type == "linear":
            print(f"🚗 ЛИНЕЙНЫЙ ТЕСТ: робот проедет ~{args.distance} м вперёд")
            print(f"   Скорость: {args.speed} м/с, Время: {duration:.1f} сек")
        else:
            print(f"🔄 УГЛОВОЙ ТЕСТ: робот повернётся на ~360°")
            print(f"   Скорость: {args.speed} рад/с, Время: {duration:.1f} сек")
        print("=" * 60)
        print()

        input("⏩ Нажмите Enter для старта (Ctrl+C для отмены)...")

        # 3. Записать старт
        node.record_start()

        # 4. Двигаться
        if test_type == "linear":
            node.send_velocity(linear_x=args.speed, angular_z=0.0, duration=duration)
        else:
            node.send_velocity(linear_x=0.0, angular_z=args.speed, duration=duration)

        # 5. Результаты
        odom_value = node.calculate_results()

        # 6. Коррекция
        node.prompt_correction(odom_value)

        print("\n✅ Тест завершён. Повторите 3 раза для точности.\n")
        return 0

    except KeyboardInterrupt:
        node.get_logger().info("Прервано пользователем")
        # Остановить робота
        stop = Twist()
        for _ in range(10):
            node.cmd_pub.publish(stop)
            time.sleep(0.1)
        return 130

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
