#!/usr/bin/env python3
"""
Диагностический инструмент для анализа одометрии VESC Nexus.

Проверяет соответствие между:
- ERPM (Electrical RPM) от VESC
- Физическим вращением колеса  
- Рассчитанной линейной скоростью
- Перемещением одометрии

Использование:
    python3 tools/odometry_diagnostic.py --wheel-radius 0.115 --wheel-poles 30
"""

import argparse
import math


class OdometryDiagnostic:
    """Диагностика расчетов одометрии VESC Nexus."""

    def __init__(self, wheel_radius: float, wheel_poles: int):
        """
        Инициализация диагностики.

        Args:
            wheel_radius: Радиус колеса в метрах
            wheel_poles: Количество полюсов мотора (магнитных пар × 2)
        """
        self.wheel_radius = wheel_radius
        self.wheel_poles = wheel_poles
        self.wheel_circumference = 2.0 * math.pi * wheel_radius

    def erpm_to_mechanical_rpm(self, erpm: float) -> float:
        """
        Конвертация ERPM (Electrical RPM) в механические обороты колеса.

        Формула согласно документации VESC:
        ERPM = Mechanical_RPM × motor_poles
        Поэтому: Mechanical_RPM = ERPM / motor_poles

        Args:
            erpm: Electrical RPM от VESC

        Returns:
            Механические обороты колеса в минуту (RPM)
        """
        return erpm / float(self.wheel_poles)

    def rpm_to_linear_velocity(self, rpm: float) -> float:
        """
        Конвертация механических оборотов в линейную скорость.

        Формула: v = (RPM / 60) × circumference
        где circumference = 2πr

        Args:
            rpm: Механические обороты колеса (RPM)

        Returns:
            Линейная скорость в м/с
        """
        rps = rpm / 60.0  # Обороты в секунду
        return rps * self.wheel_circumference

    def erpm_to_velocity(self, erpm: float) -> float:
        """
        Прямая конвертация ERPM → линейная скорость.

        Args:
            erpm: Electrical RPM от VESC

        Returns:
            Линейная скорость в м/с
        """
        rpm = self.erpm_to_mechanical_rpm(erpm)
        return self.rpm_to_linear_velocity(rpm)

    def velocity_to_distance(self, velocity: float, time_delta: float) -> float:
        """
        Расчет перемещения за время.

        Args:
            velocity: Линейная скорость в м/с
            time_delta: Время в секундах

        Returns:
            Перемещение в метрах
        """
        return velocity * time_delta

    def analyze_erpm_sample(self, erpm: float, time_delta: float = 0.02) -> dict:
        """
        Полный анализ одного значения ERPM.

        Args:
            erpm: Electrical RPM от VESC
            time_delta: Период обновления (по умолчанию 50 Hz = 0.02 сек)

        Returns:
            Словарь с результатами анализа
        """
        mechanical_rpm = self.erpm_to_mechanical_rpm(erpm)
        linear_velocity = self.rpm_to_linear_velocity(mechanical_rpm)
        distance = self.velocity_to_distance(linear_velocity, time_delta)

        return {
            "erpm": erpm,
            "mechanical_rpm": mechanical_rpm,
            "linear_velocity_ms": linear_velocity,
            "linear_velocity_kmh": linear_velocity * 3.6,
            "distance_per_update_m": distance,
            "distance_per_update_mm": distance * 1000.0,
            "time_delta_s": time_delta,
        }

    def print_analysis(self, analysis: dict):
        """Вывод результатов анализа в читаемом формате."""
        print(f"╔══════════════════════════════════════════════════════════╗")
        print(f"║  АНАЛИЗ ОДОМЕТРИИ VESC NEXUS                             ║")
        print(f"╠══════════════════════════════════════════════════════════╣")
        print(f"║  Параметры:                                              ║")
        print(f"║    Радиус колеса:  {self.wheel_radius:.3f} м                          ║")
        print(f"║    Полюсов мотора: {self.wheel_poles}                                 ║")
        print(f"║    Длина окружности: {self.wheel_circumference:.4f} м                 ║")
        print(f"╠══════════════════════════════════════════════════════════╣")
        print(f"║  Входные данные:                                         ║")
        print(f"║    ERPM:           {analysis['erpm']:>10.1f}                       ║")
        print(f"║    Период (Δt):    {analysis['time_delta_s']:.3f} сек                       ║")
        print(f"╠══════════════════════════════════════════════════════════╣")
        print(f"║  Расчётные значения:                                     ║")
        print(f"║    Механич. RPM:   {analysis['mechanical_rpm']:>10.2f}                       ║")
        print(f"║    Скорость:       {analysis['linear_velocity_ms']:>10.4f} м/с               ║")
        print(f"║    Скорость:       {analysis['linear_velocity_kmh']:>10.4f} км/ч             ║")
        print(f"╠══════════════════════════════════════════════════════════╣")
        print(f"║  Перемещение за {analysis['time_delta_s']:.3f} сек:                          ║")
        print(f"║    {analysis['distance_per_update_m']:>10.6f} м = {analysis['distance_per_update_mm']:>8.3f} мм         ║")
        print(f"╚══════════════════════════════════════════════════════════╝")

    def compare_with_expected(
        self, erpm: float, expected_distance: float, time_delta: float = 0.02
    ):
        """
        Сравнение расчётного и ожидаемого перемещения.

        Args:
            erpm: Electrical RPM от VESC
            expected_distance: Ожидаемое перемещение в метрах
            time_delta: Период обновления в секундах
        """
        analysis = self.analyze_erpm_sample(erpm, time_delta)
        calculated_distance = analysis["distance_per_update_m"]

        error_m = calculated_distance - expected_distance
        error_percent = (error_m / expected_distance * 100.0) if expected_distance != 0 else 0

        print(f"\n╔══════════════════════════════════════════════════════════╗")
        print(f"║  СРАВНЕНИЕ С ОЖИДАЕМЫМ ЗНАЧЕНИЕМ                         ║")
        print(f"╠══════════════════════════════════════════════════════════╣")
        print(f"║  Ожидаемое перемещение: {expected_distance:>10.6f} м              ║")
        print(f"║  Расчётное перемещение: {calculated_distance:>10.6f} м              ║")
        print(f"║  Ошибка:                {error_m:>10.6f} м              ║")
        print(f"║  Ошибка:                {error_percent:>10.2f} %               ║")
        print(f"╚══════════════════════════════════════════════════════════╝")

        if abs(error_percent) > 10:
            print("\n⚠️  ВНИМАНИЕ: Ошибка превышает 10%!")
            print("    Возможные причины:")
            print("    1. Неправильный wheel_radius")
            print("    2. Неправильный wheel_poles")
            print("    3. Проскальзывание колеса")

    def suggest_correction(
        self, erpm: float, actual_distance: float, time_delta: float = 0.02
    ):
        """
        Предложить исправленные параметры на основе реального перемещения.

        Args:
            erpm: Измеренный ERPM от VESC
            actual_distance: Реальное измеренное перемещение в метрах
            time_delta: Период обновления в секундах
        """
        # Текущий расчёт
        current_analysis = self.analyze_erpm_sample(erpm, time_delta)
        current_distance = current_analysis["distance_per_update_m"]

        # Корректировочный коэффициент
        correction_factor = actual_distance / current_distance

        # Исправленный радиус (если poles правильные)
        corrected_radius = self.wheel_radius * correction_factor

        print(f"\n╔══════════════════════════════════════════════════════════╗")
        print(f"║  РЕКОМЕНДАЦИИ ПО ИСПРАВЛЕНИЮ                             ║")
        print(f"╠══════════════════════════════════════════════════════════╣")
        print(f"║  Текущие параметры:                                      ║")
        print(f"║    wheel_radius: {self.wheel_radius:.4f} м                           ║")
        print(f"║    wheel_poles:  {self.wheel_poles}                                  ║")
        print(f"╠══════════════════════════════════════════════════════════╣")
        print(f"║  Измерения:                                              ║")
        print(f"║    ERPM:               {erpm:>10.1f}                       ║")
        print(f"║    Реальное расстояние: {actual_distance:>10.6f} м              ║")
        print(f"║    Расчётное расстояние: {current_distance:>10.6f} м              ║")
        print(f"║    Коэффициент ошибки:  {correction_factor:>10.6f}                 ║")
        print(f"╠══════════════════════════════════════════════════════════╣")
        print(f"║  РЕКОМЕНДУЕМЫЕ ПАРАМЕТРЫ:                                ║")
        print(f"║    wheel_radius: {corrected_radius:.4f} м (если poles={self.wheel_poles})      ║")
        print(f"╚══════════════════════════════════════════════════════════╝")

        print("\n💡 СОВЕТ:")
        print("   1. Измерьте физический радиус колеса линейкой")
        print("   2. Проверьте количество полюсов мотора в спецификации")
        print("   3. Обновите значения в:")
        print("      - docker/main/config/vesc_nexus/vesc_config.yaml")
        print("      - docker/main/config/vesc_nexus/robot_controller.yaml")


def run_diagnostic_scenarios():
    """Запуск типовых сценариев диагностики."""
    # Параметры по умолчанию из конфигурации
    radius = 0.115  # м
    poles = 30

    diag = OdometryDiagnostic(wheel_radius=radius, wheel_poles=poles)

    print("═" * 60)
    print("СЦЕНАРИЙ 1: Небольшое вращение колеса")
    print("═" * 60)
    # Небольшой ERPM (например, 100)
    analysis = diag.analyze_erpm_sample(erpm=100, time_delta=0.02)
    diag.print_analysis(analysis)

    print("\n" + "═" * 60)
    print("СЦЕНАРИЙ 2: Среднее вращение")
    print("═" * 60)
    analysis = diag.analyze_erpm_sample(erpm=1000, time_delta=0.02)
    diag.print_analysis(analysis)

    print("\n" + "═" * 60)
    print("СЦЕНАРИЙ 3: Высокая скорость")
    print("═" * 60)
    analysis = diag.analyze_erpm_sample(erpm=5000, time_delta=0.02)
    diag.print_analysis(analysis)

    print("\n" + "═" * 60)
    print("СЦЕНАРИЙ 4: Проверка формул")
    print("═" * 60)
    print(f"\nФормула ERPM → Механические обороты:")
    print(f"  RPM = ERPM / motor_poles")
    print(f"  RPM = ERPM / {poles}")
    print(f"\nФормула RPM → Линейная скорость:")
    print(f"  v = (RPM / 60) × 2πr")
    print(f"  v = (RPM / 60) × {2*math.pi*radius:.4f} м/с")
    print(f"\nПолная формула ERPM → скорость:")
    print(f"  v = (ERPM / {poles}) / 60 × {2*math.pi*radius:.4f}")
    print(f"  v = ERPM × {(2*math.pi*radius)/(60*poles):.8f}")


def main():
    parser = argparse.ArgumentParser(
        description="Диагностика одометрии VESC Nexus",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Примеры использования:

1. Запуск стандартных сценариев:
   python3 tools/odometry_diagnostic.py

2. Анализ конкретного ERPM:
   python3 tools/odometry_diagnostic.py --erpm 1500

3. Сравнение с ожидаемым перемещением:
   python3 tools/odometry_diagnostic.py --erpm 1000 --expected-distance 0.01

4. Расчёт корректировки параметров:
   python3 tools/odometry_diagnostic.py --erpm 1000 --actual-distance 0.008

5. Использование других параметров колеса:
   python3 tools/odometry_diagnostic.py --wheel-radius 0.127 --wheel-poles 28
        """,
    )

    parser.add_argument(
        "--wheel-radius",
        type=float,
        default=0.115,
        help="Радиус колеса в метрах (по умолчанию: 0.115)",
    )
    parser.add_argument(
        "--wheel-poles",
        type=int,
        default=30,
        help="Количество полюсов мотора (по умолчанию: 30)",
    )
    parser.add_argument(
        "--erpm",
        type=float,
        help="ERPM для анализа (если не указан, запускаются типовые сценарии)",
    )
    parser.add_argument(
        "--time-delta",
        type=float,
        default=0.02,
        help="Период обновления в секундах (по умолчанию: 0.02 = 50 Hz)",
    )
    parser.add_argument(
        "--expected-distance",
        type=float,
        help="Ожидаемое перемещение в метрах для сравнения",
    )
    parser.add_argument(
        "--actual-distance",
        type=float,
        help="Реальное измеренное перемещение в метрах для расчёта корректировки",
    )

    args = parser.parse_args()

    diag = OdometryDiagnostic(
        wheel_radius=args.wheel_radius, wheel_poles=args.wheel_poles
    )

    if args.erpm is None:
        # Запуск типовых сценариев
        run_diagnostic_scenarios()
    else:
        # Анализ конкретного ERPM
        analysis = diag.analyze_erpm_sample(args.erpm, args.time_delta)
        diag.print_analysis(analysis)

        if args.expected_distance is not None:
            diag.compare_with_expected(
                args.erpm, args.expected_distance, args.time_delta
            )

        if args.actual_distance is not None:
            diag.suggest_correction(args.erpm, args.actual_distance, args.time_delta)


if __name__ == "__main__":
    main()
