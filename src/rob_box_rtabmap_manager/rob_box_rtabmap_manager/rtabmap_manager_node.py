#!/usr/bin/env python3
"""
RTABMap Manager Node - управление БД RTABMap

Предоставляет ROS сервисы для:
- Полного удаления БД (пересоздание с нуля)
- Создания backup перед удалением

ВАЖНО: Запускается ВНУТРИ контейнера rtabmap (Main Pi)
Имеет прямой доступ к /maps/rtabmap.db
"""

import os
import shutil
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class RTABMapManagerNode(Node):
    """ROS2 нода для управления RTABMap базой данных"""

    def __init__(self):
        super().__init__("rtabmap_manager")

        # Параметры
        self.declare_parameter("db_path", "/maps/rtabmap.db")
        self.declare_parameter("backup_dir", "/maps/deleted_backups")
        self.declare_parameter("auto_backup", True)  # Автоматический backup перед удалением

        self.db_path = Path(self.get_parameter("db_path").value)
        self.backup_dir = Path(self.get_parameter("backup_dir").value)
        self.auto_backup = self.get_parameter("auto_backup").value

        # Создать директорию для backup
        self.backup_dir.mkdir(parents=True, exist_ok=True)

        # ROS Services
        self.delete_all_srv = self.create_service(
            Trigger, "/rtabmap_manager/delete_all_data", self.delete_all_data_callback
        )

        self.get_logger().info("✅ RTABMap Manager Node запущен")
        self.get_logger().info(f"  БД: {self.db_path}")
        self.get_logger().info(f"  Backup: {self.backup_dir}")
        self.get_logger().info(f"  Auto backup: {self.auto_backup}")

    def delete_all_data_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Полностью удалить БД RTABMap.
        RTABMap автоматически создаст новую пустую БД при следующем запуске.

        Returns:
            Trigger.Response: success=True если удалено, success=False если ошибка
        """
        self.get_logger().warning("🗑️  Запрос на удаление ВСЕЙ БД RTABMap")

        try:
            # Проверить существование БД
            if not self.db_path.exists():
                self.get_logger().warning("  БД не существует, уже удалена")
                response.success = True
                response.message = "БД уже отсутствует (не требует удаления)"
                return response

            # Получить размер БД
            db_size_mb = self.db_path.stat().st_size / (1024 * 1024)
            self.get_logger().info(f"  Размер БД: {db_size_mb:.2f} МБ")

            # Создать backup если включено
            if self.auto_backup:
                backup_path = self._create_backup()
                if backup_path:
                    self.get_logger().info(f"  ✅ Backup: {backup_path.name}")
                else:
                    self.get_logger().warning("  ⚠️  Backup не создан!")

            # Удалить БД
            self.db_path.unlink()
            self.get_logger().info("  ✅ БД удалена!")

            response.success = True
            response.message = f"БД удалена ({db_size_mb:.2f} МБ). RTABMap создаст новую при запуске."
            
            # Логируем в WARNING чтобы было видно
            self.get_logger().warning(f"✅ {response.message}")
            
            return response

        except PermissionError as e:
            error_msg = f"Ошибка прав доступа: {e}"
            self.get_logger().error(f"  ❌ {error_msg}")
            response.success = False
            response.message = error_msg
            return response

        except Exception as e:
            error_msg = f"Ошибка удаления БД: {e}"
            self.get_logger().error(f"  ❌ {error_msg}")
            response.success = False
            response.message = error_msg
            return response

    def _create_backup(self) -> Path | None:
        """
        Создать backup БД с timestamp

        Returns:
            Path: Путь к backup файлу или None если ошибка
        """
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_filename = f"rtabmap_deleted_{timestamp}.db"
            backup_path = self.backup_dir / backup_filename

            # Копировать БД
            shutil.copy2(self.db_path, backup_path)

            return backup_path

        except Exception as e:
            self.get_logger().error(f"Ошибка создания backup: {e}")
            return None


def main(args=None):
    """Главная точка входа"""
    rclpy.init(args=args)
    node = RTABMapManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
