#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import time

# Импортируем библиотеку для управления матрицей
try:
    from pi5neo import Pi5Neo
    PI5NEO_AVAILABLE = True
except ImportError:
    PI5NEO_AVAILABLE = False
    print("Warning: pi5neo library not found. Running in simulation mode.")


class LEDMatrixSimple(Node):
    def __init__(self):
        super().__init__('led_matrix_simple')
        
        # Объявляем параметры
        self.declare_parameters(
            namespace='',
            parameters=[
                ('num_leds', 125),
                ('spi_speed_khz', 1300),
                ('spi_device', '/dev/spidev0.0'),
                ('input_topic', 'led_matrix/data')
            ]
        )
        
        # Получаем параметры
        self.num_leds = self.get_parameter('num_leds').value
        self.spi_speed_khz = self.get_parameter('spi_speed_khz').value
        self.spi_device = self.get_parameter('spi_device').value
        self.input_topic = self.get_parameter('input_topic').value
        
        # Проверяем корректность параметров
        if self.num_leds <= 0:
            self.get_logger().error("Parameter 'num_leds' must be positive")
            raise ValueError("Parameter 'num_leds' must be positive")
        
        # Инициализируем матрицу, если доступна библиотека pi5neo
        self.matrix_initialized = False
        if PI5NEO_AVAILABLE:
            try:
                self.neo = Pi5Neo(self.spi_device, self.num_leds, self.spi_speed_khz)
                self.matrix_initialized = True
                self.get_logger().info(f"LED matrix initialized with {self.spi_speed_khz} kHz SPI speed")
                self.get_logger().info(f"Total LEDs: {self.num_leds}")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize LED matrix: {str(e)}")
                self.matrix_initialized = False
        else:
            self.get_logger().warn("Running in simulation mode (pi5neo not available)")
        
        # Подписываемся на топик с данными
        self.data_subscription = self.create_subscription(
            ByteMultiArray,
            self.input_topic,
            self.data_callback,
            10)
        
        self.get_logger().info(f"Subscribed to topic: {self.input_topic}")
        self.get_logger().info("LED Matrix Simple Driver is ready")
    
    def data_callback(self, msg):
        """Обрабатывает входящие данные для отображения на матрице"""
        if not self.matrix_initialized:
            self.get_logger().warn("Matrix not initialized, skipping data")
            return
        
        # Проверяем длину данных
        expected_length = self.num_leds * 3
        if len(msg.data) != expected_length:
            self.get_logger().warn(f"Received data with incorrect length: {len(msg.data)} (expected {expected_length})")
            return
        
        try:
            # Отображаем данные на матрице
            for i in range(self.num_leds):
                # Определяем индекс в данных
                idx = i * 3
                
                # Получаем цвет
                r = msg.data[idx]
                g = msg.data[idx + 1]
                b = msg.data[idx + 2]
                
                # Устанавливаем цвет
                self.neo.set_led_color(i, r, g, b)
            
            # Обновляем матрицу
            self.neo.update_strip()
        except Exception as e:
            self.get_logger().error(f"Error displaying  {str(e)}")
    
    def destroy_node(self):
        """Очистка при завершении работы"""
        if self.matrix_initialized:
            try:
                # Очищаем матрицу
                self.neo.clear_strip()
                self.neo.update_strip()
                self.get_logger().info("Matrix cleared on shutdown")
            except Exception as e:
                self.get_logger().error(f"Error clearing matrix on shutdown: {str(e)}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    led_matrix_simple = LEDMatrixSimple()
    
    try:
        rclpy.spin(led_matrix_simple)
    except KeyboardInterrupt:
        pass
    finally:
        led_matrix_simple.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()