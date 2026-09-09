# 🚨 Быстрое решение проблемы рывков колес

## Проблема

Колеса движутся рывками и не расслабляются при остановке.

## Причина

Команды отправляются в CAN шину **50 раз в секунду** (50 Hz), даже когда скорость = 0.0.  
VESC интерпретирует 0.0 ERPM как **"удерживать позицию"**, а не **"расслабить"**.

## Решение (3 варианта)

### Вариант 1: Временное решение (быстро, без изменений кода)

Увеличить `cmd_vel_timeout` и добавить decay:

```yaml
# docker/main/config/controllers/controller_manager.yaml
diff_drive_controller:
  ros__parameters:
    cmd_vel_timeout: 0.1  # Уменьшить до 100ms (было 0.5)
```

Перезапустить:

```bash
cd ~/rob_box_project/docker/main
docker compose restart ros2-control
```

### Вариант 2: Изменение в vesc_nexus (правильное решение)

**Файл:** `src/vesc_nexus/src/vesc_nexus/src/vesc_system_hardware_interface.cpp`

```cpp
// Добавить в класс:
std::vector<rclcpp::Time> last_command_time_;
double command_timeout_ = 0.5;

// В on_init():
last_command_time_.resize(info.joints.size(), rclcpp::Time(0));

// В write():
hardware_interface::return_type VescSystemHardwareInterface::write(
  const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    bool has_command = std::abs(cmd_velocities_[i]) > 1e-6;
    
    if (has_command) {
      last_command_time_[i] = time;
      double linear_speed = cmd_velocities_[i] * wheel_radius_;
      vesc_handlers_[i]->sendSpeed(linear_speed);
    } else {
      double timeout = (time - last_command_time_[i]).seconds();
      
      if (timeout < command_timeout_) {
        vesc_handlers_[i]->sendSpeed(0.0);  // Активное торможение
      }
      // Иначе - не отправляем команды (мотор расслабится)
    }
  }
  return hardware_interface::return_type::OK;
}
```

### Вариант 3: Использовать Current Control (экспериментально)

Вместо `CAN_PACKET_SET_RPM` использовать `CAN_PACKET_SET_CURRENT`:

```cpp
void VescHandler::sendSpeed(double linear_speed) {
    // При ток = 0 → мотор свободен
    double current = linear_speed * 2.0;  // 2A на 1 m/s
    auto frame = vesc_nexus::createSetCurrentFrame(can_id_, current);
    send_can_func_(frame);
}
```

## Тестирование

```bash
# 1. Запустить робота
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

# 2. Подождать 1 секунду

# 3. Остановить
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"

# 4. Наблюдать - колеса должны остановиться плавно и расслабиться
```

## Проверка результата

✅ **Успех, если:**

- Колеса останавливаются **плавно** без рывков
- После остановки колеса **можно крутить рукой** (расслаблены)
- Нет "удержания позиции"

❌ **Не работает, если:**

- Колеса всё еще рывками
- Колеса напряжены после остановки (сопротивление вращению)

## Полная документация

См. [VESC_COMMAND_TIMEOUT_FIX.md](./VESC_COMMAND_TIMEOUT_FIX.md)
