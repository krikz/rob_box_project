# Анализ системы управления моторами VESC - Проблема рывков колес

**Дата анализа:** 2025-11-07  
**Файл:** `vesc_nexus/src/vesc_system_hardware_interface.cpp`

---

## 🔍 Обнаруженная проблема

### Симптомы
- Колеса движутся **рывками**
- Колеса **удерживают позицию** даже когда команды `/cmd_vel` прекращаются
- Нет механизма **расслабления колес** при отсутствии команд

---

## 📊 Текущая архитектура

### Частота обновления команд

```yaml
# controller_manager.yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # 50 Hz = каждые 20ms

# rob_box_ros2_control.xacro
<param name="publish_rate">50.0</param>
```

**Вывод:** Команды отправляются в CAN шину **50 раз в секунду** (каждые 20ms).

### Путь выполнения команды

```
/cmd_vel (Twist) 
    ↓
twist_mux → /diff_drive_controller/cmd_vel
    ↓
DiffDriveController (преобразование Twist → wheel velocities)
    ↓
write() вызывается @ 50 Hz
    ↓
VescSystemHardwareInterface::write()
    ↓
vesc_handlers_[i]->sendSpeed(linear_speed)
    ↓
VescHandler::sendSpeed()
    ↓
createSetSpeedFrame(can_id, erpm) → CAN bus
```

---

## ❌ Проблемы в текущей реализации

### 1. Нет таймаута команд на уровне Hardware Interface

**Код в `VescSystemHardwareInterface::write()`:**
```cpp
hardware_interface::return_type VescSystemHardwareInterface::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    double linear_speed = cmd_velocities_[i] * wheel_radius_;  // rad/s → m/s
    vesc_handlers_[i]->sendSpeed(linear_speed);  // ⚠️ ВСЕГДА отправляет команду!
  }
  return hardware_interface::return_type::OK;
}
```

**Проблема:**
- Метод `write()` вызывается **каждые 20ms** (50 Hz)
- Он **ВСЕГДА** отправляет команду в CAN, даже если `cmd_velocities_[i]` не обновлялся
- Если последняя команда была 0.0 m/s, VESC продолжает получать 0.0 ERPM каждые 20ms
- VESC интерпретирует 0.0 ERPM как **"удерживать позицию"**, а не **"расслабить мотор"**

### 2. Нет механизма "расслабления" мотора

**В VESC есть несколько режимов:**
- `CAN_PACKET_SET_RPM` (3) - Установить скорость (мотор активен)
- `CAN_PACKET_SET_CURRENT` (1) - Установить ток (мотор активен)
- `CAN_PACKET_SET_CURRENT_BRAKE` (2) - Установить тормозной ток
- **НЕТ команды:** "Выключить мотор / Free spin"

**Текущая логика:**
```cpp
void VescHandler::sendSpeed(double linear_speed) {
    // ... конвертация linear_speed → erpm ...
    
    // ⚠️ Ограничение по min_erpm
    if (std::abs(erpm) > 0 && std::abs(erpm) < min_erpm_) {
        erpm = (erpm > 0) ? min_erpm_ : -min_erpm_;
    }
    
    // Отправляем ERPM (ВСЕГДА!)
    auto frame = vesc_nexus::createSetSpeedFrame(can_id_, erpm);
    send_can_func_(frame);
}
```

**Проблема:**
- Даже если `linear_speed = 0.0`, отправляется `erpm = 0.0`
- VESC удерживает мотор с нулевой скоростью вместо расслабления

### 3. DiffDriveController имеет таймаут, но он не передается в Hardware Interface

**В `controller_manager.yaml`:**
```yaml
diff_drive_controller:
  ros__parameters:
    cmd_vel_timeout: 0.5  # seconds - остановка если нет команд
```

**Проблема:**
- DiffDriveController устанавливает `cmd_velocities_[i] = 0.0` после таймаута
- Но Hardware Interface не знает **разницы между:**
  - "Активная команда: двигайся со скоростью 0.0" (→ удержание)
  - "Нет команд уже 0.5 сек" (→ нужно расслабить мотор)

---

## ✅ Решения

### Решение 1: Добавить таймаут команд в Hardware Interface (Рекомендуется)

**Изменения в `vesc_system_hardware_interface.hpp`:**
```cpp
class VescSystemHardwareInterface : public hardware_interface::SystemInterface {
private:
  std::vector<double> cmd_velocities_;
  std::vector<rclcpp::Time> last_command_time_;  // ✅ Добавить
  double command_timeout_;  // ✅ Добавить (из конфига)
};
```

**Изменения в `vesc_system_hardware_interface.cpp`:**

```cpp
hardware_interface::CallbackReturn VescSystemHardwareInterface::on_init(
  const hardware_interface::HardwareInfo& info) {
  // ... существующий код ...
  
  // ✅ Добавить параметр таймаута
  command_timeout_ = 0.5;  // default 0.5 seconds
  if (info.hardware_parameters.find("command_timeout") != info.hardware_parameters.end()) {
    command_timeout_ = std::stod(info.hardware_parameters.at("command_timeout"));
  }
  
  // Инициализация буферов
  for (const auto& joint : info.joints) {
    // ... существующий код ...
    last_command_time_.push_back(rclcpp::Time(0));  // ✅ Добавить
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VescSystemHardwareInterface::write(
  const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    // ✅ Проверка изменения команды
    bool command_changed = std::abs(cmd_velocities_[i]) > 1e-6;
    
    if (command_changed) {
      // Команда активна - обновляем время
      last_command_time_[i] = time;
      double linear_speed = cmd_velocities_[i] * wheel_radius_;
      vesc_handlers_[i]->sendSpeed(linear_speed);
    } else {
      // Команда = 0.0, проверяем таймаут
      double time_since_command = (time - last_command_time_[i]).seconds();
      
      if (time_since_command < command_timeout_) {
        // Недавно была команда - отправляем 0.0 (активное торможение)
        vesc_handlers_[i]->sendSpeed(0.0);
      } else {
        // ✅ Таймаут превышен - расслабляем мотор
        vesc_handlers_[i]->sendRelease();  // Новый метод!
      }
    }
  }
  
  return hardware_interface::return_type::OK;
}
```

**Добавить в `vesc_handler.hpp`:**
```cpp
class VescHandler {
public:
  // ... существующие методы ...
  
  // ✅ Новый метод для расслабления мотора
  void sendRelease();
};
```

**Добавить в `vesc_handler.cpp`:**
```cpp
void VescHandler::sendRelease() {
    if (!send_can_func_) return;
    
    // Вариант 1: Отправить очень маленький ток (почти 0)
    auto frame = vesc_nexus::createSetCurrentFrame(can_id_, 0.01);  // 10mA
    send_can_func_(frame);
    
    // Вариант 2: Не отправлять ничего (VESC перейдет в timeout сам)
    // (но это может быть опасно если VESC настроен агрессивно)
}
```

**Обновить URDF (`rob_box_ros2_control.xacro`):**
```xml
<hardware>
  <plugin>vesc_nexus/VescSystemHardwareInterface</plugin>
  <param name="can_interface">can0</param>
  <param name="publish_rate">50.0</param>
  <param name="wheel_radius">0.115</param>
  
  <!-- ✅ Добавить таймаут -->
  <param name="command_timeout">0.5</param>  <!-- seconds -->
</hardware>
```

---

### Решение 2: Использовать режим Current Control вместо Speed Control

**Проблема Speed Control:**
- VESC активно удерживает заданную скорость (PID регулятор)
- При 0 RPM → активно тормозит / удерживает

**Преимущества Current Control:**
- Если ток = 0 → мотор расслаблен (free spin)
- Более плавное управление
- Меньше рывков

**Изменения в `VescHandler::sendSpeed()`:**
```cpp
void VescHandler::sendSpeed(double linear_speed) {
    if (!send_can_func_) return;

    // Конвертируем скорость в ожидаемый ток
    // (требуется калибровка: A/m/s)
    const double CURRENT_PER_SPEED = 2.0;  // 2A на 1 m/s (пример)
    double current = linear_speed * CURRENT_PER_SPEED;
    
    // Ограничиваем ток
    current = std::clamp(current, -10.0, 10.0);
    
    // ✅ При нулевой скорости ток = 0 → мотор расслаблен
    auto frame = vesc_nexus::createSetCurrentFrame(can_id_, current);
    send_can_func_(frame);
}
```

**Недостатки:**
- Менее точное управление скоростью
- Требует калибровки коэффициента `CURRENT_PER_SPEED`

---

### Решение 3: Hybrid - Speed Control + Timeout + Current Release

Комбинация лучших аспектов:

```cpp
hardware_interface::return_type VescSystemHardwareInterface::write(
  const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    double linear_speed = cmd_velocities_[i] * wheel_radius_;
    
    // Детектируем новую команду (не 0)
    if (std::abs(cmd_velocities_[i]) > 1e-6) {
      last_command_time_[i] = time;
      vesc_handlers_[i]->sendSpeed(linear_speed);  // Speed control
    } else {
      double time_since_command = (time - last_command_time_[i]).seconds();
      
      if (time_since_command < 0.1) {  // 100ms grace period
        // Активное торможение (Speed = 0)
        vesc_handlers_[i]->sendSpeed(0.0);
      } else if (time_since_command < command_timeout_) {
        // Легкий тормоз (Current = 0.1A)
        vesc_handlers_[i]->sendCurrent(0.1);
      } else {
        // ✅ Полное расслабление (Current = 0)
        vesc_handlers_[i]->sendCurrent(0.0);
      }
    }
  }
  
  return hardware_interface::return_type::OK;
}
```

---

## 🎯 Рекомендуемое решение

**Комбинация Решения 1 + Решения 3:**

1. **Добавить таймаут команд** в Hardware Interface
2. **Трёхступенчатая логика:**
   - 0-100ms без команд → Speed control (0 RPM) для точной остановки
   - 100ms-500ms → Легкий ток (0.1-0.5A) для предотвращения скатывания
   - >500ms → Ток = 0 (полное расслабление)

3. **Параметр в конфиге** для настройки поведения:
```yaml
# В будущем vesc_nexus.yaml
vesc_nexus:
  ros__parameters:
    command_timeout: 0.5
    hold_current: 0.2  # A - ток удержания после остановки
    release_timeout: 0.1  # s - время до перехода на ток удержания
```

---

## 📝 Дальнейшие действия

### Немедленные (для текущей отладки):

1. **Проверить текущее поведение:**
   ```bash
   # На Main Pi
   ros2 topic echo /cmd_vel
   ros2 topic echo /joint_states
   
   # Логи VESC
   docker logs ros2-control --tail 100 -f
   ```

2. **Тест с ручными командами:**
   ```bash
   # Отправить команду вперед
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.1}, angular: {z: 0.0}}"
   
   # Подождать 1 секунду, затем стоп
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.0}, angular: {z: 0.0}}"
   
   # Проверить - останавливаются ли колеса плавно или рывком?
   ```

### Долгосрочные (изменения в vesc_nexus):

1. Внести изменения в `vesc_nexus` субмодуль
2. Протестировать с разными таймаутами
3. Добавить параметр `command_timeout` в URDF
4. Добавить метод `sendRelease()` в VescHandler
5. Создать PR в https://github.com/krikz/vesc_nexus

---

## 🔗 Связанные файлы

- `src/vesc_nexus/src/vesc_nexus/src/vesc_system_hardware_interface.cpp`
- `src/vesc_nexus/src/vesc_nexus/src/vesc_handler.cpp`
- `src/rob_box_description/urdf/rob_box_ros2_control.xacro`
- `docker/main/config/controllers/controller_manager.yaml`
- `docs/packages/vesc_nexus/VESC_COMMAND_TIMEOUT_FIX.md` (этот документ)

---

**Автор:** AI Assistant  
**Версия:** 1.0
