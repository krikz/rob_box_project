# Визуализация робота в RViz

Этот гайд показывает как запустить визуализацию Rob Box в RViz для проверки URDF модели и корректности геометрии робота.

![Rob Box в RViz](../assets/rob_box_rviz_demo.gif)

## Что визуализируется

### Компоненты робота:
- **base_link** - основной корпус (белый)
- **4 колеса** - continuous joints с правильными осями вращения (черные)
  - `rear_left_wheel_link`
  - `front_left_wheel_link`
  - `front_right_wheel_link`
  - `rear_right_wheel_link`

### Сенсоры:
- **LSLIDAR N10** - 2D лидар (черный)
- **OAK-D Lite** - стерео камера (темно-серый)
- **RPi Camera** - потолочная камера (зеленый, направлена вверх)

### TF Frames:
- Все TF frames отображаются правильно
- Оси координат: X (красный), Y (зеленый), Z (синий)
- Колеса имеют TF frames в центрах для правильного вращения

## Геометрия робота

Все размеры взяты из Fusion 360 экспорта:

| Параметр | Значение | Описание |
|----------|----------|----------|
| **Wheelbase** | 390 мм | База (расстояние между левым и правым колесом по оси X) |
| **Track** | 289 мм | Колея (расстояние между передним и задним колесом по оси Y) |
| **Wheel Radius** | 115 мм | Радиус колеса |
| **Wheel Width** | 75 мм | Ширина колеса |

### Координаты компонентов:

**Колеса** (относительно base_link):
```
rear_left:   ( 0.195,  0.144, 0.029)
front_left:  ( 0.195, -0.146, 0.029)
front_right: (-0.196, -0.145, 0.029)
rear_right:  (-0.195,  0.144, 0.029)
```

**Сенсоры**:
```
LSLIDAR N10:  (-0.0003,  0.171, 0.477)
OAK-D Lite:   (-0.0002,  0.116, 0.460)
RPi Camera:   ( 0.065,   0.171, 0.462) [повернута на pitch=-90°]
```

## Запуск визуализации

### Вариант 1: Launch файл (рекомендуется)

```bash
cd ~/rob_box_project
source install/setup.bash
ros2 launch rob_box_bringup display_simple.launch.py
```

Этот launch файл запускает:
1. **robot_state_publisher** - публикует TF дерево
2. **dummy_joint_state_publisher** - симулирует вращение колес
3. **RViz2** - визуализация

### Вариант 2: Вручную

Терминал 1 - Robot State Publisher:
```bash
cd ~/rob_box_project
source install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro src/rob_box_description/urdf/rob_box_complete.xacro)"
```

Терминал 2 - Joint State Publisher (для вращения колес):
```bash
cd ~/rob_box_project
source install/setup.bash
ros2 run rob_box_bringup dummy_joint_state_publisher.py
```

Терминал 3 - RViz:
```bash
cd ~/rob_box_project
source install/setup.bash
rviz2 -d src/rob_box_description/rviz/rob_box.rviz
```

## Проверка TF трансформаций

Проверить трансформацию между фреймами:
```bash
ros2 run tf2_ros tf2_echo base_link rear_left_wheel_link
```

Вывод должен показать:
```
Translation: [0.195, 0.144, 0.029]
Rotation: quaternion [0, 0, 0, 1]
```

Посмотреть все TF фреймы:
```bash
ros2 run tf2_tools view_frames
```

Это создаст файл `frames.pdf` с полным TF деревом робота.

## Настройки RViz

Файл конфигурации: `src/rob_box_description/rviz/rob_box.rviz`

Основные дисплеи:
- **Grid** - сетка для ориентации
- **RobotModel** - 3D модель робота из URDF
- **TF** - отображение всех систем координат

Fixed Frame: `base_link`

## Материалы и стили

Доступны 6 стилей оформления (в `rob_box_materials.xacro`):

1. **Классический** (по умолчанию): белый корпус + черные колеса
2. **Киберпанк**: космический градиент + синие неоновые колеса
3. **Гоночный**: красный корпус + золотые диски
4. **Премиум**: матовое золото + черные колеса
5. **Хром**: серебристый металлик + хромированные колеса
6. **Matrix**: черный + зеленые неоновые колеса

Чтобы изменить стиль, отредактируйте `rob_box_complete.xacro`:
```xml
<!-- Замените rob_box_white на другой материал -->
<material name="space_gradient"/>  <!-- для киберпанка -->
<material name="racing_red"/>      <!-- для гоночного -->
```

## Устранение проблем

### RViz не показывает робота

1. Проверьте что robot_state_publisher запущен:
```bash
ros2 node list | grep robot_state_publisher
```

2. Проверьте публикацию TF:
```bash
ros2 topic echo /tf_static --once
```

3. Проверьте robot_description:
```bash
ros2 topic echo /robot_description --once | head -50
```

### Колеса не вращаются

Убедитесь что dummy_joint_state_publisher запущен:
```bash
ros2 node list | grep dummy_joint_state_publisher
```

Проверьте joint_states:
```bash
ros2 topic echo /joint_states
```

### TF frames в неправильных местах

Пересоберите пакет:
```bash
cd ~/rob_box_project
rm -rf build/rob_box_description install/rob_box_description
colcon build --packages-select rob_box_description
source install/setup.bash
```

## Интеграция с ros2_control

После проверки визуализации можно интегрировать с реальными моторами VESC.

См. [VESC Integration Guide](../reference/vesc-integration.md) для деталей.

## Следующие шаги

- [ ] Проверить ориентацию всех сенсоров
- [ ] Протестировать ros2_control с VESC моторами
- [ ] Добавить в docker-compose для автоматического запуска
- [ ] Настроить RViz конфигурацию для навигации

## Ссылки

- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
- [robot_state_publisher](http://wiki.ros.org/robot_state_publisher)
- [RViz User Guide](http://wiki.ros.org/rviz/UserGuide)
- [TF2 Tutorials](http://wiki.ros.org/tf2/Tutorials)
