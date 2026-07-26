# 🤖 Сравнение параметров Nav2: Rob Box vs открытые роверы

**Дата:** 16 декабря 2025  
**Цель:** Сравнить настройки навигации Rob Box с эталонными четырехколесными роверами

---

## 📊 Сравнительная таблица параметров

### 🚗 Характеристики роботов

| Параметр | **Rob Box** | **TurtleBot3 Burger** | **Husky** | **Jackal** |
|----------|------------|---------------------|-----------|-----------|
| **Тип привода** | Дифференциальный (4 колеса) | Дифференциальный (2 колеса) | Дифференциальный (4 колеса) | Дифференциальный (4 колеса) |
| **Размер (м)** | 0.4×0.4×0.3 | 0.140×0.140×0.192 | 1.0×0.67×0.39 | 0.508×0.430×0.250 |
| **Вес (кг)** | ~8-10 кг | 1.0 кг | 50 кг | 17 кг |
| **Макс. скорость** | 0.3 m/s | 0.22 m/s | 1.0 m/s | 2.0 m/s |
| **Лидар** | LS-C16 (360°) | LDS-01 (360°, 3.5m) | Sick LMS1XX | Sick LMS1XX |
| **Одометрия** | ICP (RTAB-Map) | Колесная | Колесная | Колесная |
| **ROS Distro** | kilted | kilted | Noetic (ROS 1) | Noetic (ROS 1) |

---

## ⚙️ DWB Controller Parameters

### 🎯 Velocity Limits (Линейная скорость)

| Параметр | **Rob Box<br>(текущий)** | **Rob Box<br>(ffcff97)** | **TurtleBot3<br>Burger** | **Рекомендация** |
|----------|------------|------------|------------|----------|
| **max_vel_x** | 0.15 m/s | ✅ 0.15 m/s | 0.22 m/s | ✅ 0.15 OK (68% от TB3) |
| **min_vel_x** | 0.0 m/s | 0.0 m/s | 0.0 m/s | ✅ OK |
| **max_vel_y** | 0.0 m/s | 0.0 m/s | 0.0 m/s | ✅ OK (diff drive) |
| **acc_lim_x** | 0.2 m/s² | 0.2 m/s² | 2.5 m/s² | ⚠️ **Слишком низкое!** |
| **decel_lim_x** | -0.2 m/s² | -0.2 m/s² | -2.5 m/s² | ⚠️ **Слишком низкое!** |

**Анализ:**
- **max_vel_x**: Rob Box медленнее TurtleBot3 → консервативно ✅
- **acc_lim_x**: Rob Box 0.2 vs TB3 2.5 → **в 12.5 раз медленнее!** 🔴
  - Время разгона 0→0.15 m/s: Rob Box = 0.75s, TB3 = 0.06s
  - Это может быть причиной "no valid trajectories" — робот не успевает маневрировать!

### 🔄 Angular Velocity (Угловая скорость)

| Параметр | **Rob Box<br>(текущий)** | **Rob Box<br>(ffcff97)** | **TurtleBot3<br>Burger** | **Рекомендация** |
|----------|------------|------------|------------|----------|
| **max_vel_theta** | 0.6 rad/s<br>(~34°/s) | ✅ 0.6 rad/s | 1.0 rad/s<br>(~57°/s) | ⚠️ Можно 0.8 |
| **min_speed_theta** | 0.1 rad/s | ✅ 0.1 rad/s | 0.0 rad/s | ⚠️ TB3 = 0! |
| **acc_lim_theta** | 0.6 rad/s² | ✅ 0.6 rad/s² | 3.2 rad/s² | ⚠️ **в 5x медленнее** |
| **decel_lim_theta** | -0.6 rad/s² | -0.6 rad/s² | -3.2 rad/s² | ⚠️ **в 5x медленнее** |

**Анализ:**
- **max_vel_theta**: Rob Box 60% от TB3 → консервативно, но разумно ✅
- **min_speed_theta**: Rob Box = 0.1, TB3 = 0.0
  - Rob Box требует минимум 6°/s для поворота
  - TurtleBot3 может поворачивать сколь угодно медленно
  - Может ограничивать fine-tuning траекторий ⚠️
- **acc_lim_theta**: Rob Box 0.6 vs TB3 3.2 → **в 5.3x медленнее!** 🔴
  - Время разгона 0→0.6 rad/s: Rob Box = 1.0s, TB3 = 0.19s

### 📐 Trajectory Simulation

| Параметр | **Rob Box** | **TurtleBot3** | **Анализ** |
|----------|----------|-----------|----------|
| **sim_time** | 2.0s | 1.5s | Rob Box симулирует дальше |
| **vx_samples** | 20 | 20 | ✅ Одинаково |
| **vy_samples** | 5 | 0 | ⚠️ У Rob Box лишние (diff drive!) |
| **vtheta_samples** | 40 | 40 | ✅ Одинаково |
| **linear_granularity** | 0.05 | 0.05 | ✅ Одинаково |
| **angular_granularity** | 0.025 (~1.4°) | 0.025 (~1.4°) | ✅ Одинаково |

**Проблема:** `vy_samples: 5` бесполезно для дифференциального привода! ❌  
**Рекомендация:** `vy_samples: 0` (экономия 5x траекторий)

### 🎯 Critics (Оценщики траекторий)

| Critic | **Rob Box** | **TurtleBot3** | **Комментарий** |
|--------|----------|-----------|-------------|
| **RotateToGoal** | scale: 32.0 | scale: 32.0 | ✅ Одинаково |
| **Oscillation** | scale: **0.5** ⭐ | scale: 1.0 (default) | Rob Box более permissive |
| **BaseObstacle** | scale: 0.02 | scale: 0.02 | ✅ Одинаково |
| **GoalAlign** | scale: 24.0 | scale: 24.0 | ✅ Одинаково |
| **PathAlign** | scale: 32.0 | scale: 32.0 | ✅ Одинаково |
| **PathDist** | scale: 32.0 | scale: 32.0 | ✅ Одинаково |
| **GoalDist** | scale: 24.0 | scale: 24.0 | ✅ Одинаково |
| **PreferForward** | ❌ Удалено | ❌ Нет | ✅ Правильно |

**Анализ Oscillation:**
- TurtleBot3: default 1.0 (строгий контроль осцилляций)
- Rob Box: 0.5 (более permissive из-за ICP uncertainty ±7mm)
- **Вердикт:** Rob Box правильно снизил для ICP ✅

---

## 🗺️ Costmap Parameters

### Local Costmap

| Параметр | **Rob Box** | **TurtleBot3** | **Анализ** |
|----------|----------|-----------|----------|
| **update_frequency** | 5.0 Hz | 5.0 Hz | ✅ Одинаково |
| **publish_frequency** | 2.0 Hz | 2.0 Hz | ✅ Одинаково |
| **width** | 4 m | 3 m | Rob Box +33% больше |
| **height** | 4 m | 3 m | Rob Box +33% больше |
| **resolution** | 0.05 m | 0.05 m | ✅ Одинаково |
| **robot_radius** | **0.5 m** 🔴 | 0.1 m | **Rob Box в 5x больше!** |
| **inflation_radius** | **0.55 m** | 1.0 m | Rob Box меньше |
| **cost_scaling_factor** | 5.0 | 3.0 | Rob Box более aggressive |
| **transform_tolerance** | **0.5s** ⭐ | 0.2s | **Критично для ICP!** |

**Анализ robot_radius:**
```
Rob Box физический размер: 400×400mm = 0.4×0.4m
Диагональ: √(0.4² + 0.4²) = 0.566m
robot_radius: 0.5m (footprint ≈ 1000mm круг)

TurtleBot3 размер: 140×140mm
robot_radius: 0.1m (footprint ≈ 200mm круг)
```

**Вывод:** `robot_radius: 0.5m` правильный (200mm safety margin ✅), но может быть слишком консервативным в узких проходах!

**transform_tolerance:**
- Rob Box: 0.5s → критично для ICP delay 30-125ms ✅
- TurtleBot3: 0.2s → достаточно для колесной одометрии

### Global Costmap

| Параметр | **Rob Box** | **TurtleBot3** | **Анализ** |
|----------|----------|-----------|----------|
| **update_frequency** | 1.0 Hz | 1.0 Hz | ✅ Одинаково |
| **resolution** | 0.05 m | 0.05 m | ✅ Одинаково |
| **robot_radius** | **0.5 m** | 0.1 m | Rob Box в 5x больше |
| **inflation_radius** | **0.6 m** | 0.55 m | Rob Box +9% больше |
| **transform_tolerance** | **0.5s** ⭐ | нет | **Добавлено Rob Box!** |

---

## 🎮 Controller Server

| Параметр | **Rob Box** | **TurtleBot3** | **Анализ** |
|----------|----------|-----------|----------|
| **controller_frequency** | 20.0 Hz | 10.0 Hz | **Rob Box в 2x быстрее!** ⭐ |
| **min_x_velocity_threshold** | 0.001 m/s | 0.001 m/s | ✅ Одинаково |
| **min_theta_velocity_threshold** | 0.001 rad/s | 0.001 rad/s | ✅ Одинаково |
| **failure_tolerance** | 0.3 | 0.3 | ✅ Одинаково |
| **xy_goal_tolerance** | 0.1 m | 0.25 m | **Rob Box в 2.5x точнее!** |
| **yaw_goal_tolerance** | 0.1 rad (~6°) | 0.25 rad (~14°) | **Rob Box в 2.5x точнее!** |

**Анализ:**
- **controller_frequency: 20Hz** → Rob Box обновляет команды в 2x чаще ✅
  - Может компенсировать медленные acc_lim!
- **goal_tolerance: 0.1m** → Rob Box требует точнее целиться ✅
  - Хорошо для ICP precision ±7mm

---

## 🔍 Ключевые находки

### 🔴 КРИТИЧНЫЕ ПРОБЛЕМЫ

#### 1. **Ускорения в 5-12 раз медленнее TurtleBot3!**

```yaml
Rob Box:
  acc_lim_x: 0.2 m/s²        # Время 0→0.15 m/s = 0.75s
  acc_lim_theta: 0.6 rad/s²  # Время 0→0.6 rad/s = 1.0s

TurtleBot3:
  acc_lim_x: 2.5 m/s²        # Время 0→0.22 m/s = 0.088s (8.5x быстрее!)
  acc_lim_theta: 3.2 rad/s²  # Время 0→1.0 rad/s = 0.31s (3.2x быстрее!)
```

**Почему это проблема:**
- DWB генерирует траектории на `sim_time: 2.0s` вперед
- За 2 секунды Rob Box едва успевает разогнаться
- TurtleBot3 за то же время успевает сделать полный маневр
- Результат: "No valid trajectories" → backup → failure

**Рекомендация:**
```yaml
# ТЕСТ 1: Приблизить к TurtleBot3
acc_lim_x: 0.5 m/s²        # +150% (было 0.2)
acc_lim_theta: 1.2 rad/s²  # +100% (было 0.6)
decel_lim_x: -0.5 m/s²
decel_lim_theta: -1.2 rad/s²

# ТЕСТ 2: Если Test 1 ОК, еще агрессивнее
acc_lim_x: 1.0 m/s²        # +400%
acc_lim_theta: 2.0 rad/s²  # +233%
```

#### 2. **min_speed_theta: 0.1 rad/s ограничивает точную навигацию**

TurtleBot3: `min_speed_theta: 0.0` → может крутиться сколь угодно медленно  
Rob Box: `min_speed_theta: 0.1` → минимум 6°/s

**Проблема:** В узких проходах или при точном наведении робот не может делать микро-коррекции.

**Рекомендация:**
```yaml
min_speed_theta: 0.0  # Разрешить медленные повороты
```

#### 3. **vy_samples: 5 бесполезны для дифференциального привода**

```yaml
vx_samples: 20
vy_samples: 5   # ❌ Бесполезно! Diff drive не может двигаться вбок
vtheta_samples: 40

# Всего траекторий: 20 × 5 × 40 = 4000 траекторий
# Из них полезных: 20 × 1 × 40 = 800 траекторий (только 20%!)
```

**Рекомендация:**
```yaml
vy_samples: 0  # Diff drive не может двигаться вбок!
# Экономия: 5x меньше вычислений!
```

### ⚠️ ПОТЕНЦИАЛЬНЫЕ УЛУЧШЕНИЯ

#### 4. **robot_radius: 0.5m может быть слишком большим**

```
Физический размер: 400×400mm
Диагональ: 566mm
robot_radius: 500mm (200mm safety margin)
```

**Проблема:** В узких проходах (например, дверные проемы 800mm) робот может считать проход непроходимым:
```
Дверь 800mm
- robot_radius 500mm × 2 = 1000mm нужно
- Результат: ❌ "hitting obstacles"
```

**Рекомендация тестов:**
```yaml
# ТЕСТ 1: Moderate (150mm margin)
robot_radius: 0.45 m

# ТЕСТ 2: Minimal (100mm margin)
robot_radius: 0.4 m

# ТЕСТ 3: Aggressive (50mm margin)
robot_radius: 0.35 m
```

#### 5. **controller_frequency: 20Hz может быть избыточным**

TurtleBot3: 10Hz  
Rob Box: 20Hz (2x быстрее)

**Вопрос:** Нужна ли такая частота при медленных ускорениях?

**Рекомендация:**
```yaml
# ТЕСТ: Если повысим acc_lim, можно снизить частоту
controller_frequency: 15.0  # Компромисс
```

### ✅ ЧТО РАБОТАЕТ ХОРОШО

1. **transform_tolerance: 0.5s** → критично для ICP delay ✅
2. **Oscillation.scale: 0.5** → правильно для ICP ±7mm ✅
3. **goal_tolerance: 0.1m** → точнее TurtleBot3 ✅
4. **controller_frequency: 20Hz** → компенсирует медленные ускорения ✅
5. **max_vel_x: 0.15** → консервативно и безопасно ✅

---

## 🎯 Рекомендованная конфигурация

### 🔥 Приоритет 1: FIX КРИТИЧНЫХ ПРОБЛЕМ

```yaml
# docker/main/config/nav2/nav2_params.yaml

controller_server:
  ros__parameters:
    FollowPath:
      # ===== КРИТИЧНО: Повысить ускорения =====
      acc_lim_x: 0.5          # Было: 0.2 (+150%)
      decel_lim_x: -0.5       # Было: -0.2
      acc_lim_theta: 1.2      # Было: 0.6 (+100%)
      decel_lim_theta: -1.2   # Было: -0.6
      
      # ===== Разрешить медленные повороты =====
      min_speed_theta: 0.0    # Было: 0.1 (разрешить микро-коррекции)
      
      # ===== Убрать бесполезные траектории =====
      vy_samples: 0           # Было: 5 (diff drive не может вбок!)
      
      # ===== Оставить как есть =====
      max_vel_x: 0.15         # ✅ OK
      max_vel_theta: 0.6      # ✅ OK
      vx_samples: 20          # ✅ OK
      vtheta_samples: 40      # ✅ OK
      sim_time: 2.0           # ✅ OK
      
local_costmap:
  local_costmap:
    ros__parameters:
      # ===== ТЕСТ: Уменьшить robot_radius =====
      robot_radius: 0.45      # Было: 0.5 (150mm margin вместо 200mm)
```

### 📊 Приоритет 2: ТЕСТ АГРЕССИВНЫХ ПАРАМЕТРОВ

Если Priority 1 работает хорошо, попробовать:

```yaml
acc_lim_x: 1.0            # +400% от текущего
acc_lim_theta: 2.0        # +233% от текущего
max_vel_theta: 0.8        # +33% (ближе к TB3)
robot_radius: 0.4         # 100mm safety margin
controller_frequency: 15.0 # Снизить частоту (компенсируется acc_lim)
```

---

## 📈 Ожидаемые результаты

### После Priority 1 fixes:

✅ **Меньше "No valid trajectories"** (робот успевает маневрировать)  
✅ **Плавнее движение** (быстрее разгон/торможение)  
✅ **Точнее навигация** (микро-коррекции min_speed_theta: 0)  
✅ **Быстрее вычисления** (5x меньше траекторий без vy_samples)  
✅ **Проходит узкие места** (robot_radius: 0.45m)

### Метрики для проверки:

```bash
# Перед изменениями
docker logs nav2 | grep "No valid trajectories" | wc -l
# Ожидается: много (10-50 в минуту)

# После изменений
docker logs nav2 | grep "No valid trajectories" | wc -l
# Ожидается: мало (<5 в минуту)

# Goal success rate
docker logs nav2 | grep "Goal succeeded" | wc -l
# Ожидается: 90-100% целей достигнуто
```

---

## 📚 Источники данных

### TurtleBot3 Burger (ROS 2 kilted)
- **Repository:** https://github.com/ROBOTIS-GIT/turtlebot3
- **File:** `turtlebot3_navigation2/param/kilted/burger.yaml`
- **Характеристики:**
  - Размер: 140×140×192mm
  - Вес: 1.0 кг
  - Max speed: 0.22 m/s (linear), 2.84 rad/s (angular)
  - Lidar: LDS-01 (360°, 3.5m range)
  - Одометрия: Колесная (2 колеса)

### Husky (ROS 1 Noetic)
- **Repository:** https://github.com/husky/husky
- **Статус:** ROS 1 only, нет Nav2 параметров
- **Характеристики:**
  - Размер: 1.0×0.67×0.39m
  - Вес: 50 кг
  - Max speed: 1.0 m/s
  - Lidar: Sick LMS1XX

### Jackal (ROS 1 Noetic)
- **Repository:** https://github.com/jackal/jackal
- **Статус:** ROS 1 only, нет Nav2 параметров
- **Характеристики:**
  - Размер: 0.508×0.430×0.250m
  - Вес: 17 кг
  - Max speed: 2.0 m/s
  - Lidar: Sick LMS1XX

---

## 🎓 Уроки от TurtleBot3

### 1. Агрессивные ускорения работают!

TurtleBot3 — самый легкий робот (1 кг), но имеет **самые агрессивные ускорения:**
- `acc_lim_x: 2.5 m/s²` (Rob Box: 0.2)
- `acc_lim_theta: 3.2 rad/s²` (Rob Box: 0.6)

**Вывод:** Даже маленький робот может иметь высокие ускорения. Rob Box (8-10 кг) может безопасно иметь `acc_lim_x: 0.5-1.0`.

### 2. min_speed_theta: 0.0 стандартная практика

TurtleBot3 не ограничивает минимальную угловую скорость.  
Rob Box `min_speed_theta: 0.1` — это **исключение**, скорее всего из-за старой настройки.

### 3. vy_samples: 0 для diff drive

TurtleBot3 правильно использует `vy_samples: 0` для дифференциального привода.  
Rob Box `vy_samples: 5` — **ошибка конфигурации**, тратит 80% траекторий впустую.

### 4. Точные goal tolerances полезны

TurtleBot3 использует `xy_goal_tolerance: 0.25m`.  
Rob Box использует `0.1m` — **лучше!** ✅

### 5. transform_tolerance зависит от одометрии

- TurtleBot3 (колесная): `0.2s`
- Rob Box (ICP): `0.5s` — правильно! ✅

---

## 🔗 Next Steps

1. **Применить Priority 1 fixes** (см. выше)
2. **Протестировать на реальном роботе:**
   - Узкие проходы (robot_radius test)
   - Быстрые маневры (acc_lim test)
   - Точное наведение (min_speed_theta test)
3. **Мониторить метрики:**
   - "No valid trajectories" frequency
   - Goal success rate
   - Average time to goal
4. **Если работает хорошо → Priority 2 (aggressive params)**

---

**Составлено:** GitHub Copilot  
**На основе:** TurtleBot3 Burger (ROS 2 kilted), Rob Box ffcff97, Husky/Jackal specs  
**Дата:** 2025-12-16  
