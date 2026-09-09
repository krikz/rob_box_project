# Быстрая проверка исправления AprilTag TF

## 🚀 Развертывание

### Main Pi (10.1.1.20)
```bash
# SSH подключение
sshpass -p 'open' ssh ros2@10.1.1.20

# Обновить и перезапустить robot-state-publisher
cd ~/rob_box_project/docker/main
git pull
docker-compose down robot-state-publisher
docker-compose pull robot-state-publisher
docker-compose up -d robot-state-publisher

# Проверить статус
docker ps | grep robot-state-publisher
docker logs robot-state-publisher --tail 20
```

### Vision Pi (10.1.1.21)
```bash
# SSH подключение
sshpass -p 'open' ssh ros2@10.1.1.21

# Обновить и перезапустить oak-d
cd ~/rob_box_project/docker/vision
git pull
docker-compose down oak-d
docker-compose pull oak-d
docker-compose up -d oak-d

# Проверить статус
docker ps | grep oak-d
docker logs oak-d --tail 20
```

## ✅ Проверка

### 1. Проверить TF дерево (на Main Pi)
```bash
# Сгенерировать PDF дерева TF
sshpass -p 'open' ssh ros2@10.1.1.20 \
  'docker exec rtabmap ros2 run tf2_tools view_frames'

# Проверить конкретную трансформацию
sshpass -p 'open' ssh ros2@10.1.1.20 \
  'docker exec rtabmap ros2 run tf2_ros tf2_echo base_link camera_rgb_camera_optical_frame'

# Ожидаемый результат: должна показаться трансформация без ошибок
```

### 2. Проверить топики AprilTag (на Vision Pi)
```bash
# Проверить что топик /detections публикуется
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'docker exec oak-d ros2 topic list | grep detections'

# Ожидаемый результат: /detections

# Проверить данные на топике (если есть теги в поле зрения)
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'docker exec oak-d ros2 topic echo /detections --once'
```

### 3. Проверить логи RTAB-Map (на Main Pi)
```bash
# Проверить что ошибка исчезла
sshpass -p 'open' ssh ros2@10.1.1.20 \
  'docker logs rtabmap --tail 100 | grep "Cannot transform"'

# Ожидаемый результат: пустой вывод (ошибки нет)

# Посмотреть последние логи
sshpass -p 'open' ssh ros2@10.1.1.20 \
  'docker logs rtabmap --tail 50'
```

## 🎯 Критерии успеха

- ✅ robot-state-publisher запущен и работает
- ✅ oak-d запущен и работает
- ✅ TF трансформация base_link → camera_rgb_camera_optical_frame существует
- ✅ Топик /detections публикуется
- ✅ Ошибка "Cannot transform tag pose" отсутствует в логах RTAB-Map

## 🐛 Отладка

### Проблема: robot-state-publisher не запускается
```bash
# Посмотреть логи
docker logs robot-state-publisher --tail 50

# Проверить URDF генерацию
docker exec robot-state-publisher cat /tmp/rob_box.urdf | grep camera_rgb_camera
```

### Проблема: AprilTag не публикует детекции
```bash
# Проверить что камера публикует изображения
docker exec oak-d ros2 topic hz /camera/camera/color/image_raw

# Проверить логи AprilTag ноды
docker logs oak-d --tail 100 | grep apriltag
```

### Проблема: TF трансформация не существует
```bash
# Посмотреть все TF фреймы
docker exec rtabmap ros2 run tf2_ros tf2_monitor

# Проверить конкретный фрейм
docker exec rtabmap ros2 run tf2_tools view_frames
# Откроется frames.pdf - проверить что camera_rgb_camera_optical_frame связан с base_link
```

---

**Создано**: 2025-11-10  
**Документ**: APRILTAG_TF_FIX_2025-11-10.md
