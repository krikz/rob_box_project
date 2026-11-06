---
status: posted
media: docs/channel/media/nodes.png
not_before: 2026-12-01T10:00:00Z
message_id: 16
hashtags: #железо #raspberrypi #какэтоработает
posted_at: 2026-11-06T08:59:39.279109+00:00
---
🧩 <b>Три Raspberry Pi — три узла</b>

Всё на борту разложено по трём Raspberry Pi 5, и у каждого свой docker-compose.

<b>Main Pi</b> — движение и карта: привод (VESC по CAN), лидар, RTAB-Map, Nav2, защита по лидару.
<b>Vision Pi</b> — зрение и лицо: камера OAK-D-Lite, распознавание лиц, светодиодные матрицы.
<b>Sound Pi</b> — голос и музыка: диалог, SuperCollider, супервизор и бот этого канала.

Список контейнеров взят из compose-файлов в репозитории; адреса сети не публикуем.

#железо #raspberrypi #какэтоработает
