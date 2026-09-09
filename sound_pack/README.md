# 🎵 RobBox Sound Pack

**Полная библиотека звуков для робота RobBox**

---

## 📊 Статистика

- **Всего звуков:** 51 файл
- **Общая длительность:** 89.83 секунд (~1.5 минуты)
- **Средняя длительность:** 1.76s
- **Диапазон:** 0.061s - 12.353s
- **Общий размер:** 0.71 MB
- **Формат:** MP3 (low quality previews)

### По категориям:
- **Base Library** (базовые звуки): 20 файлов
- **UI Sounds** (интерфейс): 12 файлов  
- **Robot & Sci-Fi** (роботы): 19 файлов

### По длительности:
- Очень короткие (<0.5s): 15 файлов
- Короткие (0.5-1.5s): 19 файлов
- Средние (1.5-3.5s): 11 файлов
- Длинные (3.5-6s): 5 файлов
- Очень длинные (>6s): 1 файл

---

## 📦 Источники

### Базовая библиотека (21 файл)

1. **Robot Expressions Pack** by Audionauten (CC BY 3.0)
   - `confused.mp3`, `confused_alt.mp3`, `happy.mp3`

2. **Small Robot Pack** by charonfaustinus (CC BY 3.0)
   - `affirmation.mp3`, `angry_1.mp3`, `concerned.mp3`, `confirmation.mp3`
   - `cute.mp3`, `error.mp3`, `sigh.mp3`, `surprise.mp3`
   - `talk_1.mp3`, `talk_2.mp3`, `talk_3.mp3`, `talk_4.mp3`
   - `thinking.mp3`, `very_cute.mp3`

3. **Electronic_Effects Pack** by Opossum (CC BY 3.0)
   - `electro_drip_d5.mp3`, `electro_drip_e4.mp3`
   - `robot_drip_a1.mp3`, `robot_drip_d4.mp3`

### Дополнительные звуки (30 файлов)

Скачаны с Freesound.org от 23 авторов:
- **chungus43A** - 4 звука (dot-matrix printers, elevator)
- **hotpin7** - 3 звука (activation, random_ui, bubbles)
- **nugsano** - 2 звука (videogame talk beeps)
- И еще 20 авторов...

**Лицензии:**
- CC0 (Public Domain): 37 файлов
- CC BY 3.0: 5 файлов
- CC BY 4.0: 9 файлов

---

## 🎯 Звуки по категориям

### UI & Interface (12 звуков)

Звуки интерфейса, кнопки, уведомления

| Файл | Длит. | Описание | Триггер |
|------|-------|----------|---------|
| `dot_click.mp3` | 0.06s | Минимальный dot click | `ui_dot` |
| `button_click.mp3` | 0.23s | Короткий клик кнопки | `ui_button` |
| `menu_click.mp3` | 0.25s | Меню клик | `ui_menu_click` |
| `random_ui_sound.mp3` | 0.27s | Случайный UI звук | `ui_random` |
| `note_e_blip.mp3` | 0.40s | Нота E blip | `ui_note_e` |
| `ui_confirm.mp3` | 0.50s | Милое подтверждение | `ui_confirm` |
| `radio_start.mp3` | 0.62s | Включение рации | `ui_radio_start` |
| `roger_beep.mp3` | 0.69s | Roger подтверждение | `ui_roger` |
| `notification.mp3` | 1.00s | Уведомление | `ui_notification` |
| `activation.mp3` | 1.28s | Активация функции | `ui_activate` |
| `elevator_chime.mp3` | 3.59s | Звонок лифта | `ui_chime` |
| `school_bell.mp3` | 5.34s | Школьный звонок | `ui_bell` |

**Использование:**
- Быстрая обратная связь (<0.3s): `dot_click`, `button_click`, `menu_click`
- Подтверждения (0.3-1s): `note_e_blip`, `ui_confirm`, `roger_beep`, `notification`
- Сигналы (>1s): `activation`, `elevator_chime`, `school_bell`

### Robot & Sci-Fi (19 звуков)

Роботизированные звуки, sci-fi эффекты

| Файл | Длит. | Описание | Триггер |
|------|-------|----------|---------|
| `videogame_talk_beep_high.mp3` | 0.10s | Высокий talk beep | `robot_talk_beep_2` |
| `videogame_talk_beep.mp3` | 0.14s | Обычный talk beep | `robot_talk_beep_1` |
| `whoosh.mp3` | 1.00s | Matrix whoosh | `robot_whoosh` |
| `stun_effect.mp3` | 1.03s | Эффект оглушения | `robot_stun` |
| `glitch_error.mp3` | 1.06s | Сбой системы | `robot_glitch` |
| `bubbles.mp3` | 1.19s | Бульканье | `robot_bubbles` |
| `water_fx.mp3` | 1.46s | Всплеск жидкости | `robot_liquid` |
| `c_loop.mp3` | 1.71s | C note loop | `robot_loop` |
| `horror_stinger.mp3` | 2.09s | Horror предупреждение | `robot_alert` |
| `scifi_impact.mp3` | 2.38s | Sci-fi удар | `robot_impact` |
| `power_up.mp3` | 3.26s | Включение/зарядка | `robot_power_up` |
| `dot_matrix_1.mp3` | 3.59s | Принтер #1 | `robot_work_1` |
| `dot_matrix_2.mp3` | 3.59s | Принтер #2 | `robot_work_2` |
| `dot_matrix_3.mp3` | 3.59s | Принтер #3 | `robot_work_3` |
| `fantasy_ui.mp3` | 4.36s | Fantasy UI synth | `robot_fantasy` |
| `terminal_bleeps.mp3` | 4.50s | Винтажный терминал | `robot_terminal` |
| `alien_airplane.mp3` | 5.00s | Инопланетный пролет | `robot_flyby` |
| `stinger_retro.mp3` | 12.35s | Ретро stinger | `robot_stinger` |

**Использование:**
- Имитация речи: `videogame_talk_beep`, `videogame_talk_beep_high`
- Короткие эффекты (1-2s): `whoosh`, `stun_effect`, `glitch_error`, `bubbles`, `water_fx`
- Рабочие звуки (2-5s): `power_up`, `dot_matrix_1-3`, `terminal_bleeps`
- Длинные композиции: `stinger_retro` (драматический акцент)

### Base Library (20 звуков)

Базовая библиотека эмоций и реакций робота

| Файл | Длит. | Описание | Триггер |
|------|-------|----------|---------|
| `talk_4.mp3` | 0.62s | Речь #4 | `robot_talk_4` |
| `talk_2.mp3` | 0.63s | Речь #2 | `robot_talk_2` |
| `talk_3.mp3` | 0.64s | Речь #3 | `robot_talk_3` |
| `confused.mp3` | 0.67s | Озадаченность | `robot_confused` |
| `very_cute.mp3` | 0.68s | Очень мило | `robot_very_cute` |
| `cute.mp3` | 0.69s | Мило | `robot_cute` |
| `confirmation.mp3` | 0.75s | Подтверждение | `robot_confirm` |
| `error.mp3` | 0.77s | Ошибка | `robot_error` |
| `thinking.mp3` | 0.78s | Думание | `robot_thinking` |
| `surprise.mp3` | 0.81s | Удивление | `robot_surprise` |
| `talk_1.mp3` | 0.90s | Речь #1 | `robot_talk_1` |
| `concerned.mp3` | 0.95s | Обеспокоенность | `robot_concerned` |
| `robot_drip_a1.mp3` | 1.00s | Drip A1 | `robot_drip_a1` |
| `robot_drip_d4.mp3` | 1.02s | Drip D4 | `robot_drip_d4` |
| `angry_1.mp3` | 1.07s | Злость | `robot_angry` |
| `sigh.mp3` | 1.23s | Вздох | `robot_sigh` |
| `affirmation.mp3` | 1.25s | Утверждение | `robot_affirm` |
| `confused_alt.mp3` | 1.50s | Сильное замешательство | `robot_confused_alt` |
| `electro_drip_e4.mp3` | 1.72s | Electro drip E4 | `robot_drip_e4` |
| `happy.mp3` | 2.15s | Радость | `robot_happy` |
| `electro_drip_d5.mp3` | 3.39s | Electro drip D5 | `robot_drip_d5` |

**Использование:**
- Эмоции: `happy`, `angry_1`, `concerned`, `surprise`, `sigh`
- Реакции: `confused`, `confused_alt`, `cute`, `very_cute`
- Подтверждения: `affirmation`, `confirmation`, `error`
- Имитация речи: `talk_1-4`
- Думание: `thinking`

---

## ⚙️ Интеграция в sound_node.py

### Пример структуры SOUND_DESCRIPTIONS:

```python
SOUND_DESCRIPTIONS = {
    # UI sounds
    "menu_click": "Simple menu click for UI (ui-hater2012)",
    "button_click": "Short button click (NightDrawr)",
    "dot_click": "Tiny dot click sound (TommyListens)",
    "ui_confirm": "Cute UI confirm sound (Feraly_)",
    "activation": "Hi-tech activation sound (hotpin7)",
    
    # Robot sounds
    "power_up": "Robot power up/charging (gulfstreamav)",
    "glitch_error": "Computer glitch error (SilverIllusionist)",
    "terminal_bleeps": "Vintage terminal text output (subetha2026)",
    "videogame_talk_beep": "Video game dialogue beep (nugsano)",
    
    # Base emotions
    "happy": "Happy cheerful robot melody (Audionauten)",
    "confused": "Confused robot questioning sound (Audionauten)",
    "angry_1": "Angry robot emotional sound (charonfaustinus)",
    "thinking": "Robot thinking/processing sound (charonfaustinus)",
    # ... и т.д. для всех 51 звука
}
```

### Группы для random selection:

```python
SOUND_GROUPS = {
    "ui_clicks": ["dot_click", "button_click", "menu_click"],
    "confirmations": ["ui_confirm", "confirmation", "roger_beep"],
    "robot_talk": ["videogame_talk_beep", "videogame_talk_beep_high", 
                   "talk_1", "talk_2", "talk_3", "talk_4"],
    "robot_work": ["dot_matrix_1", "dot_matrix_2", "dot_matrix_3", 
                   "terminal_bleeps", "power_up"],
    "emotions_positive": ["happy", "cute", "very_cute", "affirmation"],
    "emotions_negative": ["angry_1", "concerned", "sigh", "error"],
    "confused_sounds": ["confused", "confused_alt", "thinking"],
}
```

---

## 🎬 Рекомендации по использованию

### Быстрая обратная связь (<0.5s)
**Когда:** Клики, наведение, мгновенная реакция  
**Звуки:** `dot_click`, `videogame_talk_beep_high`, `button_click`, `menu_click`, `note_e_blip`

### Подтверждения (0.5-1.5s)
**Когда:** Успешное выполнение команды, OK  
**Звуки:** `ui_confirm`, `roger_beep`, `confirmation`, `notification`, `activation`

### Эмоции короткие (0.5-1.5s)
**Когда:** Быстрая эмоциональная реакция  
**Звуки:** `cute`, `very_cute`, `confused`, `error`, `surprise`, `thinking`

### Эмоции длинные (1.5-3s)
**Когда:** Выраженная эмоция, радость, злость  
**Звуки:** `happy`, `confused_alt`, `angry_1`, `sigh`, `affirmation`

### Рабочие звуки (2-5s)
**Когда:** Обработка данных, печать, работа системы  
**Звуки:** `power_up`, `dot_matrix_1-3`, `terminal_bleeps`, `fantasy_ui`

### Предупреждения и алерты (2-3s)
**Когда:** Ошибка, опасность, критическая ситуация  
**Звуки:** `horror_stinger`, `scifi_impact`, `glitch_error`

### Длинные композиции (>5s)
**Когда:** Драматический акцент, начало/конец задачи  
**Звуки:** `school_bell`, `stinger_retro`, `alien_airplane`

---

## 📄 Дополнительные файлы

В этой директории:
- **sound_catalog.json** - полный каталог с метаданными (duration, author, license, usage)
- **measure_durations.py** - утилита для измерения длительности (mutagen)
- **51 MP3 файла** - все звуки робота

---

## 📝 Авторы и лицензии

**Базовые паки:**
- **Audionauten** - Robot Expressions (CC BY 3.0)
- **charonfaustinus** - Small Robot (CC BY 3.0)
- **Opossum** - Electronic_Effects (CC BY 3.0)

**Дополнительные (Freesound.org):**
- chungus43A, hotpin7, gulfstreamav, subetha2026, SonosFreesound
- TannerSound, TheSoundLibrary, stano458, ui-hater2012, joeffl
- nugsano, P4INKILLA, MATUSTRM, 114802300, NightDrawr
- OverlookHotelRecords, kolel, SilverIllusionist, pedr01
- TommyListens, KerDwyn, JustASeriesofFonetics, Feraly_

**Все звуки можно использовать в коммерческих проектах!**

---

**Обновлено:** 29 января 2026  
**Проект:** Rob Box Robot  
**Модуль:** rob_box_voice
