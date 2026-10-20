# Паки сэмплов для лофи-стиля и именованных инструментов: источники, лицензии, подключение

Дата: 13.10.2026. Карточка: #3568 (ADR-0157 PR-10b; §2.4, §3.4), приоритет от 13.10 — переделка лофи упёрлась в отсутствие звуков.
Тип документа: исследование + план. Файлов в репозиторий не скачивалось, кода фетчеров нет, на робота не ходили.

## 0. Что проверено, что нет

Проверено **открытием страницы источника в этой сессии 13.10.2026** (WebFetch / GitHub API / `raw.githubusercontent.com`):

- лицензия каждого рекомендуемого кандидата (цитата ≤ 15 слов со страницы автора или `LICENSE` в репозитории);
- составы паков (списки файлов из `git/trees/...?recursive=1`, имена нот из имён файлов), размеры из API/страниц;
- формат и длительность файлов Freesound (со страницы звука).

**Не проверено** (подробно — §6): ничто не слушалось, ни одного файла не скачано; качество, тон, шум, зацикливаемость, реальный нотный диапазон
(по имени файла), сегодняшнее свободное место на Vision Pi, доступность скачивания Freesound без логина, соглашение об октавах в именах нот.

Агрегаторы (beatsyncpro, kvraudio, блоги) как источник лицензий не использовались. Страница `sfzinstruments.github.io` — каталог проекта; её
колонка «лицензия» сверена с `LICENSE`/README самого репозитория там, где мы рекомендуем пак.

## 1. Категории и результат одной строкой

| Кат. | Нужно | Лучший честный вариант | Лицензия | Статус |
|---|---|---|---|---|
| а | лофи/хип-хоп кит или мягкие брейки | два лупа holizna (95 и 85 BPM) + лофи-обработка уже подключённых паков | CC0 | кита нет, есть лупы; кит не найден (см. §2а) |
| б | виниловый треск, шипение плёнки | kyles «vinyl record crackle nice stereo» + D.S.G. «Tape Hiss» | CC0 | готово к подключению |
| в | шум дождя | samesamesame «Rain from Indoors - Perfect loop» | CC0 | готово к подключению |
| г | тёплые клавиши | Greg Sullivan Wurlitzer EP200 (Rhodes с разрешённой лицензией нет) | CC-BY 3.0 | нужна атрибуция |
| д | балалайка, арфа, тромбон, ксилофон, флейта, орган | арфа, тромбон, ксилофон, флейта, орган — VSCO 2 CE / VCSL; **балалайка — пака нет** | CC0 | балалайка остаётся `synth=None` |

## 2. Кандидаты по категориям

Обозначения: «ком.» — коммерческое использование, «мод.» — модификация (в т.ч. ресемпл/обработка), «пер.» — распространение (зеркалирование
на наш registry/хост). «Нужна атрибуция» — требует записи в `docs/music/ATTRIBUTION.md` (как MuldjordKit).

### 2а. Лофи / хип-хоп ударные

| Кандидат | URL | Лицензия (со страницы) | ком./мод./пер. | Формат, размер | Что внутри | Качество |
|---|---|---|---|---|---|---|
| **holizna: 85 BPM Lofi HipHop Drum Loop** | <https://freesound.org/people/holizna/sounds/621177/> | CC0: «copy, modify, distribute and perform the sound, even for commercial purposes» | да/да/да | wav 44.1 кГц/24 бит стерео, 34 с, 8.6 МБ | 85 BPM, теги lofi, mellow, chill | не слушано |
| **holizna: Boom Bap Hiphop Kick Snare Loop 95 BPM** | <https://freesound.org/people/holizna/sounds/629142/> | CC0 (та же формулировка) | да/да/да | wav **192 кГц/32 бит** стерео, 13.7 с, 20.1 МБ | 95 BPM, кик+снейр; «left over loops» из его паков | не слушано; формат избыточен, ресемплить |
| Karoryfer **Big Rusty Drums** | репо <https://github.com/sfzinstruments/karoryfer.big-rusty-drums> (страница автора отдала 429 — не открыта) | `LICENSE` репо = CC0 1.0 (открыт), API: CC0-1.0 | да/да/да | sfz/nki + wav, репо ~603 МБ (API) | кит; состав и характер звука не читались | не слушано, характер «lo-fi» не подтверждён |
| Karoryfer Swirly Drums | репо `sfzinstruments/karoryfer.swirly-drums` | API: CC0-1.0 (файл `license` в репо есть, текст не открывался) | да/да/да* | репо ~840 МБ | кит; не читали | не слушано |
| Уже подключённые: Sonic Pi (CC0), MuldjordKit (CC-BY 4.0) | `manifest.yaml`: `sonicpi-samples`, `muldjord-kit` | см. ADR-0153 В3 | — | 35 МБ / 7 МБ на хосте | амен, `loop_breakbeat`, `bd_*`, `elec_*` | на лофи — только через обработку |

Отброшено:

- **ritchse/tidal-drum-machines** (LinnDrum, SP-12, MPC и др. — самый подходящий по звуку источник): README не содержит лицензии, у репо нет
  лицензии в API → «лицензия неясна», нельзя (то же записано в `dj_dave_samples.lock.json`).
- **Salamander Drumkit** — CC-BY-**SA** 3.0 (сэмплы-производные обязаны нести SA) → не берём, пока есть CC0.
- **Sam's Sonor** — CC-BY-SA 4.0, по той же причине. **SM Drums** — «Public Domain» по каталогу, но репо без лицензии в API и 2.3 ГБ → не проверялось.
- Готовые лофи-паки магазинов (Wave Alchemy, Samples From Mars и т.п.) — коммерческие.
- *Swirly/Big Rusty: страницы автора не открылись, лицензия взята из `LICENSE`/API репо (первичный источник), но состав не читали.

**Вывод по а.** Честно: **готового CC0-лофи-кита (одиночные удары) я не нашёл**. Есть два лупа holizna как «мягкий брейк» (роль `break` в
каталоге; темп лупа известен, `rate = bpm/tempo`, как у Sonic Pi) и возможность «состарить» уже стоящие паки обработкой в SynthDef/`loop`
(lpf ~ 4–6 кГц, лёгкий bitcrush, тихий crackle поверх). Кит-кандидат (Big Rusty Drums) — решение только после прослушивания.

### 2б. Виниловый треск и шипение плёнки

| Кандидат | URL | Лицензия | Формат, размер | Длина | Заметка |
|---|---|---|---|---|---|
| **kyles: vinyl record crackle nice stereo.flac** | <https://freesound.org/people/kyles/sounds/637931/> | CC0 («copy, modify, distribute and perform the sound, even for commercial purposes») | flac 48 кГц/24 бит стерео, 12.2 МБ | 74.7 с | рекомендация; ровный треск, длинный |
| Anthousai: vinyl - out 05.wav | <https://freesound.org/people/Anthousai/sounds/398817/> | CC0 (та же формулировка) | wav 44.1/24 стерео, 2.4 МБ | 9.7 с | «треск после конца пластинки» — для всплеска в конце/паузах |
| scottyhigh20: Vinyl Crackle 1.wav | <https://freesound.org/people/scottyhigh20/sounds/493122/> | CC0 | wav 44.1/24 стерео, 7.4 МБ | 29.3 с | запасной |
| **Шипение: D.S.G. Tape Hiss** | <https://freesound.org/people/D.S.G./sounds/241415/> | CC0 | flac 44.1/24 стерео, 1.5 МБ | 9.4 с | рекомендация; с кассетной деки |
| kijjaz: Tape Hiss from Blank Tape - Dolby B-NR | <https://freesound.org/people/kijjaz/sounds/430975/> | CC0 | wav 48/24 стерео, 13.6 МБ | 49.6 с | запасной, длиннее |

Отброшены: `deleted_user_2906614` «Vinyl snap, crackle and pop» (самый скачиваемый, CC0 по фильтру, но автор удалён — страницу не открывал, происхождение
не проверить). Остальные результаты поиска по фильтру CC0 не открывались поштучно.

Рекомендация: **kyles + D.S.G.** (оба flac, CC0, суммарно ~14 МБ). Это подложка (бесшовный луп нужно проверить на стыке — не проверено).

### 2в. Шум дождя

| Кандидат | URL | Лицензия | Формат, размер | Длина | Заметка |
|---|---|---|---|---|---|
| **samesamesame: Rain from Indoors - Perfect loop** | <https://freesound.org/people/samesamesame/sounds/242889/> | CC0 1.0 | wav 44.1/24 стерео, 12.1 МБ | 48 с | рекомендация: заявлен «perfect loop», «как из комнаты» — подходит под лофи |
| unfa: Slowly Raining Loop | <https://freesound.org/people/unfa/sounds/177479/> | CC0 | flac 44.1/16 стерео, 4.8 МБ | 73 с | запасной; капли по полу балкона, 10-й этаж, Варшава |
| speakwithanimals: Rain Slowly Passing TREATED LOOP | <https://freesound.org/people/speakwithanimals/sounds/525046/> | CC0 | wav 48/24 стерео, **228.5 МБ** | 13 мин 52 с | компрессирован «для ровной громкости»; слишком тяжёл → отброшен по размеру |

### 2г. Тёплые клавиши (Rhodes / Wurlitzer / мягкое пиано)

| Кандидат | URL | Лицензия | ком./мод./пер. | Формат, размер | Ноты / велосити | Качество |
|---|---|---|---|---|---|---|
| **Greg Sullivan: Wurlitzer EP200** (в репо «E-Pianos») | <https://github.com/sfzinstruments/GregSullivan.E-Pianos> | README: «Creative Commons Attribution 3.0 Unported License» | да/да/да, **нужна атрибуция** | flac, 42 файла, **2.3 МБ** (каталог `Wurlitzer EP200`) | A1…Ab6 по именам (a1, ab2, ab3, ab4, ab6, b2, b4, c2, db4–db6, e2–e4, eb3, f2, g4, g5, gb4), 4 динамики (pp, mp, f, ff) | не слушано; малый размер = сэмплы коротки, стык звучит «как гитара» — проверить |
| Greg Sullivan: Hohner Pianet T / Yamaha CP80 | тот же репо | CC-BY 3.0 | да/да/да | 7.9 МБ / 10.8 МБ | Pianet: F1…E6 (в имени MIDI-номер), 2 динамики | не слушано |
| **VCSL: TX81Z «FM Piano»** (FM-«электропиано» с хардвера) | <https://github.com/sgossner/VCSL> → `Electrophones/TX81Z` | `LICENSE` = CC0 1.0; README: «no royalties, no credit, no special terms» | да/да/да | wav, 66 файлов, 64 МБ | C0…G#6 с шагом в большую терцию, 3 велосити | не слушано |
| **VCSL / VSCO: Upright Piano No.1** (мягкое пианино) | VSCO: `Keys/Upright Nr1` (80 файлов, 147 МБ); VCSL: `Upright Piano, Yamaha` (143 файла, 236 МБ) | CC0 (`LICENSE` обоих репо) | да/да/да | wav | 2–3 велосити, часть — release-сэмплы | не слушано |
| Headroom Piano (Bengt Nilsson) | каталог sfz: CC-BY-4.0 | CC-BY 4.0 (репо API: CC-BY-4.0) | да/да/да, атрибуция | 137 МБ | Yamaha C3, слишком «концертный» для лофи | не слушано, не читали |

Отброшено:

- **jRhodes3c / jRhodes3d** (Jeff Learman, Rhodes Mark I Stage 73 — самый нужный по звуку): страница jRhodes3c: «CC-BY-NC-SA-4.0»; каталог
  sfz: jRhodes3d — «CC-BY-NC-4.0». **NC → отброшено.** Рабочего Rhodes в CC0/CC-BY я не нашёл.
- **Salamander Grand** (CC-BY 3.0, 394 МБ…1.18 ГБ, 16 слоёв) — лицензия допустима, но это концертный рояль, не лофи-мягкость, и тяжёл; запасной.
- LivingRoom Upright (CC-BY-**SA** 4.0), Maestro Concert Grand (лицензия «Custom»), Splendid Grand («Public Domain» по каталогу, репо без лицензии) —
  SA / неясно.
- ETI Roads, Fracture Blueprint Electric Keys (бесплатные Kontakt-библиотеки, Rhodes) — не SFZ/Kontakt-only, лицензия в выдаче не названа.

### 2д. Именованные инструменты (ADR-0157 §3.4)

Источник по умолчанию — **VSCO 2 Community Edition** (репо `sgossner/VSCO-2-CE`, API-лицензия CC0-1.0, `LICENSE` = «CC0 1.0 Universal»)
и **VCSL** (`sgossner/VCSL`, README: «Essentially it's Public Domain»). Оба — wav, СС0, мультисэмплы по нотам. Страница
<http://vis.versilstudios.net/vsco-community.html> не открылась (просрочен сертификат); лицензия взята из `LICENSE` репо.

| Инструмент | Кандидат | Лицензия | Файлы / размер | Ноты (по именам файлов) | Слои |
|---|---|---|---|---|---|
| **ксилофон** | VCSL `Idiophones/Struck Idiophones/Xylophone` (есть и VSCO `Percussion/Xylo`, 8 файлов, 4 МБ) | CC0 | 48 wav, 25 МБ | G3 C4 G4 C5 G5 C6 G6 C7 (шаг в квинту/кварту; ресемпл между ними) | Soft/Medium/Hard × pp/ff |
| **арфа** | VSCO `Strings/Harp` (23 wav, 33 МБ) / VCSL `Concert Harp` (45 wav, 73 МБ; те же KSHarp, +слой f) | CC0 | см. слева | B1…D7, 23 ноты (шаг ≈ терция) | mf (VCSL: mf+f) |
| **тромбон** | VSCO `Brass/Tenor Trombone` (99 wav, 81 МБ); ещё есть `OldTrombone` (389 wav, 292 МБ, Buzz/и т.д.) — не берём | CC0 | 99 wav, 81 МБ | A#0…D3 по именам (17 нот; **соглашение об октавах не проверено**), артикуляции stac/vib/… | v1–v4, rr1–2 |
| **флейта** | VSCO `Woodwinds/Flute` (94 wav, 114 МБ; `LDFlute_expvib`, `susvib`) | CC0 | 94 wav, 114 МБ | A3…E5 и др. (диапазон не сведён) | v1, rr 1–2 |
| **орган** | VSCO `Keys/Organ` = VCSL `Pipe Organ` (66 wav, 138 МБ); ноты C1…C6 через терцию + `Rode_Blower` | CC0 репо; `Info.txt`: «right to redistribute» от Versilian, «credit … encouraged» | 66 wav, 138 МБ | A1…C6 (21 нота) | несколько регистров (Man3Quiet и т.п.) |
| **пианино** (см. §2г) | VSCO `Upright Nr1` | CC0 | см. §2г | | |
| **гитара** | Karoryfer **Shinyguitar / Emilyguitar** (CC0, 360/101 МБ по API репо) — состав не открывали | CC0 по API | | | *не исследовано* |
| **балалайка** | **пака нет** (см. ниже) | | | | |

Нотное покрытие у всех VSCO/VCSL — «каждая 2-я–4-я нота»: между опорными сэмплами нужно ресэмплить с `rate = 2^(Δполутонов/12)` (без формант-коррекции).

**Балалайка — результат поиска.** На Freesound (CC0-фильтр, поиск «balalaika») 4 звука, все открыты и прочитаны:
`idabrandao/382985` (6 с, струны сувенира «через микрофон компьютера»), `claywh/698926` (**мэшап двух чужих CC0-звуков, автор сам пишет, что «не настоящая балалайка»**),
`cottager/91983` (14-струнная гитара DADGAD «под этнический инструмент»), `FullMetalJedi/353266` (15 с, фраза, Strat/«Funny»). Это фразы, а не мультисэмплы
по нотам. Без CC0-фильтра (CC-BY и др.) найдено ещё три результата (`paul_sutyrin/262454`, `zacnie/646300`, `boojum/2613`) — лицензию и содержание **не
открывал**. Коммерческие: Ilya Efimov Balalaika Prima (All Rights Reserved), Precisionsound Russian Balalaika (платная). FreePats: балалайки и мандолины
в списке банков нет. Sfz-каталог: «Terkelsen Mandolin» — лицензия не указана → нельзя.
**Вывод: пака нет → `INSTRUMENTS["balalaika"].synth` остаётся `None` с соседями из семьи pluck** (как решено в ADR-0157 §2.4). Альтернатива без лицензионных вопросов —
собственный SynthDef (Karplus-Strong + тремоло щипка), см. §4.

### 2е. Источники, которые смотрели и отбросили или отложили целиком

| Источник | Что нашли | Решение |
|---|---|---|
| **Philharmonia Orchestra samples** (<https://philharmonia.co.uk/audio-sound-samples/>) | «free to use», но: «must not be sold or made available 'as is'» (сэмплы/сэмплер-инструмент) | **отброшено для пака**: хост-зеркало, играющее как сэмплер, подпадает под «made available as is» (формат файлов страница не называет). Использовать как справочник |
| **Sonatina Symphonic Orchestra** | «CC Sampling Plus 1.0», 1.39 ГБ; арфы нет (страница) | **отброшено**: Sampling Plus разрешает копировать работу целиком только некоммерчески и запрещает рекламу; CC лицензию отозвал |
| **University of Iowa MIS** (<https://theremin.music.uiowa.edu/MIS.html>) | «may be downloaded and used for any projects, without restrictions»; флейта, тромбон, ксилофон, маримба, вибрафон; арфы нет | **резерв** (не стандартная лицензия, а свободный текст; хромаческие гаммы pp/mf/ff, но монофония, анехоик «сухо»); берём только если VSCO не устроит на слух |
| **FreePats** (<https://freepats.zenvoid.org/>) | банки: Xylophone, Pipe Organ, Electric Organ, Harp, Recorder, Ocarina, Kalimba и др.; Rhodes/Wurlitzer/тромбон/балалайка нет | лицензии по банкам **не проверялись** (страница главной лицензий не содержит); качество GM-уровня; не берём |
| **sfz-каталог** прочее | гитара-банджо `ganjo` (CC0), `Kay 5 String Banjo` (GPL-3.0), `MF Tin Whistle` (CC BY-NC-SA — отброшено), `Ixox Flute` (CC-BY 4.0, 9 МБ, синт-флейта) | `Ixox Flute` — запасная флейта с атрибуцией; остальное не нужно |
| **archive.org** | не исследовалось | — |
| Salamander Grand | см. §2г | запас |

## 3. Рекомендация: один кандидат на категорию

| Кат. | Рекомендация | Почему | Цена |
|---|---|---|---|
| а | **holizna 95 BPM + 85 BPM** (CC0) как роль `break` + лофи-обработка в `loop`-синте | единственное явно «lofi/boom-bap» CC0, что нашёл; кита нет — не выдумываю | ~28 МБ исходник (192 кГц/32 бит), после ресемпла на сборке пака — оценка ~5–8 МБ |
| б | **kyles crackle + D.S.G. tape hiss** (CC0) | flac, CC0, длинный бесшовный на слух материал | ~14 МБ |
| в | **samesamesame «Perfect loop»** (CC0) | единственный с заявленной бесшовностью; «из комнаты» = лофи | 12 МБ |
| г | **Greg Sullivan Wurlitzer EP200** (CC-BY 3.0) | 4 динамики, малый размер; Rhodes без NC нет. Мягкое пианино — VSCO Upright Nr1 (CC0) | 2.3 МБ + ~40–150 МБ |
| д | **VSCO 2 CE (CC0)**: ксилофон/арфа/тромбон/флейта/орган; **балалайка — нет** | один репозиторий, одна лицензия, wav, имена нот в файлах | см. §5 |

## 4. Как это ляжет в нашу механику

### 4.1 Доставка (ресурсный пак)

Существующий механизм (`docker/vision/scripts/resource_pack/manifest.yaml`, `fetch_sample_pack.py`, `*.lock.json`; ADR-0125/0126):

- запись `kind: sample-pack`, `type: open`, `fetch_hook: fetch_<имя>`, `sha256: ""` (запрещён у hook-записей), пофайловые sha256 и размеры — в
  `<имя>.lock.json` на **пиннутом коммите**; маркер `.complete`; `target: /opt/rob_box/samples/<имя>`; `required: soft`, `on_missing: degrade`;
- новый хук на каждый тип источника: для GitHub-паков (Wurlitzer, VSCO/VCSL) — как `fetch_muldjord_kit`: выборка файлов по пиннутому SHA через raw.githubusercontent;
  в lock — только нужные файлы (не клон на сотни МБ);
- **Freesound не отдаёт файл без логина** (вход/API-ключ; доступность прямой ссылки без логина **не проверялась**). Все файлы Freesound под CC0 — копировать и распространять
  можно, поэтому план: **один раз зеркалировать** выбранные ~8 файлов (после ресемпла) в наш registry/release-артефакт и пиннить sha256 там; в lock — URL нашего зеркала.
  Атрибуция CC0 не нужна, но имя автора и ID звука кладём в lock как метаданные;
- **CC-BY 3.0 (Wurlitzer)** — запись в `docs/music/ATTRIBUTION.md` (автор Greg Sullivan, мапинг kinwie) до включения в палитру, как для MuldjordKit;
- форматы: на роботе вывод 16 кГц (ReSpeaker) — wav 44.1/16 бит достаточно; при сборке lock конвертируем 24/32-бит и 192 кГц вниз (**sha256 считаем с конвертированного**, исходный — в метаданных).

### 4.2 Каталог

`scripts/music/build_sample_pack_catalog.py` строит `data/sample_<пак>.json` из lock + файлов (роль по таблице правил, длительность, пик/RMS через soundfile). Для новых паков:

- ударные/лупы (holizna) — роль `break`, длина в долях → `PACK_BREAK_BEATS` (`knowledge.py`, как `sonicpi_loop_amen`);
- подложки (треск, шипение, дождь) — новая роль, например `bed`/`ambience`: **таблица ролей и `SAMPLE_ROLES` её не знают** (в `ROLES` каталога: kick…fx). Нужно расширить таблицу
  и решить, как подложка попадает в микс (тихий слой под всё, не сайдчейнится);
- мультисэмплы по нотам — это **не плоский список файлов**: нужны поля `root_midi`, `velocity`, `rr`, `inst`. Текущая схема каталога (`SampleInfo`) корня ноты не хранит.

### 4.3 Игра мультисэмпла по нотам: есть ли путь у нас

Проверено grep'ом по `render/`, `arrange/samples.py`, `custom_synthdefs/`, `renardo_adapter.py`:

- Тональные партии — **именованные SynthDef-ы** (`slot >> <synth>(ноты, ...)`, `render/renardo.py::_tonal_line`), SynthDef-ы лежат в `docker/vision/voice_assistant/custom_synthdefs/*.scd`
  (retrobass, imperialbrass, …) и копируются в образ `/ws/custom_synthdefs` (Dockerfile:272).
- Сэмплы играют **только** через `loop('<путь>')` (`_sample_line`): `PlayBuf` с постоянной скоростью (`rate = bpm/tempo` для лупа), без выбора файла по высоте ноты.
- Буферы грузятся по пути: `engine/renardo_adapter.py:159–168` (`Samples.loadBuffer('../../<путь>')`, `0` = файла нет, тишина без ошибки).

**Пути «ноты → сэмпл» нет.** Нужно (на будущее, не в этой карточке):

1. Новый SynthDef-сэмплер на инструмент (например `xylo`, `harp`, `tbone`, `flute`, `organ`, `wurli`) в `custom_synthdefs/`: `PlayBuf.ar(2, bufnum_близайшей_ноты, rate: freq / root_freq, ...)`, огибающая в пределах `sus` (как `SAMPLE_EDGE_S`, #3432).
   Выбор буфера — по `freq`: либо массив буферов и `Select`/`Index` по ближайшему корню, либо одна нота — один SynthDef-вызов; велосити — по `amp`.
2. Загрузка набора буферов инструмента при старте (тот же `loadBuffer`, список — из каталога, критичные — в `CRITICAL_SYNTHS`).
3. Тогда `INSTRUMENTS[<ключ>].synth` = имя нового SynthDef (тест таблицы PR-1 требует синт из `SYNTH_PALETTE["lead"]`), строка в `SYNTH_PALETTE`, замер NRT
   (`loudness_nrt_v2.py`, `LAYER_MEASURED_DB`/`LAYER_BANDS`, координатор на katana — ADR-0152 §4).
4. Подложки: `loop`-синт с большим `sus` (PlayBuf `loop: 1`) уже умеет играть длинный луп; тон — отдельный новый слой (уровень, полосы — NRT).

Оценка трудоёмкости: SynthDef-сэмплер — один шаблон на все инструменты (разные только каталог нот), т.е. не 6 разных кодов.
Альтернатива для балалайки без паков — Karplus-Strong SynthDef (в палитре уже есть `karp`, `pluck`; именованной «балалайки» нет — честная фраза остаётся).

### 4.4 Объём на Vision Pi (оценка по размерам файлов репо)

| Блок | Полный состав | Выборка (один слой, без release) |
|---|---|---|
| Wurlitzer EP200 | 2.3 МБ | 2.3 МБ |
| Треск + шипение | 14 МБ | 8–14 МБ |
| Дождь | 12 МБ | 5–12 МБ |
| Лупы holizna (после ресемпла) | 28 МБ исходник | ~6 МБ (оценка) |
| Ксилофон | 25 МБ | ~10 МБ |
| Арфа (VSCO) | 33 МБ | 33 МБ (уже mf) |
| Тромбон | 81 МБ | ~25 МБ (одна артикуляция) |
| Флейта | 114 МБ | ~40 МБ |
| Орган | 138 МБ | ~50 МБ |
| Пианино (Upright Nr1) | 147 МБ | ~40 МБ |
| **Итого** | **~595 МБ** | **~220–230 МБ** (оценки; доли — от доли артикуляций/слоёв по именам файлов) |

Для сравнения: уже стоящие на хосте `renardo-samples` ≈ 600 МБ, `dj-dave-samples` 39 МБ, `sonicpi-samples` 35 МБ (`manifest.yaml`). **Свободное место на Vision Pi не измерялось** (по задаче на робота не ходил) — перед включением снять `df -h /opt/rob_box`.
В RAM: SC держит буфер на ноту; ~100 сэмплов × 1–3 МБ (16 бит стерео) — десятки МБ на инструмент; нагрузку на Pi не мерили.

## 5. Порядок включения (предложение)

1. Мелкие и решающие лофи: треск+шипение+дождь (CC0, ~26 МБ), два лупа — нужна роль `bed` и зеркало Freesound. Лофи можно переделывать на них, не дожидаясь мультисэмплов.
2. Wurlitzer (2.3 МБ, CC-BY) — первый тональный сэмплер: проверяет шаблон SynthDef на минимуме данных. Запись в ATTRIBUTION.
3. VSCO: ксилофон (мал, сразу приёмка «Баха на ксилофоне») → арфа → тромбон → флейта → орган. Каждый — карточка: lock, фетчер-хук/запись манифеста, каталог, SynthDef, замер NRT.
4. Балалайка: решение Шифу — оставить честное «нет пака», либо KS-SynthDef.

Гейты каждого пакета: пофайловый sha256 в lock, `.complete`, `required: soft`, запись в `ATTRIBUTION.md` (для CC-BY), проверка на роботе (по `hotpatch.sh`, ADR-0157 §5).

## 6. Что не проверено

- **Ничего не прослушано.** «Тёплый», «лофи», «бесшовный», «перфект-луп» — слова авторов с их страниц, не наши. Нужно послушать kyles/D.S.G./samesamesame на цикле, Wurlitzer по ходу октав, ксилофон между опорными нотами при ресемпле.
- Скачивание Freesound без логина/API-ключа (формулировка выше — предположение; страница звука показывает формат/размер, но не механику скачивания).
- Реальные нотные диапазоны VSCO/VCSL — взяты **из имён файлов**; соглашение об октавах (C3 vs C4 как средняя) не сверено, нужно читать `.sfz`-маппинги (`sfz`-ветка репо VSCO; у релиза 1.1.0 в API нет ассетов).
- VCSL `Pipe Organ`: `Info.txt` — «granted right to redistribute by Versilian Studios», «credit … encouraged»; репо целиком CC0. Юридическую чистоту «авторы сэмпла» (Ivy Audio) отдельно не изучал — при сомнениях держать атрибуцию (она ничего не стоит).
- Лицензии Big Rusty/Swirly Drums, Shinyguitar, Emilyguitar — из `LICENSE`/API репо (первичного источника), страницы автора (shop.karoryfer.com) отдали 429; состав и звук не читались.
- Лицензии каждого банка FreePats; содержимое Freesound-результатов, которые не открывал (`deleted_user_2906614`, `paul_sutyrin`, `zacnie`, `boojum`, `hermeshermeshermes`, остальные страницы поиска).
- Свободное место и память на Vision Pi; нагрузка SC на загрузку ~100 буферов на инструмент.
- archive.org и другие первичные архивы не обходились. Ритм-машины (tidal-drum-machines) — лицензия неясна, не использовать.
- Юридическое заключение по CC-BY 3.0 (Wurlitzer) для «производной» — ресемпл/питч считаем модификацией, разрешённой лицензией (CC-BY разрешает адаптацию с указанием); не юридическая консультация.
- Приёмка карточки #3568 (на роботе: `instrument=xylophone` → `synth=<новый>`) **не выполнена** — это исследование, подключения нет.

## 7. Ссылки (все открыты 13.10.2026, кроме помеченных в §2 как неоткрытые)

- Greg Sullivan E-Pianos: <https://github.com/sfzinstruments/GregSullivan.E-Pianos> · каталог: <https://sfzinstruments.github.io/pianos/greg_sullivan_e-pianos>
- VCSL: <https://github.com/sgossner/VCSL> (README, `LICENSE`)
- VSCO 2 CE: <https://github.com/sgossner/VSCO-2-CE> (`LICENSE`, дерево репо)
- jRhodes3c (отброшен): <https://sfzinstruments.github.io/pianos/jrhodes3c/>
- Каталоги sfzinstruments: <https://sfzinstruments.github.io/> (pianos, keyboards, orchestra, brass, melodic-percussion, folk, woodwinds, misc, drums, drum_machines, percussion)
- Freesound: страницы звуков, перечисленные в таблицах §2 (все с префиксом `https://freesound.org/people/<автор>/sounds/<id>/`)
- Philharmonia: <https://philharmonia.co.uk/audio-sound-samples/> · Iowa MIS: <https://theremin.music.uiowa.edu/MIS.html> · FreePats: <https://freepats.zenvoid.org/>
- Creative Commons Sampling Plus 1.0: <https://creativecommons.org/licenses/sampling+/1.0/>
- Репо проекта: `docker/vision/scripts/resource_pack/manifest.yaml`, `fetch_sample_pack.py`, `scripts/music/build_sample_pack_catalog.py`, `docs/music/ATTRIBUTION.md`, `src/rob_box_music/rob_box_music/knowledge.py` (`INSTRUMENTS`, `SYNTH_PALETTE`, `SAMPLE_CATALOG`, `PACK_BREAK_BEATS`), `render/renardo.py`, `docker/vision/voice_assistant/custom_synthdefs/`
