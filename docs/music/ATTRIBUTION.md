# Атрибуция: партитуры как материал аранжировщика (ADR-0154 §3.7)

Что здесь записано и чего в репозитории **нет**. Этот файл — единственное место атрибуции данных, из которых
строится материал аранжировщика (`ScoreMaterial`, `score_index`). Проверяется вручную, не гардом CI.

## Что в git, что нет

| Лежит в git | Не лежит в git |
|---|---|
| схема JSON материала (`rob_box_music/material.py`), импортёр (`scripts/music/score_import.py`), тесты на синтетических партитурах, написанных нами | партитуры (MusicXML/MXL), `PDMX.csv`, метаданные PDMX |
| выученные статистические таблицы (ступени, переходы; ADR-0154 §3.4, PR-3/PR-6) | JSON-библиотека материалов (`<каталог>/pdmx_<id>.json`) и `score_index` в `voice_memory.db` |
| числа исследования (`docs/music/research_score_material.md`) | чужие аранжировки («Интерстеллар» из `~/Downloads`) |

Решение товарища Шифу (06.10.2026, ADR-0154 §8 В1): партитуры и JSON-библиотека в репозиторий не кладутся.

## Источники

### PDMX

- **Что это:** PDMX — датасет MusicXML-партитур с MuseScore, метаданные в `PDMX.csv`, партитуры в `mxl.tar.gz`.
- **Ссылка:** Zenodo, DOI [10.5281/zenodo.15571083](https://doi.org/10.5281/zenodo.15571083).
- **Цитирование:** Long, Novack, Berg-Kirkpatrick, McAuley. *PDMX: A Large-Scale Public Domain MusicXML Dataset for
  Symbolic Music Processing.* ICASSP 2025.
- **Лицензия набора:** CC-BY-4.0 — так записано в ADR-0154 §3.7 «по брифу»; **не сверено** с карточкой Zenodo при
  написании этого файла. Перед выкладкой чего-либо, построенного на PDMX, сверить с карточкой.
- **Что берётся:** только `subset:no_license_conflict` (`license_conflict=False`); импортёр отклоняет строку с
  `license_conflict=True` до разбора (код `license_conflict` в отчёте), файл вне `PDMX.csv` — `not_in_csv`.
  В `score_index` попадает только материал с пригодной лицензией (`material.license_usable`: не пусто, не `unknown`, не
  `…conflict`).
- **Значения `license`:** как в CSV — `publicdomain`, `cc-zero`. Это **метка загрузившего**, а не юридическое
  заключение: в `no_license_conflict` лежат и современные произведения (замер `pdmx_coverage_2026-10-06.md` §4a). Поэтому
  использование материала выбирается кодом, а не обещается лицензией; живые правообладатели — `knowledge.LICENSE_STOP_LIST`.
- **Авторство аранжировки:** автор партитуры на MuseScore в `PDMX.csv` не указан (колонка `publisher` пуста).
  В `ScoreMaterial` записаны `composer` (из `composer_name`, иначе `artist_name`), `source` (`PDMX <id>`) и
  `material_id` (`pdmx:<id>` — по нему партитура находится в самом PDMX). Этого достаточно для указания источника,
  но не для указания автора аранжировки — открытый вопрос ADR-0154 В1.

### Выборка исследования (71 партитура)

- 69 файлов из `musetrainer/library` (клон 06.10.2026): произведения общественного достояния, аранжировки — участников
  проекта; 2 файла «Интерстеллар» (музыка Х. Циммера, 2014) — **не** общественное достояние, только локально, не
  распространяются, в индекс не попадают (`--license` для них не задаётся).
- В репозитории от выборки — только числа исследования.

### Инструменты

- **music21** (BSD-3-Clause) — разбор MusicXML; зависимость **только** скрипта `score_import.py` (офлайн, хост/katana).
  Не входит в образы робота и в `requirements` рантайма: `pip install music21` на машине, где запускается импортёр.

## Как указывается источник на роботе

Каждый материал несёт `material_id` (`pdmx:<id>` | `local:<sha8>`), `composer`, `source`, `license`; их пишет импортёр,
индекс хранит `license`, `rating`, `n_ratings`, `keysig` и `file`. Запись `material_id` в лог трека (ADR-0154 M6) появится вместе с потребителем материала (PR-3+); в PR-2 её нет.


# Атрибуция: сэмпл-паки Ресурсного пака (решение товарища Шифу 07.10.2026)

Сами аудиофайлы в репозиторий **не кладутся**: их ставит на хост Ресурсный пак (ADR-0125/0126; записи манифеста
`sonicpi-samples`, `muldjord-kit`, фетчер `docker/vision/scripts/resource_pack/fetch_sample_pack.py`, эталоны sha256 —
`sonicpi_samples.lock.json`, `muldjord_kit.lock.json`). В git — только код, lock-файлы и каталоги
`rob_box_music/data/sample_sonicpi.json`, `sample_muldjord.json`. В генератор паки пока не подключены (ADR-0153 S3/S5).

## DrumGizmo MuldjordKit

- **Автор:** Lars Muldjord (Tama Superstar, запись для проекта DrumGizmo).
- **Источник:** <https://drumgizmo.org/wiki/doku.php?id=kits:muldjordkit>; официальный архив
  <https://drumgizmo.org/kits/MuldjordKit/MuldjordKit3.zip> (версия 3.0, md5 по странице кита
  `8a66a3e90bbf15687b2d34fd355024f2`). Архив больше 1.1 ГБ (скачивание прервано на 1.15 ГБ), поэтому хук берёт отдельные flac из конверсии
  <https://github.com/sfzinstruments/DrumGizmo.MuldjordKit> на пиннутом коммите (лицензия репозитория `cc-by-4.0`),
  см. `_comment` в `muldjord_kit.lock.json`.
- **Лицензия:** Creative Commons Attribution 4.0 International (CC BY 4.0),
  <https://creativecommons.org/licenses/by/4.0/>. **Требуется указание авторства.**
- **Что берётся:** 32 flac (16 инструментов × 2 удара), один ближний микрофон на инструмент, ~7 МБ.
- **Как указывать:** с января 2021 автор просит при использовании кита в композиции писать в выходных данных альбома
  «Drum samples provided by DrumGizmo.org» (на странице кита; требование сформулировано для композиций). Если
  генератор начнёт отдавать сыгранное на этом ките наружу (запись, публикация), строка авторства обязательна. Пока
  кит не подключён к генератору, это условие не наступило.

## Sonic Pi samples

- **Источник:** <https://github.com/sonic-pi-net/sonic-pi/tree/dev/etc/samples> (пиннутый коммит — в lock-файле).
- **Лицензия:** CC0 1.0 Universal, <http://creativecommons.org/publicdomain/zero/1.0/>. `LICENSE.md` репозитория,
  раздел Samples: каждый сэмпл CC0; `arovane_*` пожертвованы Uwe Zahn (Arovane), `tbd_*` — The Black Dog, остальные
  с freesound.org (ссылки на оригиналы — `etc/samples/README.md`, он ставится на хост рядом с файлами).
- Указание авторства не требуется; запись здесь «для порядка».
