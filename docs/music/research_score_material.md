# Исследование: что в реальных партитурах умеет аранжировка и чего не умеет наш v2 (ADR-0154)

Дата: 2026-10-06. Автор: Claude Code (шисюн). Код сверен на `origin/develop` @ `0fa50bc8c`.

## 1. Выборка и способ

- **71 партитура**: 69 MXL из `musetrainer/library` (public domain: Бах, Бетховен, Шопен, Дебюсси, Сати, Джоплин, «Bella Ciao», «Canon in D», «Carol of the Bells»…; клон репо 06.10, `git clone --depth 1`) + 2 файла «Интерстеллар» из `~/Downloads` (орган 3 партии 3/4, 115 тактов; фортепианная сюита 372 такта). Партитуры в git **не кладутся** (чужие аранжировки); здесь — только числа.
- Инструменты: Python 3.11, music21 10.5.0 (разбор, `analyze('key')`), наш пакет `rob_box_music` из worktree (`PYTHONPATH=src/rob_box_music`): `tonality.detect_key/key_fit`, `arrange.hook.from_rtttl`, `arrange.harmony.fit_progression` (стиль `club`).
- Скрипты: `scripts/music/research/score_material_probe.py` (всё по партитуре + сводка), `scripts/music/research/score_markov_harmony.py` (опыт «выученная таблица переходов против шаблонов»). Запуск: `python scripts/music/research/score_material_probe.py <каталог> <файлы…> --out stats.json`, затем `python scripts/music/research/score_markov_harmony.py stats.json`.
- Что считается (эвристики на сетке 16-х, не музыковедческая разметка):
  - **мелодия** — skyline: верхняя нота каждого онсета по всем партиям; доля онсетов, где она из партии 0;
  - **гармония** — на каждые полтакта лучший из 24 мажорных/минорных трезвучий по длительности нот (+50 % вес приме, +15 % инерция к предыдущему — как `core/harmonize._best_chord`), ступень в тональности music21; соседние одинаковые окна слиты;
  - **бас** — низшая звучащая нота, начинающаяся на онсете; отношение к аккорду полутакта;
  - **фактура** — такт без нот skyline: ≤ 2 онсетов и ≥ 2 голосов → `sustained`; ≥ 2.5 нот на онсет → `block`; ≥ 4 онсетов и < 1.5 нот на онсет → `broken`; иначе `mixed`;
  - **структура** — отпечаток такта мелодии (шаг, длительность, MIDI); повторы; отношение соседних 4-тактовых фраз: `repeat` (те же ноты), `sequence` (тот же ритм, все ноты сдвинуты на один интервал), `rhythm` (тот же ритм, другие высоты), `new`;
  - **наш аранжировщик** — первые 8 тактов skyline → RTTTL (сетка 16-х) → `hook.from_rtttl(…, bpm=130, root, mode)`; `fit_progression` на каждом 8-тактовом окне против ступени оригинала по 2-тактовым слотам (аккорд слота — самый долгий).

## 2. Сводка (raw: `summary` из `stats.json`, 70 разобранных из 71)

```
scores 71, parsed 70 (отказ парсера: Mozart_-_Piano_Sonata_No._16_-_Allegro.mxl —
  MusicXMLImportException: In part (Piano), measure (18): incorrect accidental 9.0 for pitch F3)
время разбора: всего 88.6 с, медиана 0.9 с/партитуру, максимум 5.5 с (сюита, 372 такта)
размеры (встречаются в партитуре): 4/4 ×29, 3/4 ×24, 2/4 ×17, 3/8 ×6, 6/4 ×5, 2/2 ×3, 5/4 ×3, 12/8 ×2, 9/8 ×2, 6/8 ×1
  по партитурам: только 4/4 — 19, только 3/4 — 17, только 2/4 — 10, прочие одиночные — 11, смешанные — 13
лад (music21): мажор 32, минор 38
key_agree_ours_vs_music21: 59/70
  расхождения: орган Интерстеллар (C major ↔ E minor), Arabesque (A major ↔ C# minor), Bella_Ciao (F# minor ↔ E major),
  Ballade g-moll (G minor ↔ C minor), Nocturne op.9/1 (C# major ↔ A# minor), Flight of the Bumblebee (D minor ↔ A major),
  Fur Elise beginner (A minor ↔ E harmonicMinor), G_Minor_Bach ×2 (C minor ↔ G minor), La Campanella (G# minor ↔ D# minor),
  Schubert Serenade (D major ↔ D harmonicMinor)
melody_key_fit (skyline в тональности music21): медиана 0.901; ≥ 0.6 — 70/70
skyline_top_part_share: медиана 0.758
melody_notes_per_bar: медиана 7.43
chord_changes_per_bar: медиана 1.15; bars_with_two_chords: медиана 0.36; two_bar_single_chord_share: медиана 0.10
degrees_total (слитые окна, имя = ступень от тоники, «m» — минорное трезвучие):
  V 1146, I 915, Im 891, IVm 423, IV 409, bIII 364, IIm 297, bVI 254, bVII 244, VIm 216, Vm 212, IIIm 184
bigrams_total: V>Im 324, V>I 312, Im>V 247, I>V 205, I>IV 175, Im>IVm 174, IV>I 158, bVII>bIII 103, IVm>Im 95,
  IIm>I 92, bIII>Im 70, IVm>bVII 70, I>IVm 67, IVm>V 64, Im>bIII 63, IV>V 62, V>VIm 62, II>V 61, bIII>V 57, I>Im 57
bass_rel_total: root 14854 (41.6 %), fifth 7166 (20.1 %), third 6981 (19.5 %), other 6715 (18.8 %)
bass_notes_per_bar: медиана 5.5; bass_on_beat_share: медиана 0.507; bass_approach_total (полутон к первой доле): 646
textures_total: mixed 2704 (45.6 %), sustained 1312 (22.1 %), broken 1179 (19.9 %), block 732 (12.4 %), none 304
unique_bar_share: медиана 0.72
repeat_gaps_total (расстояние до повтора такта): 8 → 205, 4 → 176, 1 → 123, 2 → 115, 16 → 113, 70 → 59, 34 → 41, 39 → 40
phrase_relations_total: new 981 (66.9 %), rhythm 454 (31.0 %), repeat 28 (1.9 %), sequence 2 (0.1 %)
scores_with_text_marks: 51/70 (в основном темп/динамика; названия частей — 6 партитур)
our_hook_ok: 67/70; отказы: 3 × «мотив ломается: N из M нот не помещаются в коридор (58, 84) без переноса октавой»
  (Dance_of_the_sugar_plum_fairy 18/38; moonlight_sonata_3rd ×2 41/118); длина хука: 8 тактов — 54, 4 такта — 13
our_hook_key_fit: медиана 0.978 (минимумы 0.625, 0.781, 0.812)
our_progression (fit_progression, club): windows 755, slots 3020, hit_share 0.217, tonic_baseline_share 0.346,
  pool_oracle_share 0.337
```

## 3. Опыт: выученная таблица переходов (Витерби) против шаблонов (`score_markov_harmony.py`, raw)

```
партитур 70, слотов с диатоническим аккордом оригинала 2831
tonic                   1045  0.369   («везде тоника»)
viterbi_w1.0             899  0.318   (w — вес покрытия мелодии против log P перехода; leave-one-out по партитурам, +1 сглаживание)
viterbi_w0.5             893  0.315
viterbi_w0.0             881  0.311   (чистая марковская цепь, мелодия не смотрится)
viterbi_w2.0             877  0.310
viterbi_w4.0             826  0.292
template                 656  0.232   (harmony.fit_progression, 8 шаблонов club)
melody_only_argmax       601  0.212   (ступень с наибольшим покрытием нот слота)

major: переходов 2357; топ: 4->0 285 (0.12), 0->4 216 (0.09), 0->3 210 (0.09), 3->0 205 (0.09), 1->4 106 (0.04),
  4->5 83 (0.04), 0->1 81, 0->2 80, 1->0 76, 3->4 72, 2->3 71, 5->2 70
minor: переходов 3271; топ: 4->0 456 (0.14), 0->4 323 (0.10), 0->3 271 (0.08), 3->0 152 (0.05), 4->1 118 (0.04),
  1->4 117 (0.04), 6->2 114, 3->6 111, 3->4 108, 1->0 103, 2->4 91, 0->2 80
```

Честный вывод: на 2-тактовых слотах (наш `CHORD_BARS = 2`, `arrange/compose.py:65`) **ни один алгоритм подбора без материала не обгоняет «везде тонику»** (37 %), потому что аккорд слота в классике чаще всего тоника, а настоящие смены идут внутри такта (медиана 1.15 смен на такт, 2-тактовых окон с одним аккордом — 10 %). Выученная таблица лучше шаблонов на 9 п.п.; настоящий выигрыш даёт материал: авторские аккорды и авторский гармонический ритм. Все значения `w` показаны — подбор `w` шёл на той же выборке. Опыт на слотах в 1 такт/полтакта не делал.

### 3.1 Перемер M3 кодом PR-3 (`score_harmony_m3.py`, 06.10, raw)

Код аранжировщика (`harmony.from_material`, `harmony.viterbi`, `compose` с `TrackPlan.material`) на той же
выборке. Материал строится из пробы: аккорды полутактов (слитые), ступень по приме в ладу music21; пьеса режется на
куски ровных тактов одной длины ≤ 4/4 (≥ 8 тактов, ADR-0154 §3.1). Оговорка к §3: эмиссия `score_markov_harmony.py`
считает трезвучия от C, а не от тоники (вклад мелодии занижен для не-C); `harmony.viterbi` берёт тонику.

```
файлов 71, партитур с материалом 63 (кусков 93), пропущено {'нет ровного куска ≥ 8 тактов ≤ 4/4': 7, 'MusicXMLImportException': 1}; слот = 2 такт(а)
слотов с диатоническим аккордом оригинала 2359 (всех слотов 2508)
  material                2143  0.908   (from_material: M3 ≥ 0.80)
  material_raw_adapted    2285  0.969   (аккорд слота + адаптация недиатоники, без правил «один аккорд → Витерби» и каденции)
  viterbi_loo              946  0.401   (без материала, таблица leave-one-out)
  template                 585  0.248
  tonic                    864  0.366
compose: треков 63, хук и гармония из материала 28, слотов дропа 110, совпало 97 (0.882)
  compose_refused: мотив ломается: 6
  compose_refused: пэд/лад гармонии: 15      (все — «прогрессия … не помещается в регистр (50, 59|60)»)
  compose_refused: размер 2/4 не переводится в 4/4 клуба (A: 11
  compose_refused: размер 3/8 не переводится в 4/4 клуба (A: 3

слот = 1 такт:  material 4595/4698 0.978, viterbi_loo 1868 0.398, tonic 1664 0.354
опыт CADENCE_MIN_P = 0 (без каденции): material 2147 0.910 — каденция стоит 0.2 п.п., правило «фраза на одном
аккорде → Витерби» (ADR-0154 §3.3) — ≈ 5.9 п.п.
```

Вывод: с материалом ступени слотов совпадают с оригиналом в 91 % (M3 ≥ 80 % выполнен), без материала Витерби по
выученной таблице (40 %) лучше шаблонов (25 %) и «везде тоники» (37 %) на этих окнах. Сквозь `compose` материал
доходит до трека у 28 из 63 пьес: отказы — хук (размер 2/4 и 3/8 не переводится, §3.6; мотив ломается) и регистр
пэда под низким хуком: при низе лида 62 пэд зажат в (50, 59), где нет ноты C, — трезвучия с C не ставятся ни в одном
обращении (комментарий `compose.hook_register` «+9 — любое обращение» неверен; RTTTL-путь тоже упирается, но молча
берёт следующую мелодию).

## 4. Что из этого следует для v2 (кратко; решения — в ADR-0154)

1. Skyline как мелодия работает (0.76 из верхней партии; хук принят у 67/70, `key_fit` 0.98) — путь «партитура → ноты → хук» уже есть на develop, нужен общий `hook.from_notes`.
2. Тональность материала брать из партитуры (знаки + music21 по всем голосам): `detect_key` по 8 тактам мелодии расходится в 11/70.
3. Гармония и гармонический ритм — из материала; без материала — таблица переходов (данные `knowledge`), но её выигрыш на 2-тактовых слотах скромный.
4. Бас: прима 42 %, квинта 20 %, терция 20 %, чужая 19 %; половина нот на долях; подходов полутоном ≈ 9 на партитуру — политика тонов как данные; наш «тоника ×3 + квинта» не знает терции и подхода.
5. Фактуры: 46 % смешанных, 22 % выдержанных, 20 % арпеджио, 12 % блоков — у нас нет арпеджио.
6. Повторы через 8 и 4 такта доминируют; самая повторяемая фраза — кандидат в хук; 72 % тактов уникальны (вариация, не копия).
7. Развитие соседних фраз: «тот же ритм, другие высоты» 31 %, буквальный повтор 2 %, секвенция 0.1 % — у нас `DEVELOPMENT` знает укорочение/замедление/терции, но не «тот же ритм, другой контур».
8. Не-4/4 — ≈ 70 % партитур выборки (3/4, 2/4, 6/8, 3/8…): перевод размера — не частный случай.

## 5. По партитурам (raw)

| Файл | Тактов | Размеры | Тональность (music21) | Наш `detect_key` | Хук 8 т. | `key_fit` хука | Смен акк./такт | 2-т. окна с 1 акк. | Шаблон: совп./слотов | Бас прима % | Уникальных тактов |
|---|---|---|---|---|---|---|---|---|---|---|---|
| interstellar-hans-zimmer-organ-variation.musicxm | 115 | 3/4 | C major | E minor ✗ | ок 8 т. | 1.0 | 0.3 | 0.86 | 10/56 | 85 | 0.07 |
| interstellar-suite-hans-zimmer.musicxml.xml | 372 | 3/4 4/4 | A minor | A minor ✓ | ок 8 т. | 1.0 | 0.77 | 0.44 | 29/184 | 48 | 0.444 |
| 12_Variations_of_Twinkle_Twinkle_Little_Star.mxl | 325 | 2/4 3/4 | C major | C major ✓ | ок 4 т. | 1.0 | 1.25 | 0.09 | 51/160 | 45 | 0.655 |
| Arabesque_L._66_No._1_in_E_Major.mxl | 107 | 2/4 4/4 | A major | C# minor ✗ | ок 8 т. | 1.0 | 1.17 | 0.13 | 13/52 | 33 | 0.72 |
| Ave_Maria_D839_-_Schubert_-_Solo_Piano_Arrg..mxl | 17 | 4/4 | B- major | A# major ✓ | ок 8 т. | 0.927 | 1.0 | 0.25 | 2/8 | 36 | 0.882 |
| Bach_Minuet_in_G_Major_BWV_Anh._114.mxl | 32 | 3/4 | G major | G major ✓ | ок 8 т. | 1.0 | 1.16 | 0.06 | 7/16 | 54 | 0.844 |
| Bach_Toccata_and_Fugue_in_D_Minor_Piano_solo.mxl | 143 | 4/4 | D minor | D minor ✓ | ок 8 т. | 0.978 | 1.1 | 0.17 | 10/68 | 31 | 0.972 |
| Beethoven_Symphony_No._5_1st_movement_Piano_solo | 504 | 2/4 | C minor | C minor ✓ | ок 4 т. | 1.0 | 1.0 | 0.26 | 48/252 | 61 | 0.554 |
| Bella_Ciao.mxl | 38 | 4/4 | F# minor | E major ✗ | ок 8 т. | 0.984 | 0.84 | 0.26 | 2/16 | 76 | 0.737 |
| Bella_Ciao_-_La_Casa_de_Papel.mxl | 74 | 2/4 4/4 | A minor | A harmonicMinor ✓ | ок 8 т. | 1.0 | 0.85 | 0.27 | 9/36 | 31 | 0.446 |
| Canon_in_D.mxl | 102 | 4/4 | D major | D major ✓ | ок 8 т. | 1.0 | 1.88 | 0.0 | 18/48 | 48 | 0.745 |
| Canon_in_D_3.mxl | 53 | 4/4 | D major | D major ✓ | ок 8 т. | 1.0 | 1.92 | 0.0 | 9/24 | 47 | 0.906 |
| Canon_in_D_easy.mxl | 49 | 4/4 | D major | D major ✓ | ок 8 т. | 1.0 | 1.96 | 0.0 | 10/24 | 71 | 0.918 |
| Carol_of_the_Bells.mxl | 65 | 3/4 | D minor | D harmonicMinor ✓ | ок 8 т. | 1.0 | 0.77 | 0.34 | 6/32 | 39 | 0.462 |
| Carol_of_the_Bells_easy_piano.mxl | 40 | 3/4 | A minor | A harmonicMinor ✓ | ок 8 т. | 1.0 | 1.18 | 0.1 | 4/20 | 65 | 0.25 |
| Chopin_-_Ballade_no._1_in_G_minor_Op._23.mxl | 262 | 2/2 4/4 6/4 | G minor | C minor ✗ | ок 8 т. | 0.917 | 1.16 | 0.12 | 24/128 | 37 | 0.847 |
| Chopin_-_Nocturne_Op._9_No._1.mxl | 86 | 6/4 | C# major | A# minor ✗ | ок 8 т. | 0.905 | 1.1 | 0.19 | 7/40 | 33 | 0.663 |
| Chopin_-_Nocturne_Op_9_No_2_E_Flat_Major.mxl | 38 | 12/8 2/4 6/4 | E- major | D# major ✓ | ок 8 т. | 0.898 | 1.5 | 0.11 | 2/16 | 35 | 0.763 |
| Chopin_-_Spring_Waltz.mxl | 84 | 3/4 3/8 4/4 5/4 | G minor | G minor ✓ | ок 8 т. | 0.984 | 1.25 | 0.05 | 8/40 | 35 | 0.607 |
| Clair_de_lune_-_Claude_Debussy.mxl | 72 | 9/8 | C# major | C# major ✓ | ок 8 т. | 0.953 | 1.0 | 0.25 | 7/36 | 36 | 0.903 |
| Clair_de_Lune__Debussy.mxl | 72 | 9/8 | C# major | C# major ✓ | ок 8 т. | 0.953 | 0.92 | 0.28 | 7/36 | 40 | 0.903 |
| Dance_of_the_sugar_plum_fairy.mxl | 53 | 2/4 | E minor | E harmonicMinor ✓ | отказ | — | 1.47 | 0.0 | 3/24 | 31 | 0.83 |
| DANSE_VILLAGEOISE_Beethoven.mxl | 61 | 3/4 | D major | D major ✓ | ок 8 т. | 1.0 | 0.54 | 0.6 | 6/28 | 55 | 0.393 |
| Erik_Satie_-_Gymnopedie_No.1.mxl | 47 | 3/4 | D major | D major ✓ | ок 8 т. | 1.0 | 0.98 | 0.13 | 3/20 | 81 | 0.617 |
| Flight_of_the_Bumblebee.mxl | 101 | 2/4 | D minor | A major ✗ | ок 4 т. | 0.625 | 1.5 | 0.06 | 4/48 | 31 | 0.634 |
| Fur_Elise.mxl | 106 | 3/8 | A minor | A minor ✓ | ок 4 т. | 0.915 | 1.42 | 0.04 | 10/52 | 53 | 0.434 |
| Fur_Elise_-_Beethoven_-_for_beginner_piano.mxl | 24 | 3/8 | A minor | E harmonicMinor ✗ | ок 4 т. | 0.83 | 1.42 | 0.0 | 3/12 | 53 | 0.625 |
| Fur_Elise_Easy_Piano.mxl | 22 | 3/4 | A minor | A harmonicMinor ✓ | ок 8 т. | 0.923 | 1.36 | 0.0 | 0/8 | 49 | 0.545 |
| Fur_Elise_fingered.mxl | 106 | 3/8 | A minor | A minor ✓ | ок 4 т. | 0.83 | 1.34 | 0.04 | 12/52 | 53 | 0.434 |
| G_Minor_Bach.mxl | 66 | 4/4 | C minor | G minor ✗ | ок 8 т. | 1.0 | 0.94 | 0.24 | 7/32 | 37 | 0.758 |
| G_Minor_Bach_Original.mxl | 66 | 4/4 | C minor | G minor ✗ | ок 8 т. | 1.0 | 0.95 | 0.24 | 7/32 | 36 | 0.758 |
| Gnossienne_No._1.mxl | 11 |  | F minor | F minor ✓ | ок 8 т. | 0.857 | 0.27 | 0.8 | 1/4 | 79 | 1.0 |
| Greensleeves_for_Piano_easy_and_beautiful.mxl | 33 | 3/4 | A minor | A harmonicMinor ✓ | ок 8 т. | 0.958 | 1.58 | 0.0 | 2/16 | 39 | 0.848 |
| Gymnopdie_No._1__Satie.mxl | 78 | 3/4 | D major | D major ✓ | ок 8 т. | 1.0 | 0.97 | 0.1 | 5/36 | 85 | 0.372 |
| Happy_Birthday_To_You_C_Major.mxl | 8 | 3/4 | C major | C major ✓ | ок 8 т. | 1.0 | 0.5 | 0.0 | 2/4 | 45 | 0.75 |
| Happy_Birthday_To_You_Piano.mxl | 20 | 3/4 | C major | C major ✓ | ок 8 т. | 1.0 | 0.6 | 0.5 | 6/8 | 57 | 0.7 |
| Hungarian_Dance_No_5_in_G_Minor.mxl | 102 | 2/4 | G minor | G minor ✓ | ок 4 т. | 0.984 | 0.83 | 0.29 | 18/48 | 43 | 0.588 |
| Hungarian_Sonata.mxl | 49 | 2/4 4/4 | C minor | C minor ✓ | ок 8 т. | 1.0 | 1.29 | 0.04 | 4/24 | 31 | 0.939 |
| J._S._Bach_-_Air_on_the_G_String_Piano_arrangeme | 37 | 4/4 | G major | G major ✓ | ок 8 т. | 0.922 | 1.43 | 0.06 | 3/16 | 43 | 1.0 |
| La_Campanella_-_Grandes_Etudes_de_Paganini_No._3 | 150 | 6/8 | G# minor | D# minor ✗ | ок 8 т. | 0.964 | 1.15 | 0.16 | 16/72 | 40 | 0.859 |
| Lacrimosa_-_Requiem.mxl | 32 | 12/8 | D minor | D minor ✓ | ок 8 т. | 0.953 | 1.44 | 0.12 | 5/16 | 34 | 0.9 |
| Liebestraum_No._3_in_A_Major.mxl | 88 | 6/4 | A- major | G# major ✓ | ок 8 т. | 0.812 | 1.18 | 0.09 | 3/44 | 30 | 0.943 |
| Maple_Leaf_Rag_Scott_Joplin.mxl | 85 | 2/4 | A- major | G# major ✓ | ок 4 т. | 0.935 | 0.93 | 0.26 | 14/40 | 42 | 0.694 |
| Mariage_dAmour.mxl | 83 | 3/4 3/8 4/4 5/4 | G minor | G minor ✓ | ок 8 т. | 0.984 | 1.18 | 0.02 | 8/40 | 35 | 0.663 |
| Minuet_in_G_Major_Bach.mxl | 32 | 3/4 | G major | G major ✓ | ок 8 т. | 1.0 | 1.16 | 0.06 | 7/16 | 56 | 0.844 |
| moonlight_sonata_3rd_movement.mxl | 201 | 4/4 | C# minor | C# minor ✓ | отказ | — | 1.01 | 0.23 | 15/100 | 40 | 0.841 |
| Mozart_-_Piano_Sonata_No._16_-_Allegro.mxl | — | — | ошибка парсера: `MusicXMLImportException: In part (Piano), measure (18): inco` | | | | | | | | |
| Nocturne_in_C_sharp_Minor.mxl | 65 | 2/4 3/4 4/4 | C# minor | C# harmonicMinor ✓ | ок 8 т. | 0.904 | 1.23 | 0.12 | 4/32 | 42 | 0.862 |
| Nocturne_in_E-flat_Major_Op._9_No._2_Easy.mxl | 65 | 3/4 | E- major | D# major ✓ | ок 8 т. | 0.875 | 1.31 | 0.0 | 3/32 | 40 | 0.662 |
| Nocturne_No._20_in_C_Minor.mxl | 65 | 2/4 3/4 4/4 | C# minor | C# harmonicMinor ✓ | ок 8 т. | 0.896 | 1.26 | 0.09 | 3/32 | 41 | 0.846 |
| Ode_to_Joy_Easy_variation.mxl | 17 | 4/4 | G major | G major ✓ | ок 8 т. | 1.0 | 1.24 | 0.0 | 2/8 | 58 | 0.588 |
| Passacaglia.mxl | 74 | 4/4 | A minor | A minor ✓ | ок 8 т. | 1.0 | 0.85 | 0.03 | 19/36 | 38 | 0.608 |
| Passacaglia2.mxl | 74 | 4/4 | A minor | A minor ✓ | ок 8 т. | 1.0 | 0.85 | 0.03 | 19/36 | 38 | 0.608 |
| Piano_Sonata_No._11_K._331_3rd_Movement_Rondo_al | 137 | 2/4 | A major | A major ✓ | ок 4 т. | 0.906 | 1.02 | 0.18 | 10/68 | 65 | 0.504 |
| Prelude_I_in_C_major_BWV_846_-_Well_Tempered_Cla | 34 | 4/4 | C major | C major ✓ | ок 8 т. | 0.969 | 0.85 | 0.18 | 3/16 | 63 | 0.912 |
| Prelude_No._2_BWV_847_in_C_Minor.mxl | 38 | 4/4 | C minor | C harmonicMinor ✓ | ок 8 т. | 0.781 | 0.53 | 0.42 | 5/16 | 17 | 0.947 |
| Prlude_No._4_in_E_Minor_Op._28_-_Frdric_Chopin.m | 26 | 2/2 | E minor | E minor ✓ | ок 8 т. | 0.875 | 1.27 | 0.08 | 5/12 | 41 | 0.846 |
| Prlude_Opus_28_No._4_in_E_Minor__Chopin.mxl | 26 | 2/2 | E minor | E minor ✓ | ок 8 т. | 0.875 | 1.27 | 0.08 | 5/12 | 41 | 0.846 |
| Schubert_Serenade_-_Standchen_-_By_Lizst.mxl | 115 | 3/4 | D major | D harmonicMinor ✗ | ок 8 т. | 1.0 | 0.97 | 0.09 | 8/56 | 45 | 0.8 |
| Sonata_No._16_1st_Movement_K._545.mxl | 73 | 4/4 | C major | C major ✓ | ок 8 т. | 1.0 | 1.34 | 0.08 | 6/36 | 27 | 0.904 |
| Sonate_No._14_Moonlight_1st_Movement.mxl | 69 | 4/4 | C# minor | C# minor ✓ | ок 8 т. | 0.958 | 1.04 | 0.12 | 5/32 | 66 | 0.884 |
| Sonate_No._14_Moonlight_3rd_Movement.mxl | 201 | 4/4 | C# minor | C# minor ✓ | отказ | — | 1.01 | 0.24 | 17/100 | 40 | 0.836 |
| Sonate_No._8_Pathetique_2nd_Movement.mxl | 73 | 2/4 | A- major | G# major ✓ | ок 4 т. | 0.968 | 1.33 | 0.03 | 7/36 | 44 | 0.685 |
| Spring_Waltz_Mariage_dAmour_-_Chopin.mxl | 84 | 3/4 3/8 4/4 5/4 6/4 | G minor | G minor ✓ | ок 8 т. | 0.984 | 1.18 | 0.05 | 4/40 | 35 | 0.655 |
| Swan_Lake.mxl | 32 | 4/4 | B minor | B minor ✓ | ок 8 т. | 1.0 | 0.91 | 0.31 | 4/16 | 36 | 0.481 |
| The_Entertainer_-_Scott_Joplin.mxl | 92 | 2/4 | C major | C major ✓ | ок 4 т. | 0.935 | 1.18 | 0.02 | 12/44 | 39 | 0.598 |
| The_Entertainer_-_Scott_Joplin_-_1902.mxl | 92 | 2/4 | C major | C major ✓ | ок 4 т. | 0.935 | 1.11 | 0.09 | 8/44 | 35 | 0.598 |
| WA_Mozart_Marche_Turque_Turkish_March_fingered.m | 137 | 2/4 | A major | A major ✓ | ок 4 т. | 0.906 | 1.02 | 0.18 | 10/68 | 65 | 0.496 |
| Waltz_in_A_MinorChopin.mxl | 57 | 3/4 | A minor | A harmonicMinor ✓ | ок 8 т. | 0.979 | 0.98 | 0.07 | 9/28 | 64 | 0.579 |
| Waltz_of_the_Flowers.mxl | 80 | 3/4 | D major | D major ✓ | ок 8 т. | 0.958 | 0.93 | 0.25 | 6/40 | 59 | 0.575 |
| Waltz_Opus_64_No._2_in_C_Minor.mxl | 194 | 3/4 | C# minor | C# harmonicMinor ✓ | ок 8 т. | 0.957 | 1.2 | 0.03 | 35/96 | 39 | 0.351 |

## 6. Что не проверено

- PDMX не открывал (ни CSV, ни архив); всё о нём — бриф и README `pnlong/PDMX`.
- Ни один трек с гармонией из материала не рендерился и не слушался.
- Пороги фактур/фраз — мои; голосоведение по голосам не мерил; Витерби на слотах в 1 такт не мерил.
- MusPy установлен (0.5.0), но в опыте не использовался.
