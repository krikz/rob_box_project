# Сырой вывод к `docs/research/2026-11-09-robot-event-model.md` (#4512)

Скрипты — разовые, для этого замера; рантайм робота они не трогают и не импортируют (кроме `replay.py`, который
читает правила `rob_box_dialog` из `src/`). Тексты реплик в репо не кладём: `turns.jsonl` и выгрузка Loki остаются
на машине, которая считала. Здесь только счётчики.

Вход:

- архив марафонов `D:/PROJECTS/rob_box_e2e_archive/` (выгружен `scripts/e2e/archive_e2e_artifacts.py`, #4512, шаг 0);
- Loki на katana `http://10.1.1.249:3100`, только чтение.

| Шаг | Команда | Вывод |
|---|---|---|
| инвентарь архива | `python -X utf8 -I inventory.py <архив>` | `out_inventory.txt` |
| ожидания шагов | `python -X utf8 -I expect_stats.py <архив>` | `out_expect.txt` |
| ходы из логов | `python -X utf8 -I turns.py <архив> turns.jsonl` | `out_turns.txt` |
| база по трём задачам | `python -X utf8 -I analyze.py turns.jsonl` | `out_analyze.txt` |
| правила develop на тех же текстах | `python -X utf8 replay.py turns.jsonl` | `out_replay.txt` |
| точность среза | `python -X utf8 -I slice_use.py turns.jsonl` | `out_slice_use.txt` |
| Loki: счётчики за 7 дней | `python -X utf8 -I loki_counts.py voice-assistant "" "turn_id=" …` | `out_loki_counts.txt` |
| Loki: ходы по суткам | `python -X utf8 -I loki_daily.py voice-assistant "[turn] turn_id="` | `out_loki_daily.txt` |
| Loki: выгрузка строк | `python -X utf8 -I loki_pull.py loki_live.tsv "<regex>"` (два прохода, см. док §2) | — |
| живые ходы из Loki | `python -X utf8 live.py <каталог выгрузки>` | `out_live.txt` |
| прогоны и сроки артефактов | `gh run list --workflow 324736062 …`, `gh api …/runs/42142121516/artifacts` | `out_gh_runs.txt` |
| детекторы сбоя хода (шаг 3) на архиве | `python -X utf8 detectors.py <архив>` | `out_detectors.txt` |
| сетка порогов «робот глух» | `python -X utf8 detectors.py <архив> --grid` | `out_detectors_grid.txt` |
| детекторы на живых строках Loki 06–09.11 | `python -X utf8 detectors.py <архив> --live <выгрузка>` (выгрузка — `loki_pull.py` с началом окна 06.11 01:19 UTC, фильтр в PR) | `out_detectors_live.txt` |
| Sound Pi | `ssh -o BatchMode=yes ros2@10.1.1.22 'docker inspect voice-assistant …; ls /tmp/e2e_dialog'` | `out_sound_pi.txt` |

Сопоставление шага и хода в `turns.py` — по тексту (difflib ≥ 0.6) внутри одного прогона. Логи соседних актов
перекрываются, поэтому ход считается один раз по `turn_id`.

Замечание к Loki: фильтр `|=`/`|~` со знаком-эмодзи в начале (`📊 [turn]`, `🎤 STT:`) возвращает 0 строк, поэтому
фильтры без эмодзи (`[turn] turn_id=`, `dialog_node\]: .{1,4} STT: `). Запрос за 7 дней одним окном Loki отбивает
429 «too many outstanding requests», поэтому `loki_pull.py` идёт окнами по 6 ч.
