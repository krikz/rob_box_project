#!/usr/bin/env bash
# grep-факты карты PR-15 (docs/dialog/pr15_removal_map.md, приложение Б–Г).
# Запуск из корня чистого чекаута, дерево которого равно проверяемой ссылке:
#   bash docs/dialog/pr15_map_greps.sh [REF] [OLD]      (по умолчанию origin/develop и 8178e2809)
# Команды карточек (#3704/#3705/#3706/#3711) идут как в карточках — `grep -r` по рабочему дереву;
# «было» на OLD считается тем же шаблоном через `git grep -E` по ссылке.
set -u
export MSYS_NO_PATHCONV=1   # Git Bash: иначе аргументы git вида "/voice/..." превращаются в пути Windows
REF=${1:-origin/develop}
OLD=${2:-8178e2809}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
NT='/test/\|/tests/\|/test_[^/]*$'          # вне тестов: путь без каталога test/tests и без test_*.py

echo "REF=$REF $(git rev-parse --short=9 "$REF")  OLD=$OLD $(git rev-parse --short=9 "$OLD")"
if git diff --quiet "$REF" -- src scripts docker architecture tools tests .github; then
  echo "рабочее дерево = $REF по src scripts docker architecture tools tests .github"
else
  echo "ВНИМАНИЕ: рабочее дерево отличается от $REF — числа grep -r не про $REF"
fi

count() { grep -c . || true; }
old_count() {  # $1 = ERE-шаблон, $2.. = пути; печатает «всего / вне тестов» на OLD
  local pat=$1; shift
  local all; all=$(git grep -n -E -e "$pat" "$OLD" -- "$@" | count)
  local nt; nt=$(git grep -n -E -e "$pat" "$OLD" -- "$@" | sed "s|^$OLD:||" | cut -d: -f1 | grep -v -e '/test/' -e '/tests/' -e '/test_[^/]*$' | count)
  echo "  на $OLD: всего $all, вне тестов $nt"
}
per_file() { cut -d: -f1 | sort | uniq -c | sort -rn; }

echo; echo "=== #3704 PR-10b (DSM) ==="
echo '$ ls src/rob_box_voice/rob_box_voice/core/dialogue_state_machine.py   # путь из карточки'
ls src/rob_box_voice/rob_box_voice/core/dialogue_state_machine.py 2>&1
echo '$ ls src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py   # фактический путь'
ls src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py 2>&1
echo "\$ grep -rn '\"/dialogue/control\\|_pause_reason\\|check_silence_timeout' src --include=*.py"
grep -rn '"/dialogue/control\|_pause_reason\|check_silence_timeout' src --include=*.py > $TMP/3704.txt
echo "  всего $(count < $TMP/3704.txt), вне тестов $(grep -v "$NT" $TMP/3704.txt | count)"
grep -v "$NT" $TMP/3704.txt | per_file
old_count '"/dialogue/control|_pause_reason|check_silence_timeout' 'src/*.py'

echo; echo "=== #3705 PR-11 (маркеры памяти) ==="
echo '$ grep -rn "_HISTORY_MARKER_RE\|_SPEAKER_TAG_RE\|_META_PREFIX_RE\|discard_last_reply\|voice_memory.db" src --include=*.py'
grep -rn "_HISTORY_MARKER_RE\|_SPEAKER_TAG_RE\|_META_PREFIX_RE\|discard_last_reply\|voice_memory.db" src --include=*.py > $TMP/3705.txt
echo "  всего $(count < $TMP/3705.txt), вне тестов $(grep -v "$NT" $TMP/3705.txt | count)"
for p in _HISTORY_MARKER_RE _SPEAKER_TAG_RE _META_PREFIX_RE discard_last_reply 'voice_memory.db'; do
  echo "  $p: вне тестов $(grep -v "$NT" $TMP/3705.txt | grep -c -- "$p")"
done
echo "  _HISTORY_MARKER_RE / _META_PREFIX_RE / discard_last_reply вне тестов:"
grep -v "$NT" $TMP/3705.txt | grep -e _HISTORY_MARKER_RE -e _META_PREFIX_RE -e discard_last_reply | cut -d: -f1,2
echo "  voice_memory.db вне тестов по файлам:"; grep -v "$NT" $TMP/3705.txt | grep 'voice_memory.db' | per_file
old_count '_HISTORY_MARKER_RE|_SPEAKER_TAG_RE|_META_PREFIX_RE|discard_last_reply|voice_memory.db' 'src/*.py'

echo; echo "=== #3706 PR-12a (Telegram v1) ==="
echo '$ grep -rn "\[TG:\|tg_chat_id\|parse_tg_prefix" src --include=*.py'
grep -rn "\[TG:\|tg_chat_id\|parse_tg_prefix" src --include=*.py > $TMP/3706.txt
echo "  всего $(count < $TMP/3706.txt), вне тестов $(grep -v "$NT" $TMP/3706.txt | count)"
grep -v "$NT" $TMP/3706.txt | per_file
old_count '\[TG:|tg_chat_id|parse_tg_prefix' 'src/*.py'
echo "\$ grep -rn '\"/voice/dialogue/response\"\\|\"/voice/sound/stop\"' src/rob_box_telegram"
grep -rn '"/voice/dialogue/response"\|"/voice/sound/stop"' src/rob_box_telegram

echo; echo "=== #3711 критерии ==="
echo '$ grep -rn "Bug \[A-F\]" src/rob_box_voice src/rob_box_dialog --include=*.py | wc -l   # как в карточке'
echo "  $(grep -rn "Bug \[A-F\]" src/rob_box_voice src/rob_box_dialog --include=*.py | count)" \
  "  <- в BRE «\\[» — литерал «[»: команда ищет текст «Bug [A-F]» и всегда даёт 0"
echo '$ grep -rnE "Bug [A-F]\b" src/rob_box_voice src/rob_box_dialog --include=*.py   # по смыслу: Bug A … Bug F'
grep -rnE "Bug [A-F]\b" src/rob_box_voice src/rob_box_dialog --include=*.py > $TMP/bug.txt
echo "  всего $(count < $TMP/bug.txt), вне тестов $(grep -v "$NT" $TMP/bug.txt | count)"
grep -v "$NT" $TMP/bug.txt | per_file
old_count 'Bug [A-F]\b' 'src/rob_box_voice/*.py' 'src/rob_box_dialog/*.py'
echo "\$ grep -rc \"#[0-9]\\{4\\}\" src/rob_box_dialog/rob_box_dialog | awk -F: '{s+=\$2} END {print s}'"
grep -rc "#[0-9]\{4\}" src/rob_box_dialog/rob_box_dialog | awk -F: '{s+=$2} END {print "  строк: " s}'
echo "  вхождений (grep -o): $(grep -rho "#[0-9]\{4\}" src/rob_box_dialog/rob_box_dialog | count)"
echo "  git grep -c (строки): $REF $(git grep -c -E '#[0-9]{4}' "$REF" -- src/rob_box_dialog/rob_box_dialog | awk -F: '{s+=$NF} END {print s}')," \
  "$OLD $(git grep -c -E '#[0-9]{4}' "$OLD" -- src/rob_box_dialog/rob_box_dialog | awk -F: '{s+=$NF} END {print s}')"
echo '$ grep -rlE "rob_box_voice\.dialogue_node|from rob_box_voice import dialogue_node|dialogue_node import" src tests --include=*.py   # тесты ноды по строке'
echo "  тестовых файлов: $(grep -rlE "rob_box_voice\.dialogue_node|from rob_box_voice import dialogue_node|dialogue_node import" src tests --include=*.py | grep "$NT" | count)" \
  "(по AST-импорту — 47, приложение А; разница — patch(\"…\")/sys.modules)"
echo '$ grep -c "RULE #" src/rob_box_voice/prompts/*.txt'
grep -c "RULE #" src/rob_box_voice/prompts/*.txt | grep -v ':0$'
echo "\$ grep -c 're.compile' speak_helpers.py dialogue_guards.py"
grep -c "re.compile" src/rob_box_voice/rob_box_voice/core/speak_helpers.py src/rob_box_voice/rob_box_voice/core/dialogue_guards.py

echo; echo "=== String-топики v1: файлы вне тестов и docs (src docker scripts architecture tools .github) ==="
for t in /voice/dialogue/state /voice/dialogue/response /voice/tts/state /voice/tts/finished /voice/tts/request \
         /voice/tts/control /dialogue/control; do
  git grep -n -F -e "$t" "$REF" -- src docker scripts architecture tools .github \
    | sed "s|^$REF:||" | grep -v "$NT" > $TMP/topic.txt
  echo "--- $t: строк $(count < $TMP/topic.txt), файлов $(cut -d: -f1 $TMP/topic.txt | sort -u | count)"
  per_file < $TMP/topic.txt
done

echo; echo "=== флаг dialog_engine (вне тестов, без rob_box_core/dialog_engine.py) ==="
git grep -n -e dialog_engine "$REF" -- src docker scripts .github | sed "s|^$REF:||" | grep -v "$NT" \
  | grep -v '^src/rob_box_core/rob_box_core/dialog_engine.py:' > $TMP/engine.txt
echo "  строк $(count < $TMP/engine.txt), файлов $(cut -d: -f1 $TMP/engine.txt | sort -u | count)"
per_file < $TMP/engine.txt
git grep -n -e dialog_engine "$OLD" -- src docker scripts .github | sed "s|^$OLD:||" | grep -v "$NT" \
  | grep -v '^src/rob_box_core/rob_box_core/dialog_engine.py:' > $TMP/engine_old.txt
echo "  на $OLD: строк $(count < $TMP/engine_old.txt), файлов $(cut -d: -f1 $TMP/engine_old.txt | sort -u | count)"

echo; echo "=== зависимости вне src (B6) ==="
git grep -n -e skip_reason_checker -e dialogue_skip_reasons "$REF" -- scripts .github | sed "s|^$REF:||"
git grep -n -e dialogue_state_machine -e speak_helpers -e dialogue_text "$REF" -- scripts | sed "s|^$REF:||"
echo "--- имя ROS-ноды /dialogue_node вне src (не путь к файлу):"
git grep -n -E -e '/dialogue_node([^._a-z]|$)' "$REF" -- scripts .github docker | sed "s|^$REF:||"
echo "--- строковое имя dialogue_node в живом v2 (подпись /mcp/execute и срез тулов):"
git grep -n -e PERSONALITY_SENDER "$REF" -- src | sed "s|^$REF:||" | grep -v "$NT"
git grep -n -w -e dialogue_node "$REF" -- src/rob_box_mcp_tools/rob_box_mcp_tools/data/slice_policy.yaml | sed "s|^$REF:||"
echo "--- v1 собирает допуск реплики из хвоста stt_admission (DefaultSttAdmission):"
git grep -n -e DefaultSttAdmission "$REF" -- src/rob_box_voice/rob_box_voice/dialogue_node.py | sed "s|^$REF:||"

echo; echo "=== /mcp/execute: где создаются издатели (для находки duplicate_publishers) ==="
git grep -n -e 'create_publisher(String, "/mcp/execute"' "$REF" -- src | sed "s|^$REF:||" | grep -v "$NT"
git grep -n -e 'LLMToolCallAdapter(' -e '_mcp_provider(' "$REF" -- src/rob_box_voice/rob_box_voice/dialog_node.py \
  | sed "s|^$REF:||"

echo; echo "=== коммиты между картами ==="
echo "  все коммиты $OLD..$REF: $(git rev-list --count "$OLD..$REF")"
echo "  в src/rob_box_voice src/rob_box_dialog src/rob_box_harness src/rob_box_core: $(git rev-list --count "$OLD..$REF" -- src/rob_box_voice src/rob_box_dialog src/rob_box_harness src/rob_box_core)"
echo "  только в файлах карты (rob_box_voice/rob_box_voice, rob_box_dialog/rob_box_dialog, harness/core):" \
  "$(git rev-list --count "$OLD..$REF" -- src/rob_box_voice/rob_box_voice src/rob_box_dialog/rob_box_dialog src/rob_box_harness/rob_box_harness/core)"
