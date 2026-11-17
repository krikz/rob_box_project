#!/usr/bin/env bash
# Регресс-тест scripts/studio/studio_ro_gate.sh (issue #4492, доступ студии только на чтение).
#
# Хосты не нужны: гейт запускается локально с HOME во временном каталоге и командой в
# SSH_ORIGINAL_COMMAND, как его зовёт sshd. docker, scp и git подменены фейками в PATH:
# они печатают свой argv, и тест сверяет, что именно гейт исполнил. ls/stat/cat/head/tail/sha256sum —
# настоящие.
#
# Что проверяется:
#   1. bash -n гейта и install_ro_key.sh;
#   2. каждый разрешённый случай проходит (код 0, ожидаемый вывод, строка allow в gate.log);
#   3. каждая попытка обхода отбивается (код 126, строка deny в gate.log, фейк не вызван);
#   4. после всех попыток файлы в корне не изменились и новых не появилось;
#   5. install_ro_key.sh без --apply ничего не делает и печатает строку с forced command.
#
# Запуск: bash scripts/studio/tests/test_studio_ro_gate.sh

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
GATE="$REPO_ROOT/scripts/studio/studio_ro_gate.sh"
INSTALL="$REPO_ROOT/scripts/studio/install_ro_key.sh"

WORK="$(mktemp -d)"
WORK="$(cd "$WORK" && pwd -P)"
trap 'rm -rf "$WORK"' EXIT

pass=0
failures=0
ok() { echo "PASS: $1"; pass=$((pass + 1)); }
ko() { echo "FAIL: $1"; failures=$((failures + 1)); }

# ── 1. синтаксис ──────────────────────────────────────────────────────────
for f in "$GATE" "$INSTALL"; do
    if bash -n "$f"; then ok "bash -n ${f#"$REPO_ROOT"/}"; else ko "bash -n ${f#"$REPO_ROOT"/}"; fi
done

# ── стенд ─────────────────────────────────────────────────────────────────
HOME_T="$WORK/home"
ROOT="$WORK/srv/out"
SECRET="$WORK/secret"
REPO="$WORK/repo"
OTHER_REPO="$WORK/other_repo"
mkdir -p "$HOME_T/.config/studio_ro" "$ROOT/sub" "$ROOT/data/faces" "$SECRET" "$REPO" "$OTHER_REPO" "$WORK/bin"
printf 'line1\nline2\nline3\n' >"$ROOT/report.txt"
printf 'clip\n' >"$ROOT/sub/clip.mp4"
printf 'face\n' >"$ROOT/data/faces/x.jpg"
printf 'voices\n' >"$ROOT/speakers.db"
printf 'TOP SECRET\n' >"$SECRET/passwd.txt"
mkdir -p "$SECRET/dir"
printf 'TOP SECRET\n' >"$SECRET/dir/f.txt"

symlinks=1
export MSYS=winsymlinks:nativestrict # Git Bash на Windows: настоящие симлинки или ошибка
ln -s "$SECRET/passwd.txt" "$ROOT/link_out" 2>/dev/null || symlinks=0
ln -s "$SECRET/dir" "$ROOT/link_dir_out" 2>/dev/null || symlinks=0
ln -s "$ROOT/report.txt" "$ROOT/link_in" 2>/dev/null || symlinks=0
[[ -L "$ROOT/link_out" ]] || symlinks=0

cat >"$HOME_T/.config/studio_ro/roots" <<EOF
# корень стенда
$ROOT
/data
/
relative/path
$WORK/does-not-exist
EOF
printf '%s\n' nav2 oak-d '# voice-assistant' 'bad;name' >"$HOME_T/.config/studio_ro/containers"
printf '%s\n' "$REPO" >"$HOME_T/.config/studio_ro/repos"

for b in docker scp git; do
    cat >"$WORK/bin/$b" <<EOF
#!/usr/bin/env bash
echo "FAKE-$b \$*"
EOF
    chmod +x "$WORK/bin/$b"
done

snapshot() { (cd "$ROOT" && find . -exec ls -ld {} + 2>/dev/null | awk '{print $1, $5, $NF}' | sort && cat report.txt); }
BEFORE="$(snapshot)"

LOG="$HOME_T/.local/state/studio_ro/gate.log"
OUT=""
RC=0
run_gate() {
    OUT="$(HOME="$HOME_T" PATH="$WORK/bin:$PATH" SSH_CLIENT="10.1.1.156 50000 22" \
        SSH_ORIGINAL_COMMAND="$1" bash "$GATE" 2>&1)"
    RC=$?
}
last_log() { tail -n 1 "$LOG" 2>/dev/null; }

allow() { # allow «имя» «команда» «ожидаемая подстрока вывода»
    run_gate "$2"
    if [[ $RC -eq 0 && "$OUT" == *"$3"* && "$(last_log)" == *$'\tallow\t'* ]]; then
        ok "allow: $1"
    else
        ko "allow: $1 (rc=$RC, out=${OUT:0:200}, log=$(last_log))"
    fi
}

deny() { # deny «имя» «команда» [«подстрока причины»]
    run_gate "$2"
    if [[ $RC -eq 126 && "$OUT" != *FAKE-* && "$OUT" != *"TOP SECRET"* && "$OUT" == *"deny:"* &&
        "$(last_log)" == *$'\tdeny\t'* && "$OUT" == *"${3-}"* ]]; then
        ok "deny:  $1 — ${OUT#studio_ro_gate: deny: }"
    else
        ko "deny:  $1 (rc=$RC, out=${OUT:0:200}, log=$(last_log))"
    fi
}

# ── 2. разрешённое ────────────────────────────────────────────────────────
allow "ls -la корня" "ls -la $ROOT" "report.txt"
allow "ls подкаталога" "ls $ROOT/sub" "clip.mp4"
allow "stat файла" "stat $ROOT/report.txt" "report.txt"
allow "cat файла" "cat $ROOT/report.txt" "line2"
allow "cat двух файлов" "cat $ROOT/report.txt $ROOT/sub/clip.mp4" "clip"
allow "head -n 1" "head -n 1 $ROOT/report.txt" "line1"
allow "tail -n 1" "tail -n 1 $ROOT/report.txt" "line3"
allow "sha256sum" "sha256sum $ROOT/report.txt" "report.txt"
allow "scp -f (выдача файла)" "scp -f $ROOT/report.txt" "FAKE-scp -f -- $ROOT/report.txt"
allow "scp -p -f" "scp -p -f $ROOT/sub/clip.mp4" "FAKE-scp -p -f -- $ROOT/sub/clip.mp4"
allow "docker logs --tail 50 nav2" "docker logs --tail 50 nav2" "FAKE-docker logs --tail 50 -- nav2"
allow "docker logs --timestamps --tail 2000 oak-d" "docker logs --timestamps --tail 2000 oak-d" \
    "FAKE-docker logs --timestamps --tail 2000 -- oak-d"
allow "git log --oneline -n 5" "git -C $REPO log --oneline -n 5" "FAKE-git --no-pager -C $REPO log --oneline -n 5"
allow "git show --stat HEAD" "git -C $REPO show --stat HEAD" "FAKE-git --no-pager -C $REPO show --stat HEAD"
allow "git status --short" "git -C $REPO status --short" "FAKE-git --no-pager -C $REPO status --short"
allow "git log ревизия и путь" "git -C $REPO log --oneline origin/develop -- docs/channel" \
    "log --oneline origin/develop -- docs/channel"
if ((symlinks)); then
    allow "симлинк внутри корня" "cat $ROOT/link_in" "line1"
else
    echo "SKIP: симлинки не создаются на этой ФС (allow симлинк внутри корня)"
fi

# ── 3. попытки обхода ─────────────────────────────────────────────────────
deny "../ из корня" "cat $ROOT/../../secret/passwd.txt"
deny "../ через подкаталог" "cat $ROOT/sub/../../../secret/passwd.txt"
deny "ls ../ к корню ФС" "ls $ROOT/../../.."
if ((symlinks)); then
    deny "симлинк на файл снаружи" "cat $ROOT/link_out"
    deny "симлинк на каталог снаружи" "ls $ROOT/link_dir_out"
    deny "scp симлинка снаружи" "scp -f $ROOT/link_out"
else
    echo "SKIP: симлинки не создаются на этой ФС (deny симлинк ×3)"
fi
deny "; вторая команда" "cat $ROOT/report.txt; cat $SECRET/passwd.txt"
deny "; без пробела" "cat $ROOT/report.txt;id"
deny "&& вторая команда" "cat $ROOT/report.txt && cat $SECRET/passwd.txt"
deny "|| вторая команда" "cat /nope || cat $SECRET/passwd.txt"
deny "| пайп" "cat $ROOT/report.txt | sh"
deny "& фон" "cat $ROOT/report.txt & cat $SECRET/passwd.txt"
deny "\$() подстановка" "cat \$(echo $SECRET/passwd.txt)"
deny "\${} переменная" "cat \${HOME}/x"
deny "backticks" "cat \`echo $SECRET/passwd.txt\`"
deny "одинарные кавычки" "cat '$ROOT/report.txt'"
deny "двойные кавычки" "cat \"$ROOT/report.txt\""
deny "> редирект (запись)" "cat $ROOT/report.txt > $ROOT/x"
deny "< редирект" "cat < $SECRET/passwd.txt"
deny "перевод строки" "cat $ROOT/report.txt"$'\n'"cat $SECRET/passwd.txt"
deny "табуляция" "cat"$'\t'"$ROOT/report.txt"
deny "обратный слэш" "cat $ROOT/report\\.txt"
deny "glob *" "cat $ROOT/*"
deny "тильда" "cat ~/x"
deny "scp -t (загрузка на хост)" "scp -t $ROOT"
deny "scp -t с -p" "scp -p -t $ROOT/x"
deny "scp -r -f" "scp -r -f $ROOT"
deny "scp -f каталога" "scp -f $ROOT/sub"
deny "scp -f двух файлов" "scp -f $ROOT/report.txt $ROOT/sub/clip.mp4"
deny "scp без -f" "scp $ROOT/report.txt"
deny "docker exec" "docker exec nav2 sh"
deny "docker run" "docker run --rm -v /:/h alpine cat /h/etc/shadow"
deny "docker ps" "docker ps"
deny "docker logs длинный N (2001)" "docker logs --tail 2001 nav2"
deny "docker logs огромный N" "docker logs --tail 99999999 nav2"
deny "docker logs N=0" "docker logs --tail 0 nav2"
deny "docker logs неизвестный контейнер" "docker logs --tail 50 voice-assistant"
deny "docker logs закомментированный контейнер" "docker logs --tail 50 #voice-assistant"
deny "docker logs без --tail" "docker logs nav2"
deny "docker logs --follow" "docker logs --follow --tail 50 nav2"
deny "docker logs лишний аргумент" "docker logs --tail 50 nav2 oak-d"
deny "путь в /data" "cat /data/speakers.db" "приватный путь"
deny "ls /data (корень /data из конфига отброшен)" "ls /data" "приватный путь"
deny "/etc (жёсткий запрет)" "cat /etc/hostname" "приватный путь"
deny "/ в конфиге не даёт корень ФС" "cat $SECRET/passwd.txt" "вне разрешённых корней"
deny "data/faces внутри корня" "cat $ROOT/data/faces/x.jpg"
deny "speakers.db внутри корня" "cat $ROOT/speakers.db"
deny "чтение конфига гейта" "cat $HOME_T/.config/studio_ro/roots"
deny "rm" "rm $ROOT/report.txt"
deny "mv" "mv $ROOT/report.txt $ROOT/r2"
deny "cp внутрь корня" "cp $SECRET/passwd.txt $ROOT/p"
deny "tee" "tee $ROOT/report.txt"
deny "sudo" "sudo cat /etc/shadow"
deny "ros2 topic pub" "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist"
deny "bash -c" "bash -c id"
deny "sh" "sh"
deny "sftp-подсистема" "/usr/lib/openssh/sftp-server"
deny "internal-sftp" "internal-sftp"
deny "env" "env"
deny "пустая команда (интерактивный вход)" ""
deny "tail -f" "tail -f $ROOT/report.txt"
deny "head -n огромный" "head -n 9999999 $ROOT/report.txt"
deny "ls -L (follow symlinks)" "ls -L $ROOT"
deny "cat без пути" "cat"
deny "относительный путь" "cat report.txt"
deny "флаг после пути" "cat $ROOT/report.txt -n"
deny "cat каталога" "cat $ROOT/sub"
deny "git push" "git -C $REPO push"
deny "git log --output (запись файла)" "git -C $REPO log --output=$ROOT/x"
deny "git show --ext-diff" "git -C $REPO show --ext-diff"
deny "git -c конфиг" "git -C $REPO -c core.pager=id log"
deny "git чужой репо" "git -C $OTHER_REPO log"
deny "git без -C" "git log"
deny "git checkout" "git -C $REPO checkout ."
deny "git pathspec с .." "git -C $REPO log -- ../x"
deny "длинная команда" "ls $(printf 'a%.0s' {1..3000})"

# ── 4. ничего не изменилось ───────────────────────────────────────────────
AFTER="$(snapshot)"
if [[ "$BEFORE" == "$AFTER" ]]; then ok "файлы корня не изменились"; else ko "файлы корня изменились"; fi
n_allow=$(grep -c $'\tallow\t' "$LOG" || true)
n_deny=$(grep -c $'\tdeny\t' "$LOG" || true)
if [[ "$(awk -F'\t' 'NF != 5' "$LOG" | wc -l)" -eq 0 && "$(grep -c '10.1.1.156' "$LOG")" -eq $((n_allow + n_deny)) ]]; then
    ok "gate.log: $n_allow allow, $n_deny deny, у каждой строки 5 полей и адрес клиента"
else
    ko "gate.log: формат строк"
fi

# ── 5. install_ro_key.sh: сухой прогон ────────────────────────────────────
printf 'ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIStudioTestKeyOnlyForTests000000000000000 studio@builder\n' >"$WORK/k.pub"
dry="$(PATH="$WORK/bin:$PATH" bash "$INSTALL" 10.1.1.21 "$WORK/k.pub" 2>&1)"
rc=$?
want='command="/home/ros2/.local/bin/studio_ro_gate.sh",no-pty,no-port-forwarding,no-agent-forwarding,no-X11-forwarding,restrict ssh-ed25519 '
if [[ $rc -eq 0 && "$dry" == *"$want"* && "$dry" == *"профиль vision"* && "$dry" == *"ничего не сделано"* ]]; then
    ok "install_ro_key.sh без --apply: печатает строку, профиль vision"
else
    ko "install_ro_key.sh без --apply (rc=$rc): $dry"
fi
printf 'command="x" ssh-ed25519 AAAA studio\n' >"$WORK/bad.pub"
if ! bash "$INSTALL" 10.1.1.21 "$WORK/bad.pub" >/dev/null 2>&1; then
    ok "install_ro_key.sh отбивает ключ с опциями"
else
    ko "install_ro_key.sh принял ключ с опциями"
fi
if ! bash "$INSTALL" 10.9.9.9 "$WORK/k.pub" >/dev/null 2>&1; then
    ok "install_ro_key.sh: неизвестный хост без --profile — отказ"
else
    ko "install_ro_key.sh: неизвестный хост принят"
fi
for p in sandbox katana main vision sound; do
    if grep -qE '^/data(/|$)' "$REPO_ROOT/scripts/studio/ro_config/$p/roots"; then
        ko "ro_config/$p/roots содержит /data"
    else
        ok "ro_config/$p/roots без /data"
    fi
done

echo
echo "Итого: PASS=$pass FAIL=$failures"
((failures == 0))
