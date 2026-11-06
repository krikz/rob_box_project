#!/usr/bin/env bash
# #4476: REST-диспатч (scripts/dev/gh-workflow-dispatch.sh) и START_FROM_ACT раннера марафона. gh подменён заглушкой:
# настоящий gh не вызывается, workflow не запускается.
set -u
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$D/../../.." && pwd)"
HELPER="$ROOT/scripts/dev/gh-workflow-dispatch.sh"
MARATHON="$ROOT/scripts/e2e/run_night_marathon.sh"
fail=0
ok() { echo "PASS $1"; }
bad() { echo "FAIL $1"; fail=1; }

W="$(mktemp -d)"
trap 'rm -rf "$W"' EXIT
mkdir -p "$W/bin"
# Заглушка gh. GH_STUB_POST_CODES - коды ответов на POST по очереди (последний повторяется); вызовы пишутся в $GH_STUB_LOG.
cat > "$W/bin/gh" <<'EOF'
#!/usr/bin/env bash
echo "gh $*" >> "$GH_STUB_LOG"
case "$*" in
    *"actions/workflows/"*"/dispatches"*)
        n="$(cat "$GH_STUB_COUNT" 2>/dev/null || echo 0)"; n=$((n + 1)); echo "$n" > "$GH_STUB_COUNT"
        read -r -a codes <<<"$GH_STUB_POST_CODES"
        i=$((n - 1)); [ "$i" -ge "${#codes[@]}" ] && i=$((${#codes[@]} - 1))
        code="${codes[$i]}"
        if [ "$code" = 204 ]; then printf 'HTTP/2.0 204 No Content\r\n\r\n'; exit 0; fi
        printf 'HTTP/2.0 %s Server Error\r\n\r\n{"message":"stub boom %s"}\n' "$code" "$code"
        echo "gh: stub boom (HTTP $code)" >&2
        exit 1 ;;
    *"actions/workflows"*)
        echo 4242; exit 0 ;;
    "run list"*) exit 0 ;;
esac
exit 0
EOF
chmod +x "$W/bin/gh"
export PATH="$W/bin:$PATH"
export GH_STUB_LOG="$W/gh.log" GH_STUB_COUNT="$W/count"
export DISPATCH_RETRY_SEC=0

reset_stub() { : > "$GH_STUB_LOG"; rm -f "$GH_STUB_COUNT"; export GH_STUB_POST_CODES="$1"; }

# 1. 500 -> повтор -> 204: exit 0, два POST
reset_stub "500 204"
out="$(bash "$HELPER" "L-E2E Voice Test.yml" develop environment=test 'reason=a b=c' 2>&1)"; rc=$?
posts="$(grep -c 'dispatches' "$GH_STUB_LOG")"
[ "$rc" = 0 ] && [ "$posts" = 2 ] && ok "500 -> повтор -> 204 (rc=0, POST=$posts)" || { bad "500 -> повтор -> 204 (rc=$rc, POST=$posts)"; echo "$out"; }
printf '%s' "$out" | grep -q 'HTTP 500' && ok "повтор залогирован с кодом" || bad "повтор не залогирован: $out"
grep -q -- "-f inputs\[reason\]=a b=c" "$GH_STUB_LOG" && grep -q -- "-f ref=develop" "$GH_STUB_LOG" \
    && ok "inputs[k]=v и ref переданы (значение с '=' цело)" || bad "аргументы POST: $(cat "$GH_STUB_LOG")"

# 2. 500 дважды: exit != 0, stderr с кодом и телом, ровно 2 POST
reset_stub "500"
out="$(bash "$HELPER" "L-E2E Voice Test.yml" develop environment=test 2>&1)"; rc=$?
posts="$(grep -c 'dispatches' "$GH_STUB_LOG")"
[ "$rc" != 0 ] && [ "$posts" = 2 ] && ok "два 500 -> exit=$rc, POST=$posts" || bad "два 500 (rc=$rc, POST=$posts)"
printf '%s' "$out" | grep -q 'HTTP 500' && printf '%s' "$out" | grep -q 'stub boom 500' \
    && ok "ошибка с HTTP-кодом и телом в stderr" || bad "нет кода/тела: $out"

# 3. 4xx не повторяем
reset_stub "422"
out="$(bash "$HELPER" "L-E2E Voice Test.yml" develop 2>&1)"; rc=$?
posts="$(grep -c 'dispatches' "$GH_STUB_LOG")"
[ "$rc" != 0 ] && [ "$posts" = 1 ] && ok "422 без повтора (POST=$posts)" || bad "422 (rc=$rc, POST=$posts)"

# 4. раннер: ошибка диспатча видна в логе марафона
reset_stub "500"
out="$(DISPATCH_WAIT_SEC=0 SKIP_SENTINEL=1 REPORT="$W/rep1" ACTS=1 bash "$MARATHON" 2>&1)"; rc=$?
printf '%s' "$out" | grep -q 'dispatch: .*HTTP 500' && printf '%s' "$out" | grep -q 'dispatch не прошёл (акт 1)' \
    && ok "раннер: ошибка диспатча в логе марафона" || { bad "раннер глушит ошибку"; echo "$out" | tail -15; }
[ "$rc" != 0 ] && grep -q dispatch_failed "$W/rep1/summary.tsv" && ok "раннер: dispatch_failed в summary, exit=$rc" || bad "раннер: summary/exit (rc=$rc)"

# 5. START_FROM_ACT=9: пропущены акты 1..8, prior_steps из их сценариев, первый запускаемый акт - first
reset_stub "204"
out="$(DISPATCH_WAIT_SEC=0 SKIP_SENTINEL=1 REPORT="$W/rep2" START_FROM_ACT=9 bash "$MARATHON" 2>&1)"; rc=$?
posts="$(grep -c 'dispatches' "$GH_STUB_LOG")"
[ "$posts" = 2 ] && ok "START_FROM_ACT=9: диспатчей $posts (акты 9, 10)" || { bad "START_FROM_ACT=9: диспатчей $posts"; echo "$out" | tail -15; }
first="$(grep 'dispatches' "$GH_STUB_LOG" | head -1)"
case "$first" in *"night_marathon_act9_"*) ok "первый диспатч - акт 9" ;; *) bad "первый диспатч не акт 9: $first" ;; esac
case "$first" in *"inputs[e2e_marathon_phase]=first"*) ok "фаза первого акта - first" ;; *) bad "фаза не first: $first" ;; esac
case "$first" in *"n101_"*"=OK"*) ok "prior_steps содержит метки актов 1.. как OK" ;; *) bad "prior_steps без меток акта 1: $first" ;; esac
case "$first" in *"night_marathon_act5_"*|*"scenario_file]=.github/e2e/scenarios/night/night_marathon_act1_"*) bad "пропущенный акт ушёл в диспатч" ;; *) ok "пропущенные акты не диспатчатся" ;; esac
second="$(grep 'dispatches' "$GH_STUB_LOG" | tail -1)"
case "$second" in *"inputs[e2e_marathon_phase]=last"*) ok "акт 10 - last" ;; *) bad "акт 10 не last: $second" ;; esac

# 6. START_FROM_ACT=2c (буквенный номер) и несуществующий акт
reset_stub "204"
out="$(DRY_RUN=1 SKIP_SENTINEL=1 REPORT="$W/rep3" START_FROM_ACT=2c bash "$MARATHON" 2>&1)"; rc=$?
printf '%s' "$out" | grep -q 'пропущены акты \[1,2,2b\]' && ok "START_FROM_ACT=2c пропускает 1,2,2b" || { bad "START_FROM_ACT=2c"; echo "$out" | head -5; }
out="$(DRY_RUN=1 SKIP_SENTINEL=1 REPORT="$W/rep4" START_FROM_ACT=99 bash "$MARATHON" 2>&1)"; rc=$?
[ "$rc" = 2 ] && printf '%s' "$out" | grep -q 'START_FROM_ACT=99' && ok "несуществующий акт -> FATAL rc=2" || bad "START_FROM_ACT=99 (rc=$rc)"

# 7. DRY_RUN ничего не запускает
reset_stub "204"
DRY_RUN=1 SKIP_SENTINEL=1 REPORT="$W/rep5" START_FROM_ACT=5 bash "$MARATHON" >/dev/null 2>&1; rc=$?
[ "$rc" = 0 ] && [ ! -s "$GH_STUB_LOG" ] && ok "DRY_RUN: gh не вызывался" || bad "DRY_RUN вызвал gh (rc=$rc): $(cat "$GH_STUB_LOG")"

exit "$fail"
