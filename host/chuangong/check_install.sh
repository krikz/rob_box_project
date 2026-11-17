#!/usr/bin/env bash
# Проверка установки агента chuangong на хосте (ADR-0168): только чтение, ничего не включает и не пишет.
#
#   bash host/chuangong/check_install.sh [--expect-enabled]
#
# Проверяет: юниты в systemd, таймер выключен (или включён с --expect-enabled), каталоги, режим observe,
# `chuangong run --dry-run` отрабатывает и ничего не пишет в /var/lib/chuangong. Выход 0 — всё на месте.
set -uo pipefail

EXPECT_ENABLED=0
[ "${1:-}" = "--expect-enabled" ] && EXPECT_ENABLED=1
fail=0
ok() { echo "OK   $*"; }
bad() { echo "FAIL $*"; fail=1; }

units="$(systemctl list-unit-files --no-pager chuangong.service chuangong.timer 2>&1)"
echo "$units" | grep -q '^chuangong.service' && ok "chuangong.service в systemd" || bad "нет chuangong.service: $units"
echo "$units" | grep -q '^chuangong.timer' && ok "chuangong.timer в systemd" || bad "нет chuangong.timer"

state="$(systemctl is-enabled chuangong.timer 2>/dev/null || true)"
if [ "$EXPECT_ENABLED" = 1 ]; then
    [ "$state" = enabled ] && ok "таймер включён" || bad "таймер должен быть включён, сейчас: $state"
else
    [ "$state" != enabled ] && ok "таймер не включён ($state)" || bad "таймер включён, а не должен"
fi

for d in /opt/rob_box/releases /var/lib/chuangong /etc/chuangong /opt/rob_box/chuangong/chuangong; do
    [ -d "$d" ] && ok "каталог $d" || bad "нет каталога $d"
done
grep -q '^CHUANGONG_MODE=observe$' /etc/chuangong/chuangong.env 2>/dev/null \
    && ok "режим observe в /etc/chuangong/chuangong.env" || bad "в /etc/chuangong/chuangong.env не observe"
python3 -c 'import yaml, cryptography' 2>/dev/null && ok "python3-yaml и python3-cryptography импортируются" \
    || bad "python3 не импортирует yaml/cryptography"

before="$(ls -la /var/lib/chuangong 2>&1)"
out="$(chuangong run --dry-run 2>&1)"
rc=$?
after="$(ls -la /var/lib/chuangong 2>&1)"
echo "chuangong run --dry-run → rc=$rc"
echo "$out" | tail -n 1
if [ "$rc" = 0 ] && echo "$out" | tail -n 1 | python3 -c 'import json,sys; r=json.loads(sys.stdin.read()); assert r["dry_run"] is True and r["state"]' 2>/dev/null; then
    ok "dry-run отработал, запись тика — JSON с dry_run=true"
else
    bad "dry-run не отработал"
fi
[ "$before" = "$after" ] && ok "dry-run ничего не записал в /var/lib/chuangong" || bad "dry-run изменил /var/lib/chuangong"

exit "$fail"
