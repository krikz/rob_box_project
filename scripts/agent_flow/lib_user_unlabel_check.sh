#!/bin/bash
# ============================================================================
# lib_user_unlabel_check.sh — shared helper for «user-unlabel respect» guard.
#
# SOT: <repo>/scripts/agent_flow/lib_user_unlabel_check.sh
# Копии раскладываются install.sh в:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
#
# Назначение (ретро 18.08 t_de6bea69, PR #1398):
#   Автоматика (e2e-process, merge-gate reconcile, merge-gate lint path)
#   ставит метки `e2e-done` / `needs-review` на PR после успешного e2e.
#   Товарищ Шифу имеет ЭКСКЛЮЗИВНОЕ право решать, когда PR готов к ревью
#   (Q22). Если он РУКАМИ снял метку после того как auto-sweep её поставил —
#   следующий тик sweep'а НЕ должен возвращать эту метку обратно
#   (наблюдение 18.08: needs-review на PR #1398 возвращалась 5 раз подряд
#   за 2.5ч после ручного unlabel).
#
# Сигнал «user removed» из GitHub timeline:
#   UnlabeledEvent{label=L} ПОЗЖЕ последнего LabeledEvent{label=L}.
#   Actor-фильтр ослабленный (любой не-bot актор) — в этом репо krikz
#   является одновременно юзером И держателем GH-токена автоматики, так
#   что actor-фильтр по имени не различает. Реальный discriminator — это
#   именно ХРОНОЛОГИЯ: если последнее событие по метке — unlabel, значит
#   кто-то её сознательно снял, и автоматика не должна возвращать.
#
# Использование (после source):
#   if user_removed_label_recently "$pr_number" "$LABEL_NAME"; then
#       log "user previously removed $LABEL_NAME — respecting"
#       # пропустить auto-установку этой метки
#   fi
#
# Гарантии (ADR-0014 §4 req 4, conservative on uncertainty):
#   - timeline API сдох → return 1 (НЕ блокируем — fail-OPEN, старая метка
#     остаётся, не ломаем нормальный прогон; зато если unlabel был —
#     вернётся метка, что плохо, но лучше чем fail-CLOSE на каждом тике)
#   - событий нет → return 1 (нет сигнала, идём как обычно)
#   - последнее событие labeled (auto) → return 1 (не было user intervention)
#   - последнее событие unlabeled (любой не-bot actor) → return 0
#
# Test mode (used by tests/test_user_unlabel_check.sh):
#   export USER_UNLABEL_TEST_MODE=1 — пропускает реальные API-вызовы;
#   тесты инжектят мок-данные через переменные _USER_UNLABEL_TEST_JSON
#   и _USER_UNLABEL_TEST_PR.
# ============================================================================

# Защита от двойного source
if [ -n "${_LIB_USER_UNLABEL_CHECK_LOADED:-}" ]; then
    return 0
fi
_LIB_USER_UNLABEL_CHECK_LOADED=1

# Returns 0 (true) если последнее значимое событие по метке — это
# UnlabeledEvent не от bot'а, и оно ПОЗЖЕ последнего LabeledEvent по этой
# же метке. Иначе 1.
# Args: $1=pr_number $2=label_name
user_removed_label_recently() {
    local _pr="$1" _lbl="$2"

    # Test mode: берём мок-данные из переменных (НЕ зовём gh api).
    local _events
    if [ "${USER_UNLABEL_TEST_MODE:-0}" = "1" ]; then
        _events="${_USER_UNLABEL_TEST_JSON:-[]}"
        if [ -z "$_events" ] || [ "$_events" = "[]" ]; then
            return 1
        fi
    else
        # Production: тянем timeline PR (PR и issue шарится один endpoint)
        _events="$(gh api "repos/${GH_REPO}/issues/${_pr}/timeline?per_page=100" \
            2>/dev/null || echo '[]')"
        if [ -z "$_events" ] || [ "$_events" = '[]' ]; then
            return 1
        fi
    fi

    # Парсим в python: возвращаем два epoch (last_unlabel_user, last_label)
    # через \n — bash читает их по строкам.
    local _out _unlabel _label
    _out="$(printf '%s' "$_events" | _lbl="$_lbl" python3 -c '
import json, sys, os
from datetime import datetime

raw = sys.stdin.read()
try:
    data = json.loads(raw)
except Exception:
    print(0); print(0); sys.exit(0)
if not isinstance(data, list):
    print(0); print(0); sys.exit(0)

target = (os.environ.get("_lbl") or "").lower()

def is_bot(login):
    if not login:
        return True
    s = login.lower()
    if s.endswith("[bot]") or "-bot" in s or s.endswith("_bot"):
        return True
    if s in ("github-actions", "github-actions[bot]", "web-flow",
             "dependabot[bot]", "renovate[bot]", "codecov[bot]"):
        return True
    return False

last_unlabel = 0
last_label = 0
for ev in data:
    if not isinstance(ev, dict):
        continue
    e = ev.get("event")
    if e not in ("labeled", "unlabeled"):
        continue
    name = ((ev.get("label") or {}).get("name") or "").lower()
    if name != target:
        continue
    at = ev.get("created_at") or ""
    try:
        dt = datetime.fromisoformat(at.replace("Z", "+00:00"))
        epoch = int(dt.timestamp())
    except Exception:
        continue
    actor = (ev.get("actor") or {}).get("login") or ""
    if e == "unlabeled":
        # bot-actor (auto-cleanup, не считается user-decisional).
        if not is_bot(actor):
            last_unlabel = max(last_unlabel, epoch)
    elif e == "labeled":
        last_label = max(last_label, epoch)

print(last_unlabel)
print(last_label)
')"

    _unlabel="$(printf '%s\n' "$_out" | sed -n '1p')"
    _label="$(printf '%s\n' "$_out" | sed -n '2p')"

    # Пусто или 0 → нет значимого user-unlabel.
    if [ -z "$_unlabel" ] || [ "$_unlabel" = "0" ]; then
        return 1
    fi
    # User-unlabel ПОЗЖЕ последнего label → respect.
    if [ "${_unlabel:-0}" -gt "${_label:-0}" ] 2>/dev/null; then
        return 0
    fi
    return 1
}

# Удобный helper для логирования причины (когда user_removed_label_recently
# вернул 0). Не делает никаких API-вызовов.
user_unlabel_log_skip() {  # $1=pr $2=label $3=context
    local _pr="$1" _lbl="$2" _ctx="$3"
    log "[user-unlabel-guard] PR #${_pr}: user previously removed '${_lbl}' — НЕ восстанавливаю (${_ctx})"
}

# Optional standalone runner — для отладки или ad-hoc проверки.
if [ "${USER_UNLABEL_STANDALONE:-0}" = "1" ]; then
    if [ $# -lt 2 ]; then
        echo "usage: USER_UNLABEL_STANDALONE=1 GH_REPO=owner/repo $0 <pr_number> <label_name>" >&2
        exit 2
    fi
    if user_removed_label_recently "$1" "$2"; then
        echo "USER_REMOVED: $1/$2"
        exit 0
    else
        echo "NOT_REMOVED: $1/$2"
        exit 1
    fi
fi