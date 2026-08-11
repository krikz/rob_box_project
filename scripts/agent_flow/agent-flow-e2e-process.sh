#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-e2e-process.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`, который создаёт
# символические ссылки в:
#   - ~/.hermes/profiles/agent-flow/scripts/agent-flow-e2e-process.sh
#   - ~/.hermes/profiles/architect/scripts/agent-flow-e2e-process.sh
#   - ~/.hermes/scripts/agent-flow-e2e-process.sh
# Правка: редактируем <repo>/scripts/agent_flow/agent-flow-e2e-process.sh, commit, merge.
# На хост: bash <repo>/scripts/agent_flow/install.sh (или вручную cp + ln -sf).
# Если ты правишь этот файл НА ХОСТЕ руками — синхронизируй обратно в репо.
# ============================================================================
