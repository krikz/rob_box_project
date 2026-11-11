"""chuangong (传功) — агент доставки релиза на Pi робота (ADR-0168 и безопасный срез C1).

Модули:
  minisign       — формат ключей и подписей minisign (Ed25519 через `cryptography`), один на CI и агента;
  release_format — схема release.yaml, раскладка OCI-артефакта релиза, проверка целостности;
  registry       — клиент OCI Distribution API (stdlib urllib), докачка блобов;
  zenoh_rest     — REST-плагин zenoh (:8000): put / get / подписка SSE;
  state          — запись состояния агента, ключи zenoh, метрика rbx_release_info;
  stage          — stage роли: место на диске, бандл, образы по digest;
  agent          — один тик агента (fetch → verify → stage → report);
  activation     — adoption доказанного current и dry-run activation/rollback (C1);
  status         — наблюдатель для workflow «F: Update Robot» (action: status).

Переключения symlink `current`, `compose down/up` и миграций по-прежнему НЕТ.
"""

AGENT_VERSION = "0.2.0"
