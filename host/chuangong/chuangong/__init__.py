"""chuangong (传功) — агент доставки релиза на Pi робота (ADR-0168 и безопасный срез C1).

Модули:
  minisign       — формат ключей и подписей minisign (Ed25519 через `cryptography`), один на CI и агента;
  release_format — схема release.yaml, раскладка OCI-артефакта релиза, проверка целостности;
  registry       — клиент OCI Distribution API (stdlib urllib), докачка блобов;
  zenoh_rest     — REST-плагин zenoh (:8000): put / get / подписка SSE;
  state          — запись состояния агента, ключи zenoh, метрика rbx_release_info;
  stage          — stage роли: место на диске, бандл, образы по digest;
  cancel         — локальная отмена staged (CLI cancel) и чтение маркера тиком/preflight; current не трогать;
  agent          — один тик агента (fetch → verify → stage → report);
  activation     — adoption доказанного current и dry-run activation/rollback (C1);
  role_switch    — план/исполнитель одной роли по digest (checks deploy); activate зовёт plan_role;
  pointers       — файлы current/previous в state_dir после проверки подписи staged-релиза;
  status         — наблюдатель для workflow «F: Update Robot» (action: status).

Живой symlink `current` и хостовые миграции — следующий шаг; compose в тестах через FakeDocker.
"""

AGENT_VERSION = "0.2.3"
