# Доступ студии канала к хостам: только чтение

Студия канала (агент на машине builder `10.1.1.156`, [бриф](../process/channel-studio-brief.md), issue #4492)
сама забирает материал для постов: ролики и отчёты HIL с песочницы, артефакты сборки с katana, логи
контейнеров и `docs` робота. Решение Шифу 06.11: доступ **только на чтение**, через ключ студии с forced command.
Ключ на хосты ставит координатор и только после «да» Шифу в чате приложения.

| Хост | Адрес | Профиль | Что читает студия |
|---|---|---|---|
| Песочница | `10.1.1.250` | `sandbox` | `/home/ros2/rob_box_hilvideo/out`, `/home/ros2/rob_box_hil/reports`, `git log` чекаута HIL |
| katana | `10.1.1.249` | `katana` | `docs`, `/opt/rob_box/{vendor,models}`, логи раннеров `build-github-runner-1..8` |
| Main Pi | `10.1.1.10` / `.20` | `main` | `~/rob_box_project/docs`, логи навигации (nav2, rtabmap, ros2-control …) |
| Vision Pi | `10.1.1.11` / `.21` | `vision` | `docs`, логи oak-d, led-matrix, ceiling-camera, vision-hailo, vision-face |
| Sound Pi | `10.1.1.12` / `.22` | `sound` | `docs`, логи supercollider, sensor-bridge, avatar-arbiter, zenoh, voice-assistant, avatar-supervisor, telegram-bot |

Списки — в `scripts/studio/ro_config/<профиль>/{roots,containers,repos}`.

## 1. Как устроено

- В `authorized_keys` хоста ключ студии стоит строкой
  `command="/home/ros2/.local/bin/studio_ro_gate.sh",no-pty,no-port-forwarding,no-agent-forwarding,no-X11-forwarding,restrict ssh-ed25519 …`.
  Что бы студия ни попросила, sshd запускает гейт `scripts/studio/studio_ro_gate.sh`. Запрошенная команда лежит в
  `$SSH_ORIGINAL_COMMAND`.
- Гейт пропускает только:
  - `ls [-lahtr1RS]`, `stat`, `cat`, `head/tail [-n N]` (N ≤ 100000), `sha256sum` по абсолютным путям внутри корней
    из `~/.config/studio_ro/roots`;
  - `scp -f <файл>`: выдача **одного** файла из корней. Загрузка на хост (`scp -t`), `-r` и каталоги запрещены;
  - `docker logs [--timestamps] --tail N <имя>`: N от 1 до 2000, имя из `~/.config/studio_ro/containers`;
  - `git -C <репо> log|show|status` с безопасными опциями (`--oneline`, `-n N`, `--stat`, `--since=…` и т. п.)
    для репо из `~/.config/studio_ro/repos`.
- Разбор без `eval`. Сначала проверяется набор символов: разрешены буквы, цифры, `_ . / : = @ + , -` и пробел.
  Поэтому кавычки, `;`, `&`, `|`, `$()`, backticks, редиректы, `*`, `~` и перевод строки отбиваются ещё до разбора.
  Потом строка делится на массив аргументов, и каждый путь нормализуется через `realpath -e`. `..` и симлинки
  наружу корня получают отказ. Исполняется уже нормализованный путь.
- **Жёсткий запрет приватного** действует, даже если корень по ошибке попал в конфиг: `/data` и `*/data/faces*`,
  `*/data/voices*`, файлы `*.db`, `*.sqlite`, `.env`, ключи, `*.npy` (эмбеддинги), `*.wav/*.ogg/*.flac` (голоса),
  `/etc`, `/root`, `/proc`, `/sys`, `/dev`, `~/.ssh`, сам конфиг гейта. Корень `/` отбрасывается.
- Логи `voice-assistant`, `avatar-supervisor`, `telegram-bot` (речь людей мастерской, диалог, переписка) и
  `vision-face` (имена узнанных людей) **открыты** — решение Шифу 06.11 «все логи открывай». Файлы `/data`
  (лица, голоса, эмбеддинги, базы) по-прежнему под жёстким запретом гейта. Что из логов попадает в посты, решает
  правило приватности канала (`docs/process/telegram-channel.md`): людей не публикуем.
- Каждый вызов пишется строкой в `~/.local/state/studio_ro/gate.log`:
  `время UTC<TAB>allow|deny<TAB>причина<TAB>IP клиента<TAB>команда`. Если лог записать нельзя, команда
  не исполняется.

## 2. Как поставить (координатор, после «да» Шифу)

1. Студия присылает публичный ключ (`~/.ssh/studio_ro.pub` на builder). Нужен ключ без опций, отдельный от её
   прочих ключей.
2. Сухой прогон: ничего не меняет, печатает строку и пути.

   ```bash
   scripts/studio/install_ro_key.sh 10.1.1.250 studio_ro.pub
   ```

3. Сверить с хостом корни из профиля: существуют ли каталоги и нет ли в них людей. Для katana каталоги артефактов
   выбраны по коду (`/opt/rob_box/vendor`, `/opt/rob_box/models`), а не по `ls` на хосте; при установке их надо
   проверить и поправить профиль PR-ом.
4. Установка по ключу координатора (`BatchMode`, пароли запрещены):

   ```bash
   scripts/studio/install_ro_key.sh 10.1.1.250 studio_ro.pub --apply
   scripts/studio/install_ro_key.sh 10.1.1.249 studio_ro.pub --apply
   scripts/studio/install_ro_key.sh 10.1.1.10  studio_ro.pub --apply
   scripts/studio/install_ro_key.sh 10.1.1.21  studio_ro.pub --apply
   scripts/studio/install_ro_key.sh 10.1.1.22  studio_ro.pub --apply
   ```

   Скрипт кладёт гейт в `~/.local/bin/`, конфиги в `~/.config/studio_ro/` и дописывает строку, только если её
   ещё нет. Если тот же ключ уже стоит **с другими опциями** (например, без гейта), скрипт останавливается с кодом 3.
   Такую строку убирают руками, иначе гейт обходится.
5. Не путать с ключом координатора: ключ координатора остаётся полным, ограничен только ключ студии.

## 3. Как проверить (с builder, ключом студии)

```bash
H=ros2@10.1.1.250
ssh -i ~/.ssh/studio_ro $H 'ls -la /home/ros2/rob_box_hilvideo/out'        # список
scp -O -i ~/.ssh/studio_ro $H:/home/ros2/rob_box_hilvideo/out/route.log .  # -O обязателен: legacy scp
ssh -i ~/.ssh/studio_ro ros2@10.1.1.10 'docker logs --tail 50 nav2'
ssh -i ~/.ssh/studio_ro ros2@10.1.1.10 'git -C /home/ros2/rob_box_project log --oneline -n 5'
# должны получить «studio_ro_gate: deny: …» и код 126:
ssh -i ~/.ssh/studio_ro ros2@10.1.1.10 'docker exec nav2 id'
ssh -i ~/.ssh/studio_ro ros2@10.1.1.22 'cat /data/speakers.db'
ssh -i ~/.ssh/studio_ro $H                                                # интерактивный вход
sftp -i ~/.ssh/studio_ro $H                                               # sftp-подсистема
```

`scp` без `-O` в OpenSSH 9+ идёт через sftp, а sftp гейт не пропускает. Студия всегда вызывает `scp -O`.

На хосте после проверки: `tail -n 20 ~/.local/state/studio_ro/gate.log`, где видны строки allow и deny.
Регресс-тест гейта без хостов: `bash scripts/studio/tests/test_studio_ro_gate.sh` (идёт в CI, `G: Lint Code` →
Shell Scripts).

## 4. Как отозвать

На каждом хосте (ключом координатора) убрать строку ключа студии и, по желанию, сам гейт:

```bash
ssh ros2@<хост> "sed -i.bak '/studio_ro_gate.sh/d' ~/.ssh/authorized_keys && grep -c studio_ro_gate ~/.ssh/authorized_keys || true"
ssh ros2@<хост> 'rm -f ~/.local/bin/studio_ro_gate.sh; rm -rf ~/.config/studio_ro'   # лог в ~/.local/state оставить
```

`grep -c` должен вернуть `0`. Срочно, без правки `authorized_keys`, можно удалить `~/.config/studio_ro/`: без
корней, контейнеров и репо гейт отказывает в любой команде.

## 5. Как менять списки

Только PR-ом в `scripts/studio/ro_config/<профиль>/`, потом повторный `install_ro_key.sh … --apply` (он перезапишет
конфиги, строку ключа не продублирует). Руками на хостах не править. Новый корень — с проверкой, что в нём нет
людей: лица, голоса, имена, `speakers.db`.
