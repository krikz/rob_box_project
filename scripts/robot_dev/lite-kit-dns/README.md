# DNS-патч стенда lite-kit (issue #4553)

Патч для встроенного DNS сервера `equasar_server.py` стенда
[lite-kit](https://github.com/krevetkqqq/lite-kit)
(MIT, привязка к коммиту `8a82ff3`). Живёт в репо, чтобы взятие нового стенда
не повторяло разбор «почему quasar.* не резолвится».

## Что чинит

Встроенный DNS сервера обрабатывал UDP-запросы **в одном потоке**, а первый
апстрим `1.1.1.1` не отвечал (таймаут 2 с) — каждое незнакомое имя блокировало
цикл. Роутер (Cudy) льёт на стенд все имена → очередь росла, `quasar.*` умирали
по таймауту. Патч:

1. отдельный поток на UDP-запрос (`udp_answer`);
2. порядок апстримов `77.88.8.8, 8.8.8.8, 1.1.1.1`;
3. таймаут форварда 1.2 с (`DNS_UPSTREAM_TIMEOUT`).

После фикса на стенде: `quasar: 15/15 ok, avg=0.8 ms` под флудом 60 имён.
Детали: `docs/runbooks/yandex-station-root.md` (раздел «Грабли»), issue #4553.

## Файлы

| Файл | Что это |
|---|---|
| `equasar_server.py.dns.patch` | дифф исходник → пропатченный файл (применяется `patch --binary`) |
| `fixtures/equasar_server.py.bak-20261108` | исходник lite-kit @ `8a82ff3`, к которому применим патч |
| `apply-dns-patch.sh` | применение с проверкой sha256 до/после |
| `selftest-dns-patch.sh` | самопроверка на копии фикстуры (без стенда) |

## Контрольные sha256

| Файл | sha256 |
|---|---|
| исходник `equasar_server.py` (CRLF, Windows checkout) | `d08942a1627cbaa576404a303d548b623a39135500d08978893ef4833a24e6b8` |
| результат после патча | `87bd82bc1c3235be4be6b2838f17410573276078d41d9551ee47bfd5e16c8d31` |

Важно: `core.autocrlf=true` в репо lite-kit даёт CRLF-файл при `git clone` на
Windows — под этот вариант и записан хеш исходника. На Linux-чекауте файл будет
LF, sha256 не совпадёт и скрипт честно откажется — пересчитайте хеш или
конвертируйте файл в CRLF перед применением.

## Применение на стенд

```bash
git clone https://github.com/krevetkqqq/lite-kit C:\lite-kit
# Git Bash:
bash scripts/robot_dev/lite-kit-dns/apply-dns-patch.sh /c/lite-kit/scripts/equasar_server.py
```

Скрипт проверит sha256 исходника, применит дифф и сверит sha256 результата.
Повторный запуск — «уже пропатчен», выход 0.

## Самопроверка (без стенда)

```bash
bash scripts/robot_dev/lite-kit-dns/selftest-dns-patch.sh
```

То же самое гоняет pytest-тест `tests/unit/scripts/test_lite_kit_dns_patch.py`
(входит в `pytest tests/unit`, CI).

Фикстура и дифф закоммичены с `-text` в `.gitattributes` — git не должен
трогать их концы строк, иначе хеши поплывут.
