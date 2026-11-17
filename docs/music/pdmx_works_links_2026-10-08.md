# Произведения ↔ партитуры пака (ADR-0155 K-3), 2026-10-08

Сырой вывод `scripts/music/works_pdmx.py` (офлайн, без сети). В git только числа и id партитур PDMX. Реестр,
индекс и партитуры — вне git (ADR-0155 В1 «(в)»).

## Вход

| что | откуда | отпечаток |
|---|---|---|
| RTTTL-библиотека | архив пакета `rob_box_mcp_tools/data/rtttl_melodies.jsonl.gz` (не `voice_memory.db` робота) | md5 `ee98c503df0000246b8cbc687272a6d2` |
| индекс партитур пака | копия `ros2@10.1.1.21:/opt/rob_box/scores/score_index.db` (scp 08.10, только чтение), 138 627 строк | md5 `3457760398d532798e26f78a76ff15c2` |
| код | `origin/develop` @ `6450f1000` + эта ветка | — |

Команда (Windows, пакеты worktree первыми в `sys.path`):

```
python scripts/music/works_pdmx.py --db registry.sqlite --scores score_index.db
```

Время связывания 70.8 с (ноутбук, Python 3.11). Почти всё время уходит на нечёткий уровень (difflib).

## Правила уровней (`works.match_scores`)

- `exact_artist`: нормализованное название совпало (целиком, до скобок или до « - автор»), и автор партитуры
  (`composer` или хвост после « - ») пересёкся с исполнителем/композитором произведения. Партитура с таким
  совпадением не предлагается одноимённым произведениям других исполнителей («Dreams» Corrs ↔ Cranberries).
- `exact`: совпало только название (целиком или до скобок). « - хвост» без совпавшего автора не считается.
- `fuzzy`: вложение наборов слов (≥ 2 слов, разница ≤ 2) или difflib ≥ 0.88. Только если точной пары нет,
  ≤ 5 на произведение, вес — половина сходства.
- Не связываются: названия-заглушки («Theme», «Unknown», …), названия короче 3 букв, запись, названная именем
  своего автора («Mozart» — Mozart), начало до « - » из одного имени автора партитуры («Mozart - Concerto K. 191»).
- У всех связей с партитурами `confirmed=0` (В6: подтверждает только человек через `link_level=manual`).
- Партитура с непригодной лицензией (`material.license_usable`: пусто, `unknown`, `…conflict`) не связывается.

Точность уровней в этой работе **не мерилась**. Оценка — из замера K-1 (§2.2 ADR: 0.75 / 0.35 / 0.17 на 60 парах).
Я просмотрел по 20 случайных пар каждого уровня (только названия, не ноты), счёт не вёл.

## Сырой вывод

```
записей RTTTL 10461 → произведений 9487; партитур в индексе 138627, с непригодной лицензией (не связываются) 0; связей 5487 за 70.8 с
произведений 9487; с партитурой (любой уровень) 2351
уровень       произведений  связей  партитур  подтверждено
exact                 1009    2485      1664             0
exact_artist           187     381       360             0
fuzzy                 1239    2621      1577             0
лицензии связанных партитур: cc-zero 2108, publicdomain 1183
непригодных лицензий (license_conflict, unknown, пусто) среди связанных: 0 (M6 ADR-0154: 0)
связей со стоп-списком живых правообладателей (knowledge.LICENSE_STOP_LIST): 3
«Григ» строки ['mountain', 'king']
  прямой поиск партитур: 14; лицензии: cc-zero 13, publicdomain 1
  произведений реестра 5, из них с партитурой 2; партитур через произведения 8 (exact_artist 8); лицензии: cc-zero 8; в прямом поиске из них 8
  лучшие через произведения: pdmx:QmXXdTmXQW5jKbHEHgHuZFuW57wNrtApiba8WidW76APCp [exact_artist, cc-zero]; pdmx:QmRTWPHsmohLA5st7zMzdmeth38RCyytHroPkA2pCDLSCA [exact_artist, cc-zero]; pdmx:QmNjgUgefNAtAy4FnYbqAZsyQuMN1Wf45RkdMqgbjsBVjr [exact_artist, cc-zero]
«Моцарт» строки ['mozart', 'motsart']
  прямой поиск партитур: 442; лицензии: cc-zero 289, publicdomain 153
  произведений реестра 9, из них с партитурой 6; партитур через произведения 13 (exact 2, exact_artist 4, fuzzy 7); лицензии: cc-zero 7, publicdomain 6; в прямом поиске из них 10
  лучшие через произведения: pdmx:QmeMbPwtPPiSG2atikqZG4vCxPyuuR9bWGmjt5J9rpmv4W [exact_artist, cc-zero]; pdmx:QmU6tKSpTsdKCbaxAdygGtYuLZEjSgenCs8cn8247Pe7x4 [fuzzy, cc-zero]; pdmx:QmYuG1LqvMLVh84vWK6xP3AXtgSZ5Bunbnr3x6XGERbz3M [exact_artist, publicdomain]
«Бах» строки ['bach']
  прямой поиск партитур: 1386; лицензии: cc-zero 1151, publicdomain 235
  произведений реестра 20, из них с партитурой 14; партитур через произведения 54 (exact 28, exact_artist 14, fuzzy 12); лицензии: cc-zero 30, publicdomain 24; в прямом поиске из них 23
  лучшие через произведения: pdmx:QmbDeMwmoBkvBVLeT5cmWPCW29fJaNw1PfGybA886dfq5P [fuzzy, publicdomain]; pdmx:QmUYYW8Knxf2ncUGxrqTZ4vVCfE4g4GMMxXie4ZjcH8ZN2 [fuzzy, cc-zero]; pdmx:QmTJYYLf7PaqwbZ4h3tQ4eDnGnX9gG1SBDaKjiixKp2gD2 [fuzzy, cc-zero]
«Чайковский» строки ['tchaikovsky', 'chaykovskiy']
  прямой поиск партитур: 105; лицензии: cc-zero 87, publicdomain 18
  произведений реестра 2, из них с партитурой 1; партитур через произведения 2 (exact_artist 2); лицензии: publicdomain 1, cc-zero 1; в прямом поиске из них 2
  лучшие через произведения: pdmx:QmXw1JouGSKb7wd1ypVYQJJn15VXr3gUVHuYQ5qcZt7ne7 [exact_artist, publicdomain]; pdmx:QmQfVXYootWQdKrgkuP3PJVqzzivfqZxquQWjAuJodQput [exact_artist, cc-zero]
«Марио» строки ['mario']
  прямой поиск партитур: 96; лицензии: cc-zero 63, publicdomain 33
  произведений реестра 21, из них с партитурой 7; партитур через произведения 13 (exact 2, exact_artist 1, fuzzy 10); лицензии: cc-zero 12, publicdomain 1; в прямом поиске из них 12
  лучшие через произведения: pdmx:QmaYAdDvuyGi2fKyHX8dE1nxj4EuXGv6y2GDjccjzefRyK [fuzzy, cc-zero]; pdmx:QmZ9jepaqd6G8zrCppnZCb66kAhe7CwRemY1dFDQ7j5xGe [exact, cc-zero]; pdmx:QmNVP2TMnnV6ifukGM6hjrNCmuMjK1qQkZD4uvcHRmwBaX [fuzzy, cc-zero]
«Тетрис» строки ['tetris']
  прямой поиск партитур: 24; лицензии: cc-zero 20, publicdomain 4
  произведений реестра 2, из них с партитурой 1; партитур через произведения 5 (exact 5); лицензии: publicdomain 3, cc-zero 2; в прямом поиске из них 5
  лучшие через произведения: pdmx:QmUq4yHnR5TQ8D2MwV2WtBFVHBbGSXQgU1ByVuPTxQ2e8x [exact, publicdomain]; pdmx:QmPfPaKdUBn3nNuTozbqF9n9YUhqmAShhFBZczy48jtPM5 [exact, publicdomain]; pdmx:QmPuhnSDgxALaJCZknRqxudNYmtWSpLUaugSFwVzkAFDnY [exact, cc-zero]
«Гарри Поттер» строки ['garri', 'potter']
  прямой поиск партитур: 0; лицензии: —
  произведений реестра 0, из них с партитурой 0; партитур через произведения 0 (—); лицензии: —; в прямом поиске из них 0
  лучшие через произведения: —
«Интерстеллар» строки ['space', 'interstellar']
  прямой поиск партитур: 43; лицензии: cc-zero 26, publicdomain 15, private-local (Shifu only, not PD) 2
  произведений реестра 24, из них с партитурой 7; партитур через произведения 9 (exact 1, fuzzy 8); лицензии: publicdomain 5, cc-zero 4; в прямом поиске из них 4
  лучшие через произведения: pdmx:QmPyFhZNgT9JzjCk6BXfCBpwcUvdGgaJBZfQhLVoAP5zHi [fuzzy, cc-zero]; pdmx:QmWsNFdtb86iWXTkAmWHZS6vNsxjZaQMiUw9Ebrc3Zd6jh [fuzzy, cc-zero]; pdmx:QmTsYDZ8k3afZAc9FyJ97FG7EBqQcZKYSCLPE4cNNsYXvL [exact, publicdomain]
«кино» строки ['kino', 'жанр:movie']
  прямой поиск партитур: 2528; лицензии: cc-zero 2230, publicdomain 298
  произведений реестра 146, из них с партитурой 55; партитур через произведения 39 (exact 14, exact_artist 2, fuzzy 23); лицензии: cc-zero 30, publicdomain 9; в прямом поиске из них 19
  лучшие через произведения: pdmx:QmPyFhZNgT9JzjCk6BXfCBpwcUvdGgaJBZfQhLVoAP5zHi [fuzzy, cc-zero]; pdmx:QmeAx52MBc5ea57mgxZAgo5TAC2fg2wQswfiC3UNZhHC2w [fuzzy, cc-zero]; pdmx:QmPfmdKYFpxwP5siMJ4wMWd3UPMyMfqJeSGGRxxUxChArn [fuzzy, cc-zero]
«Щелкунчик» строки ['schelkunchik']
  прямой поиск партитур: 0; лицензии: —
  произведений реестра 0, из них с партитурой 0; партитур через произведения 0 (—); лицензии: —; в прямом поиске из них 0
  лучшие через произведения: —
«Зельда» строки ['zelda']
  прямой поиск партитур: 28; лицензии: cc-zero 26, publicdomain 2
  произведений реестра 12, из них с партитурой 5; партитур через произведения 9 (exact 3, fuzzy 6); лицензии: cc-zero 7, publicdomain 2; в прямом поиске из них 6
  лучшие через произведения: pdmx:QmPgorkBfJoTnXorYB6XpPXgc1oxXfgsEw29gYRjdKkCpp [fuzzy, cc-zero]; pdmx:QmPBPPqB2EFP3gqWFo7ZrEMMRs2BRUqLMHoZN22udnshm3 [exact, cc-zero]; pdmx:QmVCrjjsVR6hdVQyeXzJdkDzLTW6kGrxmQXzyYPw6R9zx9 [exact, publicdomain]
«Звёздные войны» строки ['star', 'wars']
  прямой поиск партитур: 5; лицензии: cc-zero 5
  произведений реестра 11, из них с партитурой 4; партитур через произведения 2 (fuzzy 2); лицензии: cc-zero 2; в прямом поиске из них 2
  лучшие через произведения: pdmx:QmRmXQhpQ2qesBm9oFVP58yuByZcZRVfvPYAbHRAYbLvsZ [fuzzy, cc-zero]; pdmx:QmUSgePajGmLpiLY9Za1bHCCduEqBTMY7o4M9Yj7mKf3w9 [fuzzy, cc-zero]
```

«Прямой поиск» — `ScoreIndex.search` по строкам `search.part_query`, как ищет сет на роботе. «Через произведения» —
тот же `ScoreIndex` по произведениям реестра (название, исполнитель + композитор, жанр каталога) и их связи
`work_sources`.

## Что из этого следует

- С партитурой связано 2 351 произведение из 9 487 (24.8 %): `exact_artist` у 187, `exact` у 1 009, `fuzzy` у
  1 239 (у одного произведения могут быть связи обоих точных уровней). Подтверждённых — 0.
- Непригодных лицензий среди связанных — 0 (M6). В индексе пака их нет вообще (0 из 138 627): импортёр уже их
  отфильтровал. Две строки пака, `local:baff7d71` и `local:a8c4e243`, с лицензией
  `private-local (Shifu only, not PD)` `license_usable` пропускает. В связи через произведения они не попали:
  RTTTL-записи с названием Interstellar нет.
- Стоп-список: 3 нечёткие связи «It's A Small World [Disney]» (`disney` в названии записи RTTTL).
- Через произведения находится **меньше** партитур, чем прямым поиском по индексу пака. Реестр сегодня состоит из
  RTTTL-записей. У партитуры без RTTTL-пары своего `Work` нет: Interstellar Main Theme, Щелкунчик, большая часть
  Моцарта и Баха.
- «Гарри Поттер» и «Щелкунчик» не находятся ни одним путём: в семенах для них нет строк поиска, и слово ищется
  транслитом (`garri`, `schelkunchik`).
- У «Death Music» (Super Mario Brothers) и подобных однословных названий нечётких связей нет: на одно слово
  нечёткий уровень не срабатывает.
