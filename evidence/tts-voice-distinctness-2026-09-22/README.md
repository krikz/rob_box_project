# Различимость голосов TTS для resemblyzer — замер 22.09.2026

Сырые данные к правке таблицы `map_tts_voice`
(`.github/workflows/scripts/e2e_voice_lib.sh`) и к ADR-0127.

**Инструмент:** `scripts/e2e/measure_tts_voice_distinctness.py`
**Где запускалось:** контейнер `voice-assistant` на Vision Pi (10.1.1.21) —
там живут и resemblyzer, которым робот считает d-vector, и ключи провайдеров.
**Что считается:** две разные фразы на голос → resemblyzer-эмбеддинги →
`intra-voice` (min-cos между фразами одного голоса) и `inter-voice`
(max-cos между голосами, та же max-of-pool семантика, что у
`SpeakerDatabase.identify()`).

Пороги робота: `IDENTIFY_THRESHOLD = 0.72`, `REGISTER_MATCH_THRESHOLD = 0.75`
(`src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py`).

## Файлы

| Файл | Что внутри |
|---|---|
| `minimax_voices.json` | полный каталог MiniMax (8 русских голосов) — таблица ДО правки |
| `silero_voices.json` | полный каталог Silero v5 (5 голосов) |
| `minimax_pitch.json` | эксперимент «развести голоса питчем»: `Russian_ReliableMan` и `Russian_HandsomeChildhoodFriend` при pitch −6 / 0 / +6 |
| `minimax_final.json` | итоговая четвёрка MiniMax из новой таблицы |

## Главные числа

Intra-voice (один и тот же голос, две фразы): **0.95–0.98** у всех
провайдеров. Это «потолок» — так выглядит один человек.

### MiniMax, каталог целиком (`minimax_voices.json`)

Пары каста по СТАРОЙ таблице:

```
0.7386  anton/ermil    (Russian_ReliableMan + Russian_HandsomeChildhoodFriend)  <-- выше 0.72
0.8099  ermil/filipp   (Russian_HandsomeChildhoodFriend + Russian_AttractiveGuy) <-- выше 0.75
0.6173  anton/zahar    0.7103 anton/filipp   0.5186 ermil/zahar   0.5037 zahar/filipp
```

Всего в каталоге 11 пар из 28 выше порога опознания; женский кластер
(`BrightHeroine`/`AmbitiousWoman`/`CrazyQueen`) плотный — до 0.864.

### MiniMax, новая таблица (`minimax_final.json`)

```
anton  → Russian_ReliableMan       ermil  → Russian_PessimisticGirl
zahar  → Russian_CrazyQueen        filipp → Russian_AttractiveGuy

0.5269 anton/ermil   0.5453 anton/zahar   0.6613 anton/filipp
0.6838 ermil/zahar   0.4507 ermil/filipp  0.4577 zahar/filipp
```

Худшая пара 0.684 — ниже порога опознания. Критичная для акта 2 пара
`anton`/`ermil` (Саша и Борис) упала с 0.739 до 0.527.

### Silero (`silero_voices.json`)

```
0.6783 aidar/eugene   0.5127 aidar/baya    0.5601 aidar/xenia   0.6110 aidar/kseniya
0.4801 eugene/baya    0.5467 eugene/xenia  0.5588 eugene/kseniya
0.7217 baya/xenia     0.7881 baya/kseniya  0.6312 xenia/kseniya
```

`baya` — источник обеих плохих пар, поэтому в новой таблице `zahar`
получил `kseniya`, а `baya` остался за `alena` (в касте не участвует).
Худшая пара каста: 0.678.

### Yandex — НЕ ИЗМЕРЕН

```
grpc._channel._MultiThreadedRendezvous: StatusCode.PERMISSION_DENIED
details = "Permission to [resource-manager.folder b1gfmjogjodcgff82pjd, ...] denied"
```

Ключ, лежащий в контейнере робота, к SpeechKit v3 не допущен. Замерить,
когда права починят: `measure_tts_voice_distinctness.py --provider yandex`.

### Питч/темп как альтернатива (`minimax_pitch.json`) — отклонено

Питч реально разводит эмбеддинги, и intra-voice при ФИКСИРОВАННОМ за
говорящим сдвиге не страдает (0.95–0.97):

```
Russian_ReliableMan          vs Russian_HandsomeChildhoodFriend          0.7033
Russian_ReliableMan[p+6]     vs Russian_HandsomeChildhoodFriend          0.4242
Russian_ReliableMan[p-6]     vs Russian_HandsomeChildhoodFriend[p-6]     0.7686  (сдвиг в одну сторону не помогает)
```

Но STT ломается. Тот же файл через vosk (`/models/vosk-model-small-ru-0.22`),
эталон — фраза синтеза, WER по словам:

```
без сдвига:                                    WER 0.05-0.21
Russian_ReliableMan[p+3]                       WER 0.26
Russian_HandsomeChildhoodFriend[p-3]           WER 0.21
Russian_ReliableMan[p+6]                       WER 0.32   «правда от привет давай знакомиться...»
Russian_HandsomeChildhoodFriend[p-6]           WER 0.79   «скотт следует его куй ночную джуниор...»
```

vosk — не тот STT, что стоит в проде (сейчас `minimax`, см.
`/data/stt_provider_state.json`), и small-модель строже к искажениям. Но
направление однозначное, а цена ошибки — подменить диагноз «не различил
дикторов» на «не расслышал». Поэтому чиним таблицей голосов, а не просодией.

## Оговорка про канал

Замер идёт по ФАЙЛАМ синтеза, а робот слышит их через колонку, комнату и
микрофон. Единственная точка сверки: пара `anton`/`ermil` в файлах —
0.739, а на роботе (эмбеддинги из `/data/speakers.db` после акта 2 run
35667281570) — **0.846**. Общий канал записи косинус поднимает, поэтому
числа здесь — оптимистичная оценка: если пара неразличима в файлах, на
роботе будет только хуже.
