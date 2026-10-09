# Слушающие судьи (audio-LLM) и быстрый классификатор выбора (Jev и альтернативы)

> Исследование для ADR-0157 (виток 3 аранжировщика). Выполнено воркером 09.10.2026 по веб-источникам и локальным обзорам `docs/research/jev/`; **ничего из перечисленного не запускалось и не оплачивалось** — все цифры с чужих страниц и статей, ссылки и даты указаны. Там, где официальных данных нет, так и написано. Перед постановкой в пайплайн каждую цифру мерить у себя.

## 0. Локальные обзоры (docs/research/jev/)

### awesome-jev-survey.md (конспект)
1. Обзор `cobanov/awesome-jev` (commit bb350810, 25.09.2026, 155 проектов) + чтение исходников SDK v0.7.2, jev-engineering, jev-guard, embodied-jev. Платных вызовов Jev не было.
2. Общий паттерн всего списка: **код генерирует кандидатов → Jev выбирает → код исполняет**; hard-rules всегда до модели, модель может только ужесточить.
3. Fail-open/fail-closed зависит от класса решения; shadow-mode перед включением; явная ветка «Uncertain/abstain».
4. Примитивы Choice / Score / Noul; у Noul нет confidence; Choice **не всегда argmax** своего распределения.
5. Антипаттерны с evidence: schema-valid ≠ правильно; высокая confidence ≠ точность (jev-guard: самый уверенный бакет — самый неточный); калибровка на «кубиках» провальна; authority-инъекция через state пробивает до 3/30.
6. Скоры плавают между прогонами и версиями → пинуется `jev-1.13.0`.
7. Независимая латентность: p50 272–371 мс, p95 533–830 мс, холодный старт ≈2.6 с; вендорские «70–500 мс» не подтверждаются 2 из 3 замеров.
8. Робототехника на Jev — только симуляции; модель проигрывает правилам (RoboJEV 5/10 vs 8/10).
9. Hosted Jev — только текст; бюджет 64k на запрос / 32k на state+вопрос.
10. **Музыкальных/аранжировочных проектов в самом обзоре не упомянуто** (есть только «rerank», «games», «robotics»). Jevthoven в обзор не попал — см. §C.

### laya-survey.md (конспект)
1. Laya (convaiinnovations) — self-hosted аналог Jev: энкодер ModernBERT/mmBERT + decision head; веса Apache-2.0.
2. Варианты: `laya:en` 421M/512 ток, `laya:multilingual` 322M/1024 ток, `laya:typed-decisions` 421M.
3. Те же примитивы choice/score/noul; лимиты 1–256 вопросов, 2–255 вариантов.
4. Раздаётся через Ollaya (Rust, `POST /v1/systemone`, порт 11435) — TypeSafe-совместимый API, клиент тот же HTTP.
5. CPU-only fp32 через ONNX; arm64-бинарь и multi-arch Docker есть; INT8/GGUF нет; glibc ≥ 2.38 (в контейнере Humble не запустится — нужен отдельный контейнер).
6. GPU-латентность 8–40 мс (RTX 4090/T4), CPU — «сотни мс», **на Pi 5 не замерено**.
7. Качество: typed-decisions 0.766 vs Jev 0.727; >20 вариантов — слабое место (Banking77 0.425 vs 0.870 у Jev).
8. Сырые чекпоинты переуверены, температуры не подогнаны; для русского калибровки нет → запрещена для safety.
9. Вердикт по хостам: Main Pi — технически возможно, не рекомендуется; Vision Pi — с риском; katana — не опора.
10. Музыкальных применений не упоминается.

## A. Audio-LLM API для критики трёхминутного трека

### Факты по провайдерам

**Google Gemini** ([docs/audio](https://ai.google.dev/gemini-api/docs/audio), обновлено 23.09.2026; [pricing](https://ai.google.dev/gemini-api/docs/pricing)): WAV/MP3/FLAC/OGG/AAC и др.; **до 9.5 ч аудио в промпте**; **32 токена/с** (1920 ток/мин → 3-минутный трек ≈ 5.8k токенов); аудио даунсемплится до 16 кбит/с и сводится в моно; инлайн до 20 МБ, больше — Files API; **таймстемпы «MM:SS»** в промпте поддерживаются официально; **structured output (JSON-схема)** — да. Цены за 1M входных токенов: 2.5 Flash $1.00, 2.5 Flash-Lite $0.30, 2.5 Pro $1.25, 3.8 Flash $0.75 (до 31.12.2026, потом $1.50), 3.1 Pro Preview $2.00; у всех, кроме 3.1 Pro, есть бесплатный тир на аудио. → **3-минутный трек ≈ $0.004–0.012 за запрос** вместе с ответом. 3.8 Flash: контекст 1 048 576 ток, structured output — да ([model page](https://ai.google.dev/gemini-api/docs/models/gemini-3.8-flash), 02.09.2026).
Предупреждение: на форуме Google — отчёты о **дрейфе таймстемпов** у Gemini 3 Flash / 3.1 Pro (до −16 с к концу 12-минутного файла; «с песнями выдаёт бессмыслицу»), при этом 2.5 Pro у тех же авторов точен ([thread 129501](https://discuss.ai.google.dev/t/bug-gemini-3-flash-and-3-1-pro-progressive-timestamp-drift-in-audio-transcription/129501), [thread 110751](https://discuss.ai.google.dev/t/it-seems-gemini-3-has-lost-its-audio-data-interpretation-capabilities-especially-regarding-time/110751)). Это пользовательские отчёты, не контролируемый тест.

**OpenAI** ([pricing](https://developers.openai.com/api/docs/pricing), [gpt-audio-1.5](https://developers.openai.com/api/docs/models/gpt-audio-1.5), [gpt-4o-audio-preview](https://developers.openai.com/api/docs/models/gpt-4o-audio-preview)): аудио-вход через Chat Completions у `gpt-audio-1.5` ($32/M аудио-токенов, текст-выход $10/M, контекст 128k), `gpt-audio-mini` ($10/M), `gpt-4o-audio-preview` ($40/M, снапшот 2025-06-03). Лимит длительности **не указан** в доках; формат в примерах — WAV. Токенов на минуту официально не опубликовано; по старому прайсу «$0.06/мин при $100/M» выходит ~600 ток/мин (≈10 ток/с) — **расчёт, не источник** ([Simon Willison, 18.10.2024](https://simonwillison.net/2024/Oct/18/openai-audio)). → 3-минутный трек ≈ **$0.02–0.07**. Музыку в доках не упоминают; в MMAU-Pro GPT-4o-Audio набрал 63.1 на music, но в PitchBench — худший (8.4 %).

**Alibaba Qwen** ([Qwen3-Omni GitHub](https://github.com/QwenLM/Qwen3-Omni), релиз 22.09.2025; [Model Studio omni docs](https://www.alibabacloud.com/help/en/model-studio/qwen-omni), обновлено 08.10.2026): через DashScope `qwen3-omni-flash` принимает **до 20 мин аудио / 100 МБ** (MP3/WAV/AAC…), `qwen3.5-omni` / `qwen3.8-omni-flash` — до 3 ч / 2 ГБ. **Цены официальная страница не показывает** («смотрите в консоли»); третьи стороны: ~12.5 ток/с аудио, `qwen3.8-omni-flash` $0.15/M вход ([LiteLLM](https://docs.litellm.ai/blog/qwen3_8_omni_flash)), Qwen3.5-Omni бесплатен в превью ([tokencost](https://tokencost.app/blog/qwen3-5-omni-pricing-benchmarks)) — **не подтверждено официально**. На OpenRouter страницы omni-моделей не нашёл (404); Qwen3-Omni-30B-A3B есть у Novita $0.25/$0.97. Музыкальные бенчи Qwen3-Omni: RUL-MuchoMusic 52.0 (предыдущий open-SOTA AF3 47.6), GTZAN 93.0, MMAU 77.5. Open-weights Apache-2.0, но BF16 требует 69–145 ГБ VRAM по таблице репо — **на одной RTX без квантизации не пойдёт**.

**Open-weights на рабочей станции**:
- *Audio Flamingo 3* ([HF](https://huggingface.co/nvidia/audio-flamingo-3), 10.07.2025; [arXiv 2507.08128](https://arxiv.org/abs/2507.08128)): бэкбон Qwen2.5-7B, до 10 мин (окна 30 с), тестировали на A100 80GB; **лицензия NVIDIA OneWay Noncommercial** — коммерческое использование запрещено. MMAU-Pro music 61.7.
- *Music Flamingo* ([HF](https://huggingface.co/nvidia/music-flamingo-hf), статья 13.11.2025): 8B, та же non-commercial лицензия; промпты про key/tempo/chords/structure; оценку качества не заявляет. В PitchBench «Audio Flamingo Next» — 15.4 %.
- *Qwen2-Audio / SALMONN / MU-LLaMA* — поколение 2023–24; на MuChoMusic (ISMIR 2024, [arXiv 2408.01337](https://arxiv.org/html/2408.01337)) Qwen-Audio 51.4 %, SALMONN 41.8 %, MU-LLaMA 32.4 % при случайном 25 %, и большинство **не теряет точности при замене аудио на белый шум** — опираются на текст.

### Бенчмарки и честная оценка слабостей

| Бенчмарк (дата) | Что меряет | Результат |
|---|---|---|
| MMAU-Pro ([arXiv 2508.13992](https://arxiv.org/html/2508.13992), 19.08.2025) | music-домен, MCQ | Human 70.5; Gemini 2.5 Flash **64.9**; GPT-4o-Audio 63.1; AF3 61.7; Qwen2.5-Omni-7B 61.5; random 26.1. Слабейшие навыки — Temporal Event Reasoning и Quantitative Reasoning |
| MMAU leaderboard (via [Covo-Audio](https://arxiv.org/pdf/2602.09823)) | music subset | Gemini 2.5 Pro 68.26 (test-mini) / 64.77 (test) |
| **PitchBench** ([arXiv 2605.26176](https://arxiv.org/html/2605.26176v1), 25.05.2026) | слух на высоту: абс./отн., полифония, детюн | Qwen-3.5 Omni Plus 47.7 %, Qwen-3.5 Omni Flash 34.2 %, **Gemini 3.1 Pro 17.8 %**, AF Next 15.4 %, Gemini 3 Flash 14.0 %, **GPT-4o audio 8.4 %**; полифонические линии — **0 % у всех**; вывод авторов: «Current ALMs do not yet possess stable pitch perception» |
| Factual Music Comprehension ([arXiv 2511.05550 v3](https://arxiv.org/html/2511.05550), 07.08.2026) | метр, инструменты, жанр | **Размер такта: ни одна модель (Qwen3-Omni, AF3, Music-Flamingo, Gemini 3) не отличает правильное аудио от случайного** — все отвечают 4/4 |
| MuChoMusic (ISMIR 2024) | знание/рассуждение о музыке | см. выше; текстовая предвзятость |
| SongEval ([arXiv 2505.10793](https://arxiv.org/pdf/2505.10793), 2025; [toolkit](https://github.com/ASLP-lab/SongEval)) | 5 эстетических осей песен с вокалом | не про высоту/ритм; для инструментального робота малополезна |
| MuseCritic ([arXiv 2608.11755](https://arxiv.org/abs/2608.11755), 12.08.2026, v2 01.10.2026) | обученный критик + текстовые критики | LCC 0.907 с человеком на SongEval; Gemini-3.1-Pro используют как judge-baseline; таймстемпы в критике не заявлены |
| MusicEval ([arXiv 2501.10811](https://arxiv.org/abs/2501.10811), 2025) | 2748 клипов, 14 экспертов | скорер на CLAP, не LLM-judge |

**Вывод по A (честно):** ни одна audio-LLM в октябре 2026 не умеет надёжно слышать фальшивую ноту или сбитый размер. Лучшая по высоте — Qwen-3.5 Omni Plus (47.7 %, и та «badly hurt by detuning»); Gemini и GPT в open-ended формате около 8–18 %. В MCQ-формате (выбор из вариантов) точность вырастает на 24–39 п.п. — это аргумент за **узкие вопросы с перечислением**, а не «напиши критику». Таймстемпы Gemini годятся для грубой привязки («в районе 1:20 бас заглушает лид»), но не для точного «нота на 1:23.4». Поэтому judge-LLM стоит использовать как **обёртку над объективными метриками (раздел B) и известной партитурой**: передавать модели не только аудио, но и извлечённые кодом факты (ключ, сетка битов, список нот вне лада с секундами) и просить резюме/приоритизацию — это ровно принцип ADR-0148.

### Сравнительная таблица A

| Модель / API | Макс. аудио | Токены/с | Цена за 3-мин трек (вход) | Таймстемпы | JSON-схема | Музыкальная сила | Слабости |
|---|---|---|---|---|---|---|---|
| Gemini 2.5 Flash | 9.5 ч | 32 | ≈$0.006 | MM:SS офиц. | да | MMAU-Pro music 64.9 (лучшая в табл.) | pitch n/a в PitchBench; 2.5-серия — legacy |
| Gemini 2.5 Pro | 9.5 ч | 32 | ≈$0.007 | да, по отчётам точнее 3.x | да | MMAU music 68.3 | дорогой выход $10/M |
| Gemini 3.8 Flash | 9.5 ч | 32 | ≈$0.004 | да, но отчёты о дрейфе у 3.x | да | не измерена на music | PitchBench 3 Flash 14 % |
| Gemini 3.1 Pro Preview | 9.5 ч | 32 | ≈$0.012 | да, дрейф по отчётам | да | judge-baseline в MuseCritic | PitchBench 17.8 % |
| OpenAI gpt-audio-1.5 | не указан (128k ctx) | ~10 (расчёт) | ≈$0.06 | нет офиц. механизма | да (JSON mode) | — | GPT-4o audio: PitchBench 8.4 % |
| OpenAI gpt-audio-mini | не указан | ~10 | ≈$0.02 | нет | да | MMAU-Pro music 59.7 (4o-mini) | как выше |
| Qwen3-Omni-Flash (DashScope) | 20 мин / 100 МБ | ~12.5 (неофиц.) | цена в консоли; ~$0.001 по 3-м сторонам | не заявлены | через промпт | RUL-MuchoMusic 52, GTZAN 93 | лимиты/цены не на странице |
| Qwen3.5-Omni Plus (DashScope) | 3 ч | ? | превью бесплатно (неофиц.) | ? | ? | **лучшая в PitchBench 47.7 %** | ломается на детюне; OpenRouter нет |
| Audio Flamingo 3 (local) | 10 мин | — | $0 + A100-класс | нет | нет | MMAU-Pro music 61.7 | **non-commercial**, VRAM не указана |
| Music Flamingo 8B (local) | 10–20 мин | — | $0 + A100-класс | нет | нет | key/tempo/chords в промптах | non-commercial |
| Qwen3-Omni-30B-A3B (local) | — | — | 69–145 ГБ BF16 | нет | нет | RUL-MuchoMusic 52 | не влезает в RTX без квантизации |

## B. Объективные локальные метрики (для регрессии в CI)

- **Audiobox Aesthetics** ([GitHub](https://github.com/facebookresearch/audiobox-aesthetics), [arXiv 2502.05139](https://arxiv.org/html/2502.05139v1), 07.02.2025): `pip install audiobox_aesthetics`, CC-BY 4.0, 16 кГц моно, **окна 10 с**; выдаёт CE/CU/PC/PQ. Корреляция с человеческим OVL на музыке: CE 0.528, PQ 0.464, CU 0.465, PC 0.251 — **ниже, чем у PAM (0.581)**. Пригодна как регрессионная метрика «не стало ли хуже» (PQ — грязь микса, CE — общая приятность), не как детектор фальши.
- **PAM** ([GitHub](https://github.com/soham97/PAM), MIT): no-reference, промптит CLAP антонимными парами, `python run.py --folder`. Корреляция с музыкой 0.581. Быстрая, лёгкая.
- **FAD via fadtk** ([GitHub](https://github.com/microsoft/fadtk), MIT): CLAP (MS/LAION), VGGish, MERT, Encodec…; `--indiv` даёт **per-song FAD** (поиск выбросов), `--inf` — FAD∞. Нужен эталонный набор (например, наши «хорошие» записи робота); сравнивать после ресемпла в 16k (звук робота — 16 кГц ReSpeaker).
- **SongEval toolkit** — для песен с вокалом, лицензия противоречива; для инструментального аранжировщика не подходит.
- **Essentia** ([KeyExtractor](https://essentia.upf.edu/reference/std_KeyExtractor.html), [Dissonance](https://essentia.upf.edu/reference/std_Dissonance.html)): ключ/лад/strength, диссонанс Plomp–Levelt; **лицензия AGPL** — ок для внутреннего CI. Поскольку аранжировщик знает заданный лад, честнее считать «ноты вне лада» pitch-трекером против заданного лада в центах — это объективный детектор фальши, которого LLM не даёт. (У нас тема и гармония известны из модели трека — проверка по модели ещё точнее, чем по аудио.)
- **Бит-трекинг**: madmom на PyPI 0.16.1 от 2017; **beat_this** (ISMIR 2024, `pip install beat-this`, torch ≥ 2.0, CPU-fallback, [репо](https://github.com/CPJKU/beat_this)) — актуальная замена; BeatNet — для онлайна/размера. Регрессия: стабильность межбитового интервала, доля пропущенных битов = «сломанный грув».

Для CI реалистичен набор: PAM + Audiobox (PQ/CE) как «гладкие» скоры с порогом на регресс, FAD-per-song против эталонной выборки, Essentia-ключ + доля нот вне заданного лада, beat_this-джиттер. Всё запускается локально на katana (GPU) или на CPU медленнее.

## C. TypeSafe Jev и альтернативы для структурного выбора

**Состояние Jev (09.10.2026):**
- Доступность: `docs.typesafe.ai` **открывается** ([/models](https://docs.typesafe.ai/models)); `api.typesafe.ai/` отвечает 404 на корень (хост жив); `typesafe.ai/pricing` — 404, цена на [главной](https://typesafe.ai/) (08.10.2026): **$42 за Btok = $0.042/Mtok входа, выход бесплатно**; статус «early access», консоль console.typesafe.ai.
- Версия: единственная `jev-1.13.0`, алиасы `jev-latest`/`jev-preview` на неё; лимиты 64k на запрос, 32k state+вопрос; rate-limit 100K ток/с и 80 rps; **латентность в доках не заявлена**.
- Независимая латентность — только из локального обзора (p50 272–371, p95 533–830 мс, холодный старт 2.6 с); свежих замеров 2026 не нашёл.
- **Музыка на Jev**: есть **Jevthoven** ([cocktailpeanut/jevthoven](https://github.com/cocktailpeanut/jevthoven), MIT, 15 звёзд, 7 коммитов) — prompt-to-MIDI студия: Jev отвечает Choice на план (размер/темп/форма), инструмент, гармонию, грув, паттерн каждого такта; код превращает выбор в ноты, Tone.js играет, экспорт MIDI. **Только символьно, аудио не слушает, оценки качества нет.** Это ближайший аналог «Jev выбирает среди кандидатов аранжировщика». Ещё **Prosodia** ([alperiox/audio-jevlike](https://github.com/alperiox/audio-jevlike)) — экспериментальная audio-native Jev-подобная модель на речевых эмоциях, музыки нет. Для ранжирования — `jev-reranker`, `jev-rerank-bench`. Официальных музыкальных демо у TypeSafe нет.

**Альтернативы:**
- *Constrained decoding*: **Outlines** ([GitHub](https://github.com/dottxt-ai/outlines), Apache-2.0); **llguidance** ([GitHub](https://github.com/guidance-ai/llguidance), MIT; встроен в vLLM ≥ 0.8.2, llama.cpp, SGLang). Важно: **StructureBench (IJCAI 2026)** — ограничение гарантирует синтаксис, но **не повышает семантическую точность и может ухудшать малые модели** ([papers.cool](https://papers.cool/venue/267@2026@IJCAI)).
- *Cross-encoder рерэнкеры*: **bge-reranker-v2-m3** ([HF](https://huggingface.co/BAAI/bge-reranker-v2-m3), 0.6B, Apache-2.0, multilingual); **jina-reranker-v3** ([jina.ai](https://jina.ai/models/jina-reranker-v3/), 597M, **CC-BY-NC-4.0**, listwise до 64 документов). Рерэнкеры меряют текстовую релевантность «запрос↔кандидат», а не музыкальную уместность — для «какой лад выбрать» это натяжка; для «какое произведение из каталога соответствует фразе» — уместно.
- *Chat-LLM с enum-схемой*: Claude Haiku 5.5 **$0.10/$0.50 за M**, Sonnet 5.5 $2/$10 ([claude.com/pricing](https://claude.com/pricing)); Gemini 2.5 Flash-Lite $0.30/$0.40.

### Сравнительная таблица C

| Вариант | Цена за выбор (state ~1k ток) | Латентность | Калибровка/вероятности | Хостинг | Лицензия | Риски |
|---|---|---|---|---|---|---|
| Jev 1.13 (hosted) | ≈$0.00004 | p50 0.3–0.4 с, p95 до 0.8 с, холодный 2.6 с (чужие) | да, но «уверенность ≠ точность», не argmax | облако, early access | проприетарно | дрейф версий, 32k, без аудио |
| Laya via Ollaya (self-hosted) | $0 | GPU 8–40 мс; CPU «сотни мс», Pi не мерен | переуверена без подгонки | katana/Pi, отдельный контейнер | Apache-2.0 | >20 вариантов — падает; русский не калиброван |
| Haiku 5.5 + enum-схема | ≈$0.0001–0.0002 | не замерял (обычно сотни мс–1.5 с) | нет (только выбор) | облако | — | соблазн «рассуждать» вместо выбора |
| Gemini 2.5 Flash-Lite + enum | ≈$0.0003 | не замерял | нет | облако | — | как выше |
| Малая instruct-модель + Outlines/llguidance (local) | $0 | десятки мс на GPU | logits есть, семантика не гарантирована | katana GPU / CPU | Apache/MIT | мелкие модели глупее; обслуживать vLLM |
| bge-reranker-v2-m3 (local) | $0 | ~100–200 мс/20 кандидатов (оценка) | скор relevance | CPU/GPU | Apache-2.0 | не про музыку |
| jina-reranker-v3 listwise | $0 локально / API | ~188 мс (железо неизв.) | да | CPU/GPU | CC-BY-NC | лицензия |
| Детерминированный код (правила, история, совпадение по алиасам) | $0 | <1 мс | точность известна | in-process | — | нужно самим кодировать правила |

Trade-off честно: разница в цене между Jev и Haiku-с-enum — микроцента; реальная разница — **вероятности и латентность** у Jev против **понимания музыки и доступности уже сейчас** у Haiku/Gemini. Для вопросов вида «какой из 5 ладов для сцены X» у чат-LLM больше музыкальных знаний; у Jev — калибровка, которую всё равно надо переснять на наших данных (урок jev-guard).

## Что реалистично в этом месяце (с оговорками)

1. **Судья для dev-цикла:** Gemini 2.5 Flash или 2.5 Pro через Files API (бесплатный тир покроет сотни треков; ≈$0.01 за 3 минуты на платном) с JSON-схемой `{issues:[{t:"MM:SS", kind:enum, severity:1-5}], summary}`. Подавать вместе с аудио **факты от кода** (ключ, сетка битов, список нот вне лада). Не доверять таймстемпам точнее ±5–10 с и не доверять «фальшиво/не фальшиво» без подтверждения по модели трека — PitchBench 14–18 % у Gemini. Qwen3.5-Omni Plus лучше по слуху (47.7 %), но цены/лимиты не опубликованы — пробовать как второй судья, не как основной.
2. **Регрессия в CI:** PAM + Audiobox (PQ/CE) + FAD-per-song (fadtk/CLAP) + доля нот вне лада + джиттер beat_this. Всё локально, лицензии ок (AGPL Essentia — только внутри).
3. **Выбор кандидатов:** на этот месяц — **код + Haiku/Gemini-Flash-Lite с enum-схемой** (ключи и инфраструктура есть, ADR-0148: код задаёт кандидатов, модель только выбирает). Jev — в shadow-режиме параллельно, если early access дадут: Jevthoven показывает, что Choice на план/инструмент/такт работает, но ни у кого нет данных о качестве музыки. Laya на Pi — только после замера p95.
4. **Неопределённости:** цены OpenAI/Qwen на аудио-токены в минутах — расчёт, не документ; латентность Jev 2026 — нет свежих замеров; open-weights AF3/Music Flamingo — non-commercial, на RTX не проверено; дрейф таймстемпов Gemini 3.x — форумные отчёты. Каждое из этого надо мерить у нас прежде, чем ставить в пайплайн.
