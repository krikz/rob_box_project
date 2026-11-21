# sound_pack/sound_answer — заглушки режима «отвечай звуками»

Пять коротких файлов (`bird.wav`, `seal.wav`, `elephant.wav`, `alligator.wav`,
`growl.wav`) — **заглушки для приёмки** режима ADR-0175, не настоящие голоса существ.

Их собирает `scripts/dev/gen_sound_answer_clips.py` (синус, свист, щелчки, пила;
numpy + stdlib `wave`, без скачивания). Формат: 16 кГц, моно, PCM16, короче секунды.
Имена совпадают с `src/rob_box_voice/config/sound_answer_manifest.json`.

Настоящие звуки кладутся сюда под теми же именами. Код, манифест и параметр
`sound_answer_clip_dir` для этого менять не нужно.
