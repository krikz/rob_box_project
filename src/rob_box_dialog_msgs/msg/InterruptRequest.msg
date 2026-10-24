# Запрос на перебивание речи (ADR-0150 §6.2, PR-5 #3694). Заменяет SpeechCancel от stt_node на любой вейк.
# Пишет stt_node на каждый новый сегмент фразы с вейком (partial и final), решает диалог:
# ясная стоп-команда (rob_box_dialog.words.stop_verdict) - SpeechCancel по эпохе, иначе речь продолжается.

# Детерминированный id фразы по PCM (тот же, что в Utterance); "" - неизвестен.
string utterance_id
# Текст сегмента, в котором услышан вейк (partial или final).
string text_partial
# Направление на говорящего (DOA ReSpeaker) во время фразы, градусы; NaN - неизвестно.
float32 doa_deg
# Время публикации.
builtin_interfaces/Time stamp
