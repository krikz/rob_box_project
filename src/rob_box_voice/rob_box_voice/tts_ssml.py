"""
Конверт SSML для Silero (ADR-0160 §2.3, #4239).

На проводе голый текст реплики. Silero — единственный провайдер, которому нужен SSML:
``apply_tts(ssml_text=…)`` разбирает XML, и сырой ``&`` или ``<`` в тексте его роняет.
Здесь текст обрамляется в ``<speak>`` с экранированием.

Цепочка просодии из SSML (rate/pitch/volume → ``Hints`` Yandex, уровни pitch Silero) удалена в #4240:
с провода она больше не приходит, тон и скорость задают ROS-параметры провайдера.

Без ROS/grpc/numpy — тестируется отдельно, как соседний ``tts_chunking.py``.
"""

#: XML-экранирование текстового узла SSML: ``&`` первым, иначе съест entity.
_XML_ESCAPE_TABLE = str.maketrans({"&": "&amp;", "<": "&lt;", ">": "&gt;"})


def silero_ssml(text: str) -> str:
    """Конверт Silero: голый текст реплики в ``<speak><prosody pitch="medium">`` с экранированием ``&``, ``<``, ``>``.

    Единственное место, где текст реплики обрамляется в SSML (ADR-0160 §2.3, глоссарий «Реплика»).

    >>> silero_ssml("x<y & z")
    '<speak><prosody pitch="medium">x&lt;y &amp; z</prosody></speak>'
    """
    return f'<speak><prosody pitch="medium">{(text or "").translate(_XML_ESCAPE_TABLE)}</prosody></speak>'
