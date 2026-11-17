"""Связка хода ядра диалога без ROS (ADR-0167 Ш1г, решение Шифу 09.11 по В3: отдельный пакет).

Мост команд LLM (``command_bridge``), порты диалога и сборка цепочки LLM (``dialog_v2_ports``), разбор команд движения
(``command_parser``, ``command_intent``), память голоса и собеседников (``voice_memory``, ``speaker_memory``,
``faq_store``), состояние плеера (``music_player_state``), фразы медиа (``media_phrases``), каталог голосов
(``tts_voice_registry``). Раньше жили в ``rob_box_voice``; хосты робота (``dialog_node``, ТАРС) и колонка берут их
отсюда. Модули без ROS на верхнем уровне - это сверяет ``rob_box_dialog/test/test_core_package_4245.py``.
"""
