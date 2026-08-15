"""
test_agent_loop.py — Unit-тесты агентского цикла DialogueNode.

Покрывает: _continue_after_tool_calls, _execute_tool_calls
Не требует ROS2 — rclpy замокан в conftest.py.
"""

import json
import time
from concurrent.futures import TimeoutError as FuturesTimeoutError
from typing import Any, Dict, List
from unittest.mock import MagicMock, patch, call

import pytest

# conftest.py уже установил mock rclpy, поэтому импорт работает
from rob_box_voice.dialogue_node import DialogueNode
from rob_box_voice.core.conversation_history import ConversationHistory


# ─────────────────────────────────────────────────────────────────────────────
#  Helpers: mock stream chunks
# ─────────────────────────────────────────────────────────────────────────────

def _make_chunk(content=None, tool_calls_delta=None, finish_reason=None, usage=None):
    """Создаёт мок-чанк как у OpenAI streaming API."""
    chunk = MagicMock()
    chunk.choices[0].delta.content = content
    chunk.choices[0].delta.tool_calls = tool_calls_delta
    chunk.choices[0].finish_reason = finish_reason
    if usage is not None:
        chunk.usage = MagicMock(
            prompt_tokens=usage.get('prompt', 100),
            completion_tokens=usage.get('completion', 20),
            total_tokens=usage.get('total', 120),
        )
    else:
        chunk.usage = None
    return chunk


def _plain_text_stream(text: str):
    """Поток с plain-text ответом (без SSML JSON)."""
    return [
        _make_chunk(content=text),
        _make_chunk(finish_reason='stop', usage={'prompt': 100, 'completion': 20, 'total': 120}),
    ]


def _tool_calls_stream(tool_name: str, tool_args: str, tool_id: str = 'tc_1'):
    """Поток с tool_calls ответом (finish_reason='tool_calls')."""
    # Chunk 1: открывает tool call
    tc0 = MagicMock()
    tc0.index = 0
    tc0.id = tool_id
    tc0.function.name = tool_name
    tc0.function.arguments = tool_args

    # Chunk 2: finish
    return [
        _make_chunk(tool_calls_delta=[tc0]),
        _make_chunk(finish_reason='tool_calls', usage={'prompt': 100, 'completion': 10, 'total': 110}),
    ]


# ─────────────────────────────────────────────────────────────────────────────
#  Fixture: минимальная DialogueNode без __init__
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def node():
    """
    Создаёт DialogueNode через object.__new__ (без вызова __init__),
    вручную устанавливая атрибуты необходимые для _continue_after_tool_calls.
    """
    n = object.__new__(DialogueNode)

    # Logger
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger

    # LLM client
    n.client = MagicMock()
    n.model = "test-model"
    n.temperature = 0.7
    n.max_tokens = 500
    n.system_prompt = "<system>test</system>"

    # MCP tools
    n.enable_mcp_tools = False
    n.mcp_tools_available = False
    n.available_tools = []
    n.mcp_adapter = MagicMock()

    # Conversation history
    n.conversation_history = ConversationHistory(max_messages=20)

    # Voice memory (None → не используется)
    n.voice_memory = None

    # Accent replacer
    n.accent_replacer = MagicMock()
    n.accent_replacer.add_accents = lambda text: text

    # Publisher
    n.response_pub = MagicMock()

    # Dialogue state flags
    n.interrupt_agent_loop = False
    n.llm_processing = True
    n.dialogue_in_progress = True
    n._listen_response_waiting = False
    n.current_dialogue_id = "test-dialogue-id"

    # DialogueManager mock
    n.dialogue_manager = MagicMock()
    n.dialogue_manager.last_interaction_time = time.time()

    # _speak_simple (проверяем вызовы)
    n._speak_simple = MagicMock()

    return n


# ─────────────────────────────────────────────────────────────────────────────
#  Тесты управления потоком цикла
# ─────────────────────────────────────────────────────────────────────────────

class TestAgentLoopFlowControl:

    def test_interrupt_at_start(self, node):
        """interrupt_agent_loop=True → цикл сразу выходит, флаги сброшены."""
        node.interrupt_agent_loop = True

        node._continue_after_tool_calls(
            messages=[{"role": "system", "content": "sys"}],
            tool_calls=[],
            tool_results=[],
        )

        assert node.interrupt_agent_loop is False
        assert node.llm_processing is False
        assert node.dialogue_in_progress is False
        node._speak_simple.assert_not_called()

    def test_listen_for_response_stops_loop(self, node):
        """tool_name='listen_for_response' → устанавливает флаг ожидания, не продолжает."""
        tool_results = [
            {'tool_call_id': 'tc0', 'tool_name': 'listen_for_response',
             'success': True, 'message': 'listening'},
        ]
        tool_calls = [{'id': 'tc0', 'type': 'function',
                       'function': {'name': 'listen_for_response', 'arguments': '{}'}}]

        node._continue_after_tool_calls(
            messages=[{"role": "system", "content": "sys"}],
            tool_calls=tool_calls,
            tool_results=tool_results,
        )

        # _listen_response_waiting сбрасывается в finally (это по дизайну — флаг
        # нужен только как guard внутри finally, снаружи он == False)
        assert node._listen_response_waiting is False
        assert node.llm_processing is False
        # dialogue_in_progress остаётся True — ждём ответа пользователя
        assert node.dialogue_in_progress is True

    def test_max_iterations_exceeded(self, node):
        """
        Если LLM бесконечно возвращает tool_calls, цикл останавливается на MAX_ITERATIONS.
        """
        call_count = 0

        def _fake_execute(tool_calls, messages):
            nonlocal call_count
            call_count += 1
            return [{'tool_call_id': 'tc0', 'tool_name': 'play_sound',
                     'success': True, 'message': 'ok'}]

        node._execute_tool_calls = _fake_execute

        # LLM всегда возвращает tool_calls → бесконечный цикл без лимита
        tc_dummy = [{'id': 'tc0', 'type': 'function',
                     'function': {'name': 'play_sound', 'arguments': '{}'}}]

        def _fake_streaming_that_returns_tool_calls(_result=None, **__):
            _result['tool_calls'] = [{'id': 'tc0', 'type': 'function',
                                      'function': {'name': 'play_sound', 'arguments': '{}'}}]
            _result['full_response'] = ''
            _result['chunk_count'] = 0

        with patch.object(node, 'client') as mock_client:
            # Обходим executor: патчим ThreadPoolExecutor.submit
            with patch('rob_box_voice.dialogue_node.ThreadPoolExecutor') as mock_tpe:
                future_mock = MagicMock()
                mock_tpe.return_value.__enter__ = MagicMock()
                mock_tpe.return_value.submit.return_value = future_mock
                mock_tpe.return_value.shutdown = MagicMock()

                def _side_effect(fn):
                    # Симулируем выполнение fn() напрямую (tool_calls)
                    # Меняем recursive_result через closure
                    return future_mock

                # Проще: просто вызовем со ставшим огромным iteration
                # Используем мок _do_recursive_streaming
                original_method = node._continue_after_tool_calls.__func__ if hasattr(node._continue_after_tool_calls, '__func__') else None

        # Прямой тест: установим iteration > MAX_ITERATIONS руками через перехват
        # Используем более простой подход — переопределяем MAX_ITERATIONS
        node.__class__.MAX_ITERATIONS = 2  # временно уменьшаем лимит

        call_count = 0

        original_stream_ret = [
            _make_chunk(finish_reason='tool_calls', usage={'prompt': 10, 'completion': 5, 'total': 15}),
        ]
        # tool_calls_accumulator не заполняется → tool_calls = None в результате
        # Нужно имитировать tool_calls в результате иначе цикл кончится сразу

        # Проще: мокаем поведение ThreadPoolExecutor внутри _continue_after_tool_calls
        # через прямое обращение к recursive_result через _do_recursive_streaming

        import concurrent.futures
        original_tpe = concurrent.futures.ThreadPoolExecutor

        iteration_tracker = [0]

        class FakeExecutor:
            def __init__(self, *a, **kw): pass
            def submit(self, fn):
                f = concurrent.futures.Future()
                # Симулируем fn() — модифицируем recursive_result через closure захват
                # Достаём recursive_result из параметра fn (default arg)
                import inspect
                defaults = fn.__defaults__ if hasattr(fn, '__defaults__') and fn.__defaults__ else ()
                if defaults:
                    result_dict = defaults[0]  # первый default = _result
                    iteration_tracker[0] += 1
                    result_dict['tool_calls'] = [
                        {'id': 'tc0', 'type': 'function',
                         'function': {'name': 'play_sound', 'arguments': '{}'}}
                    ]
                    result_dict['full_response'] = ''
                    result_dict['chunk_count'] = 0
                f.set_result(None)
                return f
            def shutdown(self, wait=False): pass

        original_execute = getattr(node, '_execute_tool_calls', None)
        node._execute_tool_calls = lambda tc, msgs: [
            {'tool_call_id': 'tc0', 'tool_name': 'play_sound', 'success': True, 'message': 'ok'}
        ]

        with patch('rob_box_voice.dialogue_node.ThreadPoolExecutor', FakeExecutor):
            node._continue_after_tool_calls(
                messages=[{"role": "system", "content": "sys"}],
                tool_calls=[{'id': 'tc0', 'type': 'function',
                             'function': {'name': 'play_sound', 'arguments': '{}'}}],
                tool_results=[{'tool_call_id': 'tc0', 'tool_name': 'play_sound',
                               'success': True, 'message': 'ok'}],
            )

        node.__class__.MAX_ITERATIONS = 30  # восстанавливаем
        node._speak_simple.assert_called_once()
        assert 'проблемой' in node._speak_simple.call_args[0][0].lower() or \
               'извините' in node._speak_simple.call_args[0][0].lower()


# ─────────────────────────────────────────────────────────────────────────────
#  Тесты финального ответа
# ─────────────────────────────────────────────────────────────────────────────

class TestAgentLoopFinalResponse:

    def test_plain_text_saved_to_history(self, node):
        """LLM возвращает plain text без tool_calls → сохраняется в conversation_history."""
        import concurrent.futures

        response_text = "Привет! Я робот."

        class FakeExecutor:
            def __init__(self, *a, **kw): pass
            def submit(self, fn):
                f = concurrent.futures.Future()
                defaults = fn.__defaults__ if hasattr(fn, '__defaults__') and fn.__defaults__ else ()
                if defaults:
                    result_dict = defaults[0]
                    result_dict['full_response'] = response_text
                    result_dict['chunk_count'] = 1  # уже был SSML чанк
                    result_dict['tool_calls'] = None
                f.set_result(None)
                return f
            def shutdown(self, wait=False): pass

        messages = [{"role": "system", "content": "sys"}]
        tool_calls = [{'id': 'tc1', 'type': 'function',
                       'function': {'name': 'get_time', 'arguments': '{}'}}]
        tool_results = [{'tool_call_id': 'tc1', 'tool_name': 'get_time',
                         'success': True, 'message': '12:00'}]

        with patch('rob_box_voice.dialogue_node.ThreadPoolExecutor', FakeExecutor):
            node._continue_after_tool_calls(messages, tool_calls, tool_results)

        assert node.llm_processing is False
        assert node.dialogue_in_progress is False
        # Сообщение добавлено в историю
        msgs = node.conversation_history.get_messages()
        assistant_msgs = [m for m in msgs if m['role'] == 'assistant']
        assert any(m.get('content') == response_text for m in assistant_msgs)

    def test_empty_final_response_no_crash(self, node):
        """LLM вернул пустой ответ (timeout-fallback) — нет краша, флаги сброшены."""
        import concurrent.futures

        class FakeExecutor:
            def __init__(self, *a, **kw): pass
            def submit(self, fn):
                f = concurrent.futures.Future()
                defaults = fn.__defaults__ if hasattr(fn, '__defaults__') and fn.__defaults__ else ()
                if defaults:
                    result_dict = defaults[0]
                    result_dict['full_response'] = ''
                    result_dict['chunk_count'] = 0
                    result_dict['tool_calls'] = None
                f.set_result(None)
                return f
            def shutdown(self, wait=False): pass

        with patch('rob_box_voice.dialogue_node.ThreadPoolExecutor', FakeExecutor):
            node._continue_after_tool_calls(
                messages=[{"role": "system", "content": "sys"}],
                tool_calls=[{'id': 'tc1', 'type': 'function',
                             'function': {'name': 'play_sound', 'arguments': '{}'}}],
                tool_results=[{'tool_call_id': 'tc1', 'tool_name': 'play_sound',
                               'success': True, 'message': 'ok'}],
            )

        assert node.llm_processing is False
        assert node.dialogue_in_progress is False

    def test_tool_results_sliding_window_limits_context_growth(self, node):
        """PF-2 (#827): локальный messages list не растёт без ограничений.

        Даже когда LLM бесконечно возвращает tool_calls (MAX_ITERATIONS=8
        в этом тесте), ``current_tool_results`` обрезается до
        ``AGENT_LOOP_KEEP_LAST_TOOL_RESULTS`` (5) — старые tool results
        не копятся в контексте (TASK-043 context rot).
        """
        import concurrent.futures

        node.__class__.MAX_ITERATIONS = 8  # временно уменьшаем лимит

        class FakeExecutor:
            def __init__(self, *a, **kw): pass
            def submit(self, fn):
                f = concurrent.futures.Future()
                defaults = fn.__defaults__ if hasattr(fn, '__defaults__') and fn.__defaults__ else ()
                if defaults:
                    result_dict = defaults[0]
                    result_dict['tool_calls'] = [
                        {'id': 'tc0', 'type': 'function',
                         'function': {'name': 'play_sound', 'arguments': '{}'}}
                    ]
                    result_dict['full_response'] = ''
                    result_dict['chunk_count'] = 0
                f.set_result(None)
                return f
            def shutdown(self, wait=False): pass

        node._execute_tool_calls = lambda tc, msgs: [
            {'tool_call_id': 'tc0', 'tool_name': 'play_sound',
             'success': True, 'message': 'ok'}
        ]

        with patch('rob_box_voice.dialogue_node.ThreadPoolExecutor', FakeExecutor):
            node._continue_after_tool_calls(
                messages=[{"role": "system", "content": "sys"}],
                tool_calls=[{'id': 'tc0', 'type': 'function',
                             'function': {'name': 'play_sound', 'arguments': '{}'}}],
                tool_results=[],
            )

        node.__class__.MAX_ITERATIONS = 30  # восстанавливаем

        # Скользящее окно: даже после 8 итераций в контекст уходит не
        # больше AGENT_LOOP_KEEP_LAST_TOOL_RESULTS tool results.
        kept = getattr(node, "_current_streaming_tool_results", None)
        assert kept is not None
        assert len(kept) <= node.AGENT_LOOP_KEEP_LAST_TOOL_RESULTS


# ─────────────────────────────────────────────────────────────────────────────
#  Тесты retry при timeout
# ─────────────────────────────────────────────────────────────────────────────

class TestAgentLoopRetry:

    def test_timeout_then_success_on_retry(self, node):
        """Первый запрос таймаут → retry → успех."""
        import concurrent.futures

        attempt = [0]

        class FakeExecutor:
            def __init__(self, *a, **kw): pass
            def submit(self, fn):
                f = concurrent.futures.Future()
                defaults = fn.__defaults__ if hasattr(fn, '__defaults__') and fn.__defaults__ else ()
                if defaults:
                    result_dict = defaults[0]
                    attempt[0] += 1
                    if attempt[0] == 1:
                        # Первая попытка: timeout (ничего не записываем в result,
                        # но raise TimeoutError через future)
                        result_dict['error'] = 'timeout'
                    else:
                        # Вторая попытка: успех
                        result_dict['full_response'] = 'Ответ после retry'
                        result_dict['chunk_count'] = 1
                        result_dict['tool_calls'] = None
                f.set_result(None)
                return f
            def shutdown(self, wait=False): pass

        with patch('rob_box_voice.dialogue_node.ThreadPoolExecutor', FakeExecutor):
            node._continue_after_tool_calls(
                messages=[{"role": "system", "content": "sys"}],
                tool_calls=[{'id': 'tc1', 'type': 'function',
                             'function': {'name': 'play_sound', 'arguments': '{}'}}],
                tool_results=[{'tool_call_id': 'tc1', 'tool_name': 'play_sound',
                               'success': True, 'message': 'ok'}],
            )

        assert attempt[0] == 2  # был retry
        assert node.llm_processing is False

    def test_double_timeout_calls_speak_simple(self, node):
        """Оба запроса упали с таймаутом → _speak_simple вызван с сообщением об ошибке."""
        import concurrent.futures

        class FakeExecutor:
            def __init__(self, *a, **kw): pass
            def submit(self, fn):
                f = concurrent.futures.Future()
                defaults = fn.__defaults__ if hasattr(fn, '__defaults__') and fn.__defaults__ else ()
                if defaults:
                    result_dict = defaults[0]
                    result_dict['error'] = 'timeout on both attempts'
                f.set_result(None)
                return f
            def shutdown(self, wait=False): pass

        with patch('rob_box_voice.dialogue_node.ThreadPoolExecutor', FakeExecutor):
            node._continue_after_tool_calls(
                messages=[{"role": "system", "content": "sys"}],
                tool_calls=[{'id': 'tc1', 'type': 'function',
                             'function': {'name': 'play_sound', 'arguments': '{}'}}],
                tool_results=[{'tool_call_id': 'tc1', 'tool_name': 'play_sound',
                               'success': True, 'message': 'ok'}],
            )

        node._speak_simple.assert_called_once()
        assert 'долго' in node._speak_simple.call_args[0][0].lower() or \
               'извините' in node._speak_simple.call_args[0][0].lower()
        assert node.llm_processing is False


# ─────────────────────────────────────────────────────────────────────────────
#  Тесты execute_tool_calls
# ─────────────────────────────────────────────────────────────────────────────

class TestExecuteToolCalls:

    def test_successful_tool_call(self, node):
        """Успешное выполнение одного tool call через mcp_adapter."""
        node.mcp_adapter.execute_tool_call_sync.return_value = {
            'success': True,
            'message': 'animation played',
            'data': None,
        }

        tool_calls = [
            {'id': 'tc1', 'type': 'function',
             'function': {'name': 'play_animation', 'arguments': '{"name": "wave"}'}}
        ]

        results = node._execute_tool_calls(tool_calls, messages=[])

        assert len(results) == 1
        assert results[0]['success'] is True
        assert results[0]['tool_name'] == 'play_animation'
        assert results[0]['tool_call_id'] == 'tc1'

    def test_invalid_json_args_returns_error(self, node):
        """Неверный JSON в аргументах → error result, не краш."""
        tool_calls = [
            {'id': 'tc2', 'type': 'function',
             'function': {'name': 'play_sound', 'arguments': 'not-json'}}
        ]

        results = node._execute_tool_calls(tool_calls, messages=[])

        assert len(results) == 1
        assert results[0]['success'] is False
        assert 'аргументов' in results[0]['error'].lower() or \
               'format' in results[0]['error'].lower()

    def test_multiple_tool_calls_all_executed(self, node):
        """Несколько tool_calls — все выполняются."""
        node.mcp_adapter.execute_tool_call_sync.return_value = {
            'success': True, 'message': 'ok', 'data': None,
        }

        tool_calls = [
            {'id': f'tc{i}', 'type': 'function',
             'function': {'name': 'play_sound', 'arguments': f'{{"id": {i}}}'}}
            for i in range(3)
        ]

        results = node._execute_tool_calls(tool_calls, messages=[])

        assert len(results) == 3
        assert all(r['success'] for r in results)

    def test_truncates_more_than_5_tool_calls(self, node):
        """Более 5 tool_calls — обрезаются до MAX_TOOL_CALLS=5."""
        node.mcp_adapter.execute_tool_call_sync.return_value = {
            'success': True, 'message': 'ok', 'data': None,
        }

        tool_calls = [
            {'id': f'tc{i}', 'type': 'function',
             'function': {'name': 'play_sound', 'arguments': '{}'}}
            for i in range(8)
        ]

        results = node._execute_tool_calls(tool_calls, messages=[])

        assert len(results) == 5
        assert node.mcp_adapter.execute_tool_call_sync.call_count == 5

    def test_failed_tool_stops_after_3_errors(self, node):
        """3 подряд ошибки → прекращаем выполнение оставшихся."""
        node.mcp_adapter.execute_tool_call_sync.return_value = {
            'success': False, 'message': '', 'error': 'device not found',
        }

        tool_calls = [
            {'id': f'tc{i}', 'type': 'function',
             'function': {'name': 'move_motor', 'arguments': '{}'}}
            for i in range(5)
        ]

        results = node._execute_tool_calls(tool_calls, messages=[])

        # Остановка после 3 ошибок
        assert len(results) == 3
        assert all(r['success'] is False for r in results)

    def test_mcp_adapter_exception_returns_error_result(self, node):
        """Исключение в mcp_adapter → error result, цикл продолжается."""
        node.mcp_adapter.execute_tool_call_sync.side_effect = RuntimeError("connection lost")

        tool_calls = [
            {'id': 'tc1', 'type': 'function',
             'function': {'name': 'get_status', 'arguments': '{}'}}
        ]

        results = node._execute_tool_calls(tool_calls, messages=[])

        assert len(results) == 1
        assert results[0]['success'] is False
        assert 'connection lost' in results[0]['error']


# ─────────────────────────────────────────────────────────────────────────────
#  Тесты флагов finally-блока
# ─────────────────────────────────────────────────────────────────────────────

class TestAgentLoopFinally:

    def test_finally_resets_flags_on_success(self, node):
        """После нормального завершения llm_processing и dialogue_in_progress=False."""
        import concurrent.futures

        class FakeExecutor:
            def __init__(self, *a, **kw): pass
            def submit(self, fn):
                f = concurrent.futures.Future()
                defaults = fn.__defaults__ if hasattr(fn, '__defaults__') and fn.__defaults__ else ()
                if defaults:
                    d = defaults[0]
                    d['full_response'] = 'Готово'
                    d['chunk_count'] = 1
                    d['tool_calls'] = None
                f.set_result(None)
                return f
            def shutdown(self, wait=False): pass

        node.llm_processing = True
        node.dialogue_in_progress = True

        with patch('rob_box_voice.dialogue_node.ThreadPoolExecutor', FakeExecutor):
            node._continue_after_tool_calls(
                messages=[{"role": "system", "content": "sys"}],
                tool_calls=[{'id': 'tc1', 'type': 'function',
                             'function': {'name': 'play_sound', 'arguments': '{}'}}],
                tool_results=[{'tool_call_id': 'tc1', 'tool_name': 'play_sound',
                               'success': True, 'message': 'ok'}],
            )

        assert node.llm_processing is False
        assert node.dialogue_in_progress is False
        assert node._listen_response_waiting is False

    def test_finally_keeps_dialogue_in_progress_when_waiting_for_stt(self, node):
        """Если _listen_response_waiting, dialogue_in_progress остаётся True."""
        tool_results = [
            {'tool_call_id': 'tc0', 'tool_name': 'listen_for_response',
             'success': True, 'message': 'listening'},
        ]
        tool_calls = [{'id': 'tc0', 'type': 'function',
                       'function': {'name': 'listen_for_response', 'arguments': '{}'}}]

        node.dialogue_in_progress = True

        node._continue_after_tool_calls(
            messages=[{"role": "system", "content": "sys"}],
            tool_calls=tool_calls,
            tool_results=tool_results,
        )

        # После listen_for_response: ждём STT
        assert node.dialogue_in_progress is True
        assert node.llm_processing is False
