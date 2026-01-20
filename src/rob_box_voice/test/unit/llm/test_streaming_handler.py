"""Tests for StreamingHandler"""

import unittest
from unittest.mock import Mock, MagicMock
from rob_box_voice.llm.streaming_handler import StreamingHandler, StreamingResult


class TestStreamingHandlerBasics(unittest.TestCase):
    """Test basic StreamingHandler functionality"""
    
    def test_init(self):
        """Test initialization"""
        handler = StreamingHandler(chunk_timeout=10.0)
        self.assertEqual(handler.chunk_timeout, 10.0)
        self.assertIsNone(handler.on_chunk_callback)
    
    def test_init_with_callbacks(self):
        """Test initialization with callbacks"""
        chunk_cb = Mock()
        tool_cb = Mock()
        logger = Mock()
        
        handler = StreamingHandler(
            chunk_timeout=15.0,
            on_chunk_callback=chunk_cb,
            on_tool_call_callback=tool_cb,
            logger=logger
        )
        
        self.assertEqual(handler.chunk_timeout, 15.0)
        self.assertEqual(handler.on_chunk_callback, chunk_cb)
        self.assertEqual(handler.on_tool_call_callback, tool_cb)
        self.assertEqual(handler.logger, logger)


class TestStreamingResultDataclass(unittest.TestCase):
    """Test StreamingResult dataclass"""
    
    def test_default_values(self):
        """Test default values"""
        result = StreamingResult()
        self.assertEqual(result.full_response, "")
        self.assertEqual(result.chunk_count, 0)
        self.assertEqual(result.tool_calls, [])
        self.assertIsNone(result.error)
    
    def test_with_values(self):
        """Test with custom values"""
        tool_calls = [{"id": "call_123", "function": {"name": "test"}}]
        result = StreamingResult(
            full_response="Test response",
            chunk_count=5,
            tool_calls=tool_calls,
            error="Test error"
        )
        self.assertEqual(result.full_response, "Test response")
        self.assertEqual(result.chunk_count, 5)
        self.assertEqual(result.tool_calls, tool_calls)
        self.assertEqual(result.error, "Test error")


class TestStreamingHandlerJSONParsing(unittest.TestCase):
    """Test JSON parsing logic"""
    
    def test_parse_and_emit_chunks_single_json(self):
        """Test parsing single JSON object"""
        handler = StreamingHandler()
        chunks_received = []
        
        def chunk_callback(chunk):
            chunks_received.append(chunk)
        
        handler.on_chunk_callback = chunk_callback
        
        json_text = '{"chunk": 1, "ssml": "<speak>Hello</speak>", "emotion": "happy"}'
        count = handler._parse_and_emit_chunks(json_text, "dlg_123")
        
        self.assertEqual(count, 1)
        self.assertEqual(len(chunks_received), 1)
        self.assertEqual(chunks_received[0]["chunk"], 1)
        self.assertEqual(chunks_received[0]["emotion"], "happy")
        self.assertEqual(chunks_received[0]["dialogue_id"], "dlg_123")
    
    def test_parse_and_emit_chunks_multiple_json(self):
        """Test parsing multiple JSON objects"""
        handler = StreamingHandler()
        chunks_received = []
        
        def chunk_callback(chunk):
            chunks_received.append(chunk)
        
        handler.on_chunk_callback = chunk_callback
        
        # Multiple JSONs on one line (DeepSeek style)
        json_text = '{"chunk": 1}{"chunk": 2}'
        count = handler._parse_and_emit_chunks(json_text, "dlg_123")
        
        self.assertEqual(count, 2)
        self.assertEqual(len(chunks_received), 2)
        self.assertEqual(chunks_received[0]["chunk"], 1)
        self.assertEqual(chunks_received[1]["chunk"], 2)
    
    def test_parse_and_emit_chunks_with_newlines(self):
        """Test parsing JSONs with newlines (Qwen style)"""
        handler = StreamingHandler()
        chunks_received = []
        
        def chunk_callback(chunk):
            chunks_received.append(chunk)
        
        handler.on_chunk_callback = chunk_callback
        
        json_text = '{"chunk": 1}\n{"chunk": 2}\n'
        count = handler._parse_and_emit_chunks(json_text, "dlg_123")
        
        self.assertEqual(count, 2)
        self.assertEqual(len(chunks_received), 2)
    
    def test_parse_and_emit_chunks_with_markdown(self):
        """Test parsing JSON with markdown wrapper"""
        handler = StreamingHandler()
        chunks_received = []
        
        def chunk_callback(chunk):
            chunks_received.append(chunk)
        
        handler.on_chunk_callback = chunk_callback
        
        json_text = '```json\n{"chunk": 1}\n```'
        count = handler._parse_and_emit_chunks(json_text, "dlg_123")
        
        self.assertEqual(count, 1)
        self.assertEqual(len(chunks_received), 1)
    
    def test_parse_and_emit_chunks_with_accent_processor(self):
        """Test accent processing"""
        handler = StreamingHandler()
        chunks_received = []
        
        def chunk_callback(chunk):
            chunks_received.append(chunk)
        
        def accent_processor(text):
            return text + "_accented"
        
        handler.on_chunk_callback = chunk_callback
        
        json_text = '{"chunk": 1, "ssml": "<speak>Hello</speak>"}'
        count = handler._parse_and_emit_chunks(json_text, "dlg_123", accent_processor)
        
        self.assertEqual(count, 1)
        self.assertEqual(chunks_received[0]["ssml"], "<speak>Hello</speak>_accented")
    
    def test_parse_and_emit_chunks_invalid_json(self):
        """Test handling invalid JSON"""
        handler = StreamingHandler(logger=Mock())
        chunks_received = []
        
        def chunk_callback(chunk):
            chunks_received.append(chunk)
        
        handler.on_chunk_callback = chunk_callback
        
        json_text = '{"chunk": invalid json'
        count = handler._parse_and_emit_chunks(json_text, "dlg_123")
        
        self.assertEqual(count, 0)
        self.assertEqual(len(chunks_received), 0)


class TestStreamingHandlerToolCalls(unittest.TestCase):
    """Test tool calls accumulation"""
    
    def test_accumulate_tool_calls_single(self):
        """Test accumulating single tool call"""
        handler = StreamingHandler()
        accumulator = {}
        
        # Mock tool call chunk
        tc_chunk = Mock()
        tc_chunk.index = 0
        tc_chunk.id = "call_123"
        tc_chunk.function = Mock()
        tc_chunk.function.name = "test_function"
        tc_chunk.function.arguments = '{"arg1": "value1"}'
        
        handler._accumulate_tool_calls([tc_chunk], accumulator)
        
        self.assertEqual(len(accumulator), 1)
        self.assertEqual(accumulator[0]['id'], "call_123")
        self.assertEqual(accumulator[0]['function']['name'], "test_function")
        self.assertEqual(accumulator[0]['function']['arguments'], '{"arg1": "value1"}')
    
    def test_accumulate_tool_calls_incremental(self):
        """Test accumulating tool call arguments incrementally"""
        handler = StreamingHandler()
        accumulator = {}
        
        # First chunk with name
        tc_chunk1 = Mock()
        tc_chunk1.index = 0
        tc_chunk1.id = "call_123"
        tc_chunk1.function = Mock()
        tc_chunk1.function.name = "test_function"
        tc_chunk1.function.arguments = '{"arg1":'
        
        handler._accumulate_tool_calls([tc_chunk1], accumulator)
        
        # Second chunk with more arguments
        tc_chunk2 = Mock()
        tc_chunk2.index = 0
        tc_chunk2.id = None  # ID already set
        tc_chunk2.function = Mock()
        tc_chunk2.function.name = None  # Name already set
        tc_chunk2.function.arguments = ' "value1"}'
        
        handler._accumulate_tool_calls([tc_chunk2], accumulator)
        
        self.assertEqual(len(accumulator), 1)
        self.assertEqual(accumulator[0]['function']['arguments'], '{"arg1": "value1"}')
    
    def test_accumulate_tool_calls_multiple(self):
        """Test accumulating multiple tool calls"""
        handler = StreamingHandler()
        accumulator = {}
        
        # Tool call 1
        tc_chunk1 = Mock()
        tc_chunk1.index = 0
        tc_chunk1.id = "call_123"
        tc_chunk1.function = Mock()
        tc_chunk1.function.name = "func1"
        tc_chunk1.function.arguments = '{"a": 1}'
        
        # Tool call 2
        tc_chunk2 = Mock()
        tc_chunk2.index = 1
        tc_chunk2.id = "call_456"
        tc_chunk2.function = Mock()
        tc_chunk2.function.name = "func2"
        tc_chunk2.function.arguments = '{"b": 2}'
        
        handler._accumulate_tool_calls([tc_chunk1, tc_chunk2], accumulator)
        
        self.assertEqual(len(accumulator), 2)
        self.assertEqual(accumulator[0]['id'], "call_123")
        self.assertEqual(accumulator[1]['id'], "call_456")


class TestStreamingHandlerProcessStream(unittest.TestCase):
    """Test full stream processing"""
    
    def test_process_stream_simple_content(self):
        """Test processing stream with simple content"""
        handler = StreamingHandler()
        chunks_received = []
        
        def chunk_callback(chunk):
            chunks_received.append(chunk)
        
        handler.on_chunk_callback = chunk_callback
        
        # Mock stream
        mock_chunk1 = Mock()
        mock_chunk1.choices = [Mock()]
        mock_chunk1.choices[0].delta = Mock()
        mock_chunk1.choices[0].delta.content = '{"chunk": 1, "ssml": "<speak>Hello</speak>", "emotion": "happy"}'
        mock_chunk1.choices[0].delta.tool_calls = None
        mock_chunk1.choices[0].finish_reason = None
        
        mock_chunk2 = Mock()
        mock_chunk2.choices = [Mock()]
        mock_chunk2.choices[0].delta = Mock()
        mock_chunk2.choices[0].delta.content = None
        mock_chunk2.choices[0].delta.tool_calls = None
        mock_chunk2.choices[0].finish_reason = "stop"
        
        stream = [mock_chunk1, mock_chunk2]
        
        result = handler.process_stream(stream, "dlg_123")
        
        self.assertEqual(result.chunk_count, 1)
        self.assertEqual(len(chunks_received), 1)
        self.assertIsNone(result.error)
    
    def test_process_stream_plain_text_fallback(self):
        """Test fallback for plain text without JSON"""
        handler = StreamingHandler()
        chunks_received = []
        
        def chunk_callback(chunk):
            chunks_received.append(chunk)
        
        handler.on_chunk_callback = chunk_callback
        
        # Mock stream with plain text
        mock_chunk1 = Mock()
        mock_chunk1.choices = [Mock()]
        mock_chunk1.choices[0].delta = Mock()
        mock_chunk1.choices[0].delta.content = "Plain text response"
        mock_chunk1.choices[0].delta.tool_calls = None
        mock_chunk1.choices[0].finish_reason = None
        
        mock_chunk2 = Mock()
        mock_chunk2.choices = [Mock()]
        mock_chunk2.choices[0].delta = Mock()
        mock_chunk2.choices[0].delta.content = None
        mock_chunk2.choices[0].delta.tool_calls = None
        mock_chunk2.choices[0].finish_reason = "stop"
        
        stream = [mock_chunk1, mock_chunk2]
        
        result = handler.process_stream(stream, "dlg_123")
        
        self.assertEqual(result.chunk_count, 1)  # Fallback creates 1 chunk
        self.assertEqual(len(chunks_received), 1)
        self.assertEqual(chunks_received[0]["ssml"], "<speak>Plain text response</speak>")


if __name__ == '__main__':
    unittest.main()
