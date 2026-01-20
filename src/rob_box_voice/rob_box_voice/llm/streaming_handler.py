"""
StreamingHandler - Обработчик LLM streaming запросов с JSON chunking
"""

import json
import re
import time
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, field


@dataclass
class StreamingResult:
    """Результат streaming обработки"""
    full_response: str = ""
    chunk_count: int = 0
    tool_calls: List[Dict[str, Any]] = field(default_factory=list)
    error: Optional[str] = None


class StreamingHandler:
    """
    Обработчик LLM streaming с поддержкой:
    - JSON chunking (баланс скобок)
    - Tool calls accumulation
    - Timeout handling
    - SSML/accent processing
    """
    
    def __init__(
        self,
        chunk_timeout: float = 15.0,
        on_chunk_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        on_tool_call_callback: Optional[Callable[[List[Dict]], None]] = None,
        logger: Optional[Any] = None
    ):
        """
        Args:
            chunk_timeout: Timeout между chunks (секунды)
            on_chunk_callback: Callback для каждого распарсенного chunk
            on_tool_call_callback: Callback для tool calls
            logger: Logger для debug сообщений
        """
        self.chunk_timeout = chunk_timeout
        self.on_chunk_callback = on_chunk_callback
        self.on_tool_call_callback = on_tool_call_callback
        self.logger = logger
        
    def process_stream(
        self,
        stream,
        dialogue_id: str,
        accent_processor: Optional[Callable[[str], str]] = None
    ) -> StreamingResult:
        """
        Обработка stream от LLM API
        
        Args:
            stream: Iterator от OpenAI/compatible API
            dialogue_id: ID диалога для отслеживания
            accent_processor: Опциональный процессор для добавления ударений
            
        Returns:
            StreamingResult с accumulated данными
        """
        result = StreamingResult()
        
        # State для JSON parsing
        current_chunk = ""
        brace_count = 0
        in_json = False
        
        # State для timeout
        start_time = time.time()
        last_chunk_time = start_time
        
        # Накопитель для tool_calls (приходят по частям)
        tool_calls_accumulator: Dict[int, Dict[str, Any]] = {}
        
        try:
            for chunk in stream:
                # Проверка timeout
                elapsed = time.time() - last_chunk_time
                if elapsed > self.chunk_timeout:
                    result.error = f"No data for {elapsed:.1f}s (after {result.chunk_count} chunks)"
                    return result
                
                # Обработка tool_calls
                if hasattr(chunk.choices[0].delta, 'tool_calls') and chunk.choices[0].delta.tool_calls:
                    self._accumulate_tool_calls(chunk.choices[0].delta.tool_calls, tool_calls_accumulator)
                    last_chunk_time = time.time()
                
                # Обработка content
                if chunk.choices[0].delta.content:
                    token = chunk.choices[0].delta.content
                    result.full_response += token
                    current_chunk += token
                    last_chunk_time = time.time()
                    
                    # Подсчёт скобок для JSON
                    for char in token:
                        if char == "{":
                            brace_count += 1
                            in_json = True
                        elif char == "}":
                            brace_count -= 1
                    
                    # Если скобки сбалансированы - парсим JSON
                    if in_json and brace_count == 0:
                        parsed_count = self._parse_and_emit_chunks(
                            current_chunk,
                            dialogue_id,
                            accent_processor
                        )
                        result.chunk_count += parsed_count
                        
                        # Сброс для следующего chunk
                        current_chunk = ""
                        in_json = False
                        brace_count = 0
                
                # Проверка finish_reason
                if chunk.choices[0].finish_reason:
                    finish_reason = chunk.choices[0].finish_reason
                    
                    if self.logger:
                        self.logger.info(f"🏁 Stream завершён: {finish_reason} ({result.chunk_count} chunks)")
                    
                    # Если LLM запросил tool_calls
                    if finish_reason == 'tool_calls' and tool_calls_accumulator:
                        result.tool_calls = list(tool_calls_accumulator.values())
                        if self.logger:
                            self.logger.info(f"🔧 LLM запросил {len(result.tool_calls)} tool calls")
                        if self.on_tool_call_callback:
                            self.on_tool_call_callback(result.tool_calls)
                    
                    break
            
            # FALLBACK: Если plain text без JSON
            if result.chunk_count == 0 and len(result.full_response) > 0:
                if self.logger:
                    self.logger.warning(f"⚠️  Plain text без JSON ({len(result.full_response)} chars)")
                
                chunk_data = {
                    "chunk": "end",
                    "ssml": f"<speak>{result.full_response.strip()}</speak>",
                    "emotion": "neutral",
                    "dialogue_id": dialogue_id
                }
                
                if self.on_chunk_callback:
                    self.on_chunk_callback(chunk_data)
                
                result.chunk_count = 1
        
        except Exception as e:
            result.error = str(e)
            if self.logger:
                self.logger.error(f"❌ Stream error: {e}")
        
        return result
    
    def _accumulate_tool_calls(
        self,
        tc_chunks: List[Any],
        accumulator: Dict[int, Dict[str, Any]]
    ):
        """Накопление tool_calls из streaming chunks"""
        for tc_chunk in tc_chunks:
            idx = tc_chunk.index
            
            # Инициализация накопителя
            if idx not in accumulator:
                accumulator[idx] = {
                    'id': '',
                    'type': 'function',
                    'function': {
                        'name': '',
                        'arguments': ''
                    }
                }
            
            # Накопление данных
            if tc_chunk.id:
                accumulator[idx]['id'] = tc_chunk.id
            if hasattr(tc_chunk, 'function'):
                if tc_chunk.function.name:
                    accumulator[idx]['function']['name'] = tc_chunk.function.name
                if tc_chunk.function.arguments:
                    accumulator[idx]['function']['arguments'] += tc_chunk.function.arguments
            
            if self.logger:
                self.logger.debug(f"🔧 Tool call chunk: index={idx}")
    
    def _parse_and_emit_chunks(
        self,
        text: str,
        dialogue_id: str,
        accent_processor: Optional[Callable[[str], str]] = None
    ) -> int:
        """
        Парсинг и публикация JSON chunks
        
        Returns:
            Количество успешно обработанных chunks
        """
        count = 0
        
        # Разбиваем по переносам и по }{
        json_objects = []
        lines = text.strip().split('\n')
        
        for line in lines:
            line = line.strip()
            if not line:
                continue
            # Разделяем склеенные JSON: }{
            parts = re.split(r'(?<=\})(?=\{)', line)
            json_objects.extend([p.strip() for p in parts if p.strip()])
        
        for json_text in json_objects:
            json_text = json_text.strip()
            if not json_text:
                continue
            
            # Убираем markdown
            if json_text.startswith("```json"):
                json_text = json_text.replace("```json", "").replace("```", "").strip()
            
            try:
                chunk_data = json.loads(json_text)
                
                if self.logger:
                    self.logger.info(
                        f"✅ JSON: chunk={chunk_data.get('chunk', '?')}, "
                        f"emotion={chunk_data.get('emotion', '?')}"
                    )
                
                # Применяем accent processor если есть
                if accent_processor and "ssml" in chunk_data:
                    chunk_data["ssml"] = accent_processor(chunk_data["ssml"])
                
                # Добавляем dialogue_id
                chunk_data["dialogue_id"] = dialogue_id
                
                # Callback
                if self.on_chunk_callback:
                    self.on_chunk_callback(chunk_data)
                
                count += 1
                
            except json.JSONDecodeError as e:
                if self.logger:
                    self.logger.warning(f"⚠️  JSON decode failed: {e}, text: {json_text[:200]}...")
        
        return count
