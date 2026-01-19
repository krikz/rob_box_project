#!/usr/bin/env python3
"""
async_executor.py - Асинхронный движок выполнения MCP инструментов

Обеспечивает:
- Параллельное выполнение инструментов через asyncio.gather()
- Прерывание длительных операций (LONG tasks)
- Fire-and-forget для мгновенных операций (INSTANT)
- Правильное handling разных execution types
"""

import asyncio
import json
import uuid
from typing import Dict, Any, List, Optional, Callable
from dataclasses import dataclass
from enum import Enum

from .base import ToolExecutionType


@dataclass
class InterruptibleTask:
    """Прерываемая задача для LONG операций"""
    
    task: asyncio.Task
    interrupt_event: asyncio.Event
    tool_name: str
    request_id: str
    parameters: Dict[str, Any]
    created_at: float
    
    async def cancel(self) -> bool:
        """
        Прервать выполнение задачи
        
        Returns:
            True если задача была успешно отменена
        """
        if not self.task.done():
            self.interrupt_event.set()  # Сигнал для проверки внутри задачи
            self.task.cancel()
            
            try:
                await self.task
            except asyncio.CancelledError:
                return True
            
        return False


class ToolCallAccumulator:
    """
    Накопитель tool_calls из streaming chunks
    
    LLM API возвращает tool_calls по частям в streaming режиме:
    - chunk 1: {tool_calls: [{index: 0, id: "call_123", type: "function", function: {name: "set_emotion"}}]}
    - chunk 2: {tool_calls: [{index: 0, function: {arguments: '{"emo'}}]}
    - chunk 3: {tool_calls: [{index: 0, function: {arguments: 'tion": '}}]}
    - chunk 4: {tool_calls: [{index: 0, function: {arguments: '"радость"}'}}]}
    
    Accumulator собирает всё в единую структуру
    """
    
    def __init__(self):
        self.tool_calls_buffer: Dict[int, Dict[str, Any]] = {}
        
    def add_chunk(self, delta_tool_calls: List[Any]) -> None:
        """
        Добавить chunk tool_calls из streaming response
        
        Args:
            delta_tool_calls: Список tool_call delta objects из OpenAI API
        """
        for tc in delta_tool_calls:
            index = tc.index
            
            if index not in self.tool_calls_buffer:
                # Инициализация новой tool_call
                self.tool_calls_buffer[index] = {
                    "id": getattr(tc, "id", None),
                    "type": getattr(tc, "type", "function"),
                    "function": {
                        "name": None,
                        "arguments": ""
                    }
                }
            
            # Обновление существующей tool_call
            if hasattr(tc, "id") and tc.id:
                self.tool_calls_buffer[index]["id"] = tc.id
                
            if hasattr(tc, "function") and tc.function:
                func = tc.function
                if hasattr(func, "name") and func.name:
                    self.tool_calls_buffer[index]["function"]["name"] = func.name
                if hasattr(func, "arguments") and func.arguments:
                    self.tool_calls_buffer[index]["function"]["arguments"] += func.arguments
    
    def get_complete_tool_calls(self) -> List[Dict[str, Any]]:
        """
        Получить полный список собранных tool_calls
        
        Returns:
            Список tool_calls в формате {id, type, function: {name, arguments}}
        """
        # Сортируем по index
        sorted_calls = [self.tool_calls_buffer[idx] for idx in sorted(self.tool_calls_buffer.keys())]
        
        # Парсим arguments из JSON string
        result = []
        for call in sorted_calls:
            try:
                arguments_str = call["function"]["arguments"]
                parsed_args = json.loads(arguments_str) if arguments_str else {}
                
                result.append({
                    "id": call["id"],
                    "type": call["type"],
                    "function": {
                        "name": call["function"]["name"],
                        "arguments": parsed_args
                    }
                })
            except json.JSONDecodeError:
                # Если не смогли распарсить - оставляем как есть
                result.append(call)
        
        return result
    
    def clear(self) -> None:
        """Очистить буфер"""
        self.tool_calls_buffer.clear()


class AsyncToolExecutor:
    """
    Асинхронный executor для MCP инструментов
    
    Обеспечивает:
    - Параллельное выполнение независимых tool_calls
    - Fire-and-forget для INSTANT операций
    - Прерывание LONG операций
    - Правильные timeouts для каждого типа
    """
    
    def __init__(self, execute_pub, result_callback: Callable, logger):
        """
        Args:
            execute_pub: ROS publisher для /mcp/execute
            result_callback: Callback для получения результатов (request_id, result)
            logger: ROS logger
        """
        self.execute_pub = execute_pub
        self.result_callback = result_callback
        self.logger = logger
        
        # Реестр активных LONG задач
        self.long_tasks: Dict[str, InterruptibleTask] = {}
        
        # Кэш результатов: request_id -> result
        self.results_cache: Dict[str, Dict[str, Any]] = {}
        
        # Timeouts по типам (seconds)
        self.timeouts = {
            ToolExecutionType.INSTANT: 0.1,   # Fire-and-forget, не ждём
            ToolExecutionType.FAST: 2.0,       # Звуки, короткие операции
            ToolExecutionType.MEDIUM: 10.0,    # Запросы данных
            ToolExecutionType.LONG: 300.0,     # Навигация, mapping (5 минут)
        }
    
    def on_result_received(self, request_id: str, result: Dict[str, Any]) -> None:
        """
        Callback когда получен результат от MCP сервера
        
        Args:
            request_id: ID запроса
            result: Результат выполнения
        """
        self.results_cache[request_id] = result
        
        # Удаляем из long_tasks если там был
        if request_id in self.long_tasks:
            del self.long_tasks[request_id]
        
        # Вызываем внешний callback
        if self.result_callback:
            self.result_callback(request_id, result)
    
    async def execute_tool_async(
        self,
        tool_name: str,
        parameters: Dict[str, Any],
        execution_type: ToolExecutionType,
        request_id: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Асинхронное выполнение одного инструмента
        
        Args:
            tool_name: Имя инструмента
            parameters: Параметры
            execution_type: Тип выполнения
            request_id: ID запроса (опционально)
        
        Returns:
            Результат выполнения
        """
        if request_id is None:
            request_id = str(uuid.uuid4())
        
        # Формируем запрос
        request = {
            "tool_name": tool_name,
            "parameters": parameters,
            "request_id": request_id
        }
        
        # Публикуем запрос в ROS
        from std_msgs.msg import String
        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        self.execute_pub.publish(msg)
        
        self.logger.info(f"📤 Отправлен {execution_type.value} запрос {request_id[:8]}: {tool_name}")
        
        # Логика в зависимости от типа
        if execution_type == ToolExecutionType.INSTANT:
            # Fire-and-forget: не ждём результата
            await asyncio.sleep(0.01)  # Минимальная задержка для yield
            return {
                "success": True,
                "message": f"{tool_name} запущен (fire-and-forget)",
                "request_id": request_id
            }
        
        elif execution_type in [ToolExecutionType.FAST, ToolExecutionType.MEDIUM]:
            # Await с timeout
            timeout = self.timeouts[execution_type]
            
            try:
                result = await self._wait_for_result(request_id, timeout)
                return result
            except asyncio.TimeoutError:
                self.logger.error(f"⏱️ Timeout {timeout}s для {tool_name} (request_id: {request_id[:8]})")
                return {
                    "success": False,
                    "error": f"Timeout {timeout}s",
                    "request_id": request_id
                }
        
        elif execution_type == ToolExecutionType.LONG:
            # Background task с возможностью прерывания
            interrupt_event = asyncio.Event()
            
            # Создаём задачу с interrupt check
            task = asyncio.create_task(
                self._execute_long_with_interrupt(request_id, interrupt_event, self.timeouts[execution_type])
            )
            
            # Регистрируем в long_tasks
            import time
            interruptible_task = InterruptibleTask(
                task=task,
                interrupt_event=interrupt_event,
                tool_name=tool_name,
                request_id=request_id,
                parameters=parameters,
                created_at=time.time()
            )
            self.long_tasks[request_id] = interruptible_task
            
            # Не ждём завершения - возвращаем сразу
            return {
                "success": True,
                "message": f"{tool_name} запущен в фоне",
                "request_id": request_id,
                "background": True
            }
        
        else:
            # Unknown type
            self.logger.warning(f"⚠️ Неизвестный execution_type: {execution_type}")
            return await self._wait_for_result(request_id, 5.0)
    
    async def _wait_for_result(self, request_id: str, timeout: float) -> Dict[str, Any]:
        """
        Ожидание результата с timeout
        
        Args:
            request_id: ID запроса
            timeout: Timeout в секундах
        
        Returns:
            Результат выполнения
        """
        start_time = asyncio.get_event_loop().time()
        
        while request_id not in self.results_cache:
            if asyncio.get_event_loop().time() - start_time > timeout:
                raise asyncio.TimeoutError()
            
            await asyncio.sleep(0.05)  # Poll interval
        
        # Получаем результат из кэша
        result_data = self.results_cache.pop(request_id)
        return result_data.get("result", {"success": False, "error": "Пустой результат"})
    
    async def _execute_long_with_interrupt(
        self,
        request_id: str,
        interrupt_event: asyncio.Event,
        timeout: float
    ) -> Dict[str, Any]:
        """
        Выполнение LONG задачи с проверкой прерывания
        
        Args:
            request_id: ID запроса
            interrupt_event: Event для прерывания
            timeout: Timeout
        
        Returns:
            Результат выполнения
        """
        try:
            # Ждём результата или прерывания
            result_task = asyncio.create_task(self._wait_for_result(request_id, timeout))
            interrupt_task = asyncio.create_task(interrupt_event.wait())
            
            done, pending = await asyncio.wait(
                [result_task, interrupt_task],
                return_when=asyncio.FIRST_COMPLETED
            )
            
            # Отменяем pending задачи
            for task in pending:
                task.cancel()
            
            # Проверяем что завершилось
            if interrupt_task in done:
                # Прерывание
                self.logger.warning(f"🛑 LONG задача {request_id[:8]} прервана пользователем")
                return {
                    "success": False,
                    "error": "Операция прервана пользователем",
                    "interrupted": True
                }
            else:
                # Результат получен
                return result_task.result()
                
        except asyncio.CancelledError:
            self.logger.warning(f"🛑 LONG задача {request_id[:8]} отменена")
            return {
                "success": False,
                "error": "Операция отменена",
                "cancelled": True
            }
    
    async def execute_tools_parallel(
        self,
        tool_calls: List[Dict[str, Any]],
        tool_registry: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Параллельное выполнение множества tool_calls
        
        Группирует инструменты по execution_type и выполняет оптимально:
        - INSTANT: Fire-and-forget параллельно
        - FAST: Await параллельно через gather()
        - MEDIUM: Await параллельно через gather()
        - LONG: Запуск в фоне без ожидания
        
        Args:
            tool_calls: Список tool_calls в формате {id, function: {name, arguments}}
            tool_registry: Опциональный реестр инструментов для определения execution_type
        
        Returns:
            Список результатов выполнения
        """
        if not tool_calls:
            return []
        
        self.logger.info(f"🔧 Параллельное выполнение {len(tool_calls)} инструментов")
        
        # Группировка по execution_type
        instant_tasks = []
        fast_tasks = []
        medium_tasks = []
        long_tasks = []
        
        for tool_call in tool_calls:
            tool_name = tool_call["function"]["name"]
            parameters = tool_call["function"]["arguments"]
            tool_call_id = tool_call.get("id", str(uuid.uuid4()))
            
            # Определяем execution_type
            # TODO: Получать из tool_registry когда будет доступен
            # Пока используем эвристику по имени
            if tool_name in ["set_emotion", "play_animation"]:
                execution_type = ToolExecutionType.INSTANT
            elif tool_name in ["play_sound"]:
                execution_type = ToolExecutionType.FAST
            elif tool_name in ["navigate_to_waypoint", "move_direction", "start_mapping"]:
                execution_type = ToolExecutionType.LONG
            else:
                execution_type = ToolExecutionType.MEDIUM
            
            task_info = {
                "tool_name": tool_name,
                "parameters": parameters,
                "execution_type": execution_type,
                "request_id": tool_call_id
            }
            
            if execution_type == ToolExecutionType.INSTANT:
                instant_tasks.append(task_info)
            elif execution_type == ToolExecutionType.FAST:
                fast_tasks.append(task_info)
            elif execution_type == ToolExecutionType.MEDIUM:
                medium_tasks.append(task_info)
            elif execution_type == ToolExecutionType.LONG:
                long_tasks.append(task_info)
        
        # Выполнение
        results = []
        
        # 1. INSTANT - fire-and-forget параллельно
        if instant_tasks:
            instant_coros = [
                self.execute_tool_async(t["tool_name"], t["parameters"], t["execution_type"], t["request_id"])
                for t in instant_tasks
            ]
            instant_results = await asyncio.gather(*instant_coros, return_exceptions=True)
            results.extend(instant_results)
        
        # 2. FAST - await параллельно
        if fast_tasks:
            fast_coros = [
                self.execute_tool_async(t["tool_name"], t["parameters"], t["execution_type"], t["request_id"])
                for t in fast_tasks
            ]
            fast_results = await asyncio.gather(*fast_coros, return_exceptions=True)
            results.extend(fast_results)
        
        # 3. MEDIUM - await параллельно
        if medium_tasks:
            medium_coros = [
                self.execute_tool_async(t["tool_name"], t["parameters"], t["execution_type"], t["request_id"])
                for t in medium_tasks
            ]
            medium_results = await asyncio.gather(*medium_coros, return_exceptions=True)
            results.extend(medium_results)
        
        # 4. LONG - запуск в фоне, не ждём
        if long_tasks:
            for t in long_tasks:
                result = await self.execute_tool_async(
                    t["tool_name"], t["parameters"], t["execution_type"], t["request_id"]
                )
                results.append(result)
        
        self.logger.info(f"✅ Выполнено {len(results)} инструментов (parallel)")
        return results
    
    async def interrupt_all_long_tasks(self) -> int:
        """
        Прервать все активные LONG задачи
        
        Returns:
            Количество прерванных задач
        """
        if not self.long_tasks:
            return 0
        
        self.logger.warning(f"🛑 Прерывание {len(self.long_tasks)} LONG задач")
        
        cancelled_count = 0
        tasks_to_cancel = list(self.long_tasks.values())
        
        for task in tasks_to_cancel:
            if await task.cancel():
                cancelled_count += 1
        
        self.long_tasks.clear()
        
        self.logger.info(f"✅ Прервано {cancelled_count} задач")
        return cancelled_count
    
    async def interrupt_task_by_name(self, tool_name: str) -> int:
        """
        Прервать все LONG задачи с указанным именем
        
        Args:
            tool_name: Имя инструмента
        
        Returns:
            Количество прерванных задач
        """
        cancelled_count = 0
        tasks_to_cancel = [
            (request_id, task) 
            for request_id, task in self.long_tasks.items() 
            if task.tool_name == tool_name
        ]
        
        for request_id, task in tasks_to_cancel:
            if await task.cancel():
                cancelled_count += 1
                del self.long_tasks[request_id]
        
        if cancelled_count > 0:
            self.logger.info(f"🛑 Прервано {cancelled_count} задач {tool_name}")
        
        return cancelled_count
