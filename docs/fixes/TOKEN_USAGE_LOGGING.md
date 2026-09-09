# Token Usage Logging - Implementation Guide

## Overview

Added token usage logging for LLM API calls to track input (prompt) and output (completion) tokens. This enables cost analysis and performance monitoring.

## Changes Made

### File: `dialogue_node.py`

#### 1. Enable Token Usage in Streaming (2 locations)

**Initial Request (~line 891):**
```python
request_params = {
    "model": self.model,
    "messages": messages,
    "temperature": self.temperature,
    "max_tokens": self.max_tokens,
    "stream": True,
    "stream_options": {"include_usage": True}  # ← NEW!
}
```

**Recursive Agent Loop (~line 2141):**
```python
request_params = {
    "model": self.model,
    "messages": messages,
    "temperature": self.temperature,
    "max_tokens": self.max_tokens,
    "stream": True,
    "stream_options": {"include_usage": True}  # ← NEW!
}
```

#### 2. Extract and Log Token Usage (2 locations)

**Initial Request (~line 1061-1069):**
```python
# ============ Логируем использование токенов ============
if hasattr(chunk, 'usage') and chunk.usage:
    usage = chunk.usage
    prompt_tokens = getattr(usage, 'prompt_tokens', 0)
    completion_tokens = getattr(usage, 'completion_tokens', 0)
    total_tokens = getattr(usage, 'total_tokens', 0)
    self.get_logger().info(
        f"📊 Token usage: input={prompt_tokens}, output={completion_tokens}, total={total_tokens}"
    )
```

**Recursive Agent Loop (~line 2223-2231):**
```python
# ============ Логируем использование токенов ============
if hasattr(chunk, 'usage') and chunk.usage:
    usage = chunk.usage
    prompt_tokens = getattr(usage, 'prompt_tokens', 0)
    completion_tokens = getattr(usage, 'completion_tokens', 0)
    total_tokens = getattr(usage, 'total_tokens', 0)
    self.get_logger().info(
        f"📊 Token usage (recursive): input={prompt_tokens}, output={completion_tokens}, total={total_tokens}"
    )
```

## How It Works

### OpenAI Streaming API

According to OpenAI API specification:
- By default, streaming responses do NOT include token usage
- Setting `stream_options={"include_usage": True}` adds usage info to the **last chunk**
- The last chunk (when `finish_reason` is set) contains the `usage` object

### Token Types

1. **prompt_tokens (input)** - Tokens in the input prompt
   - Includes system prompt
   - Includes conversation history
   - Includes tool definitions (if MCP tools are used)
   - Includes tool results (in agent loop)

2. **completion_tokens (output)** - Tokens in LLM response
   - Includes generated text
   - Includes tool call JSON (if tools are called)

3. **total_tokens** - Sum of prompt_tokens + completion_tokens

## Log Output Examples

### Simple Request (No Tools)
```
[dialogue_node] [INFO] 🤔 Запрос к DeepSeek...
[dialogue_node] [INFO] 🏁 Stream завершён: stop (обработано 3 chunks)
[dialogue_node] [INFO] 📊 Token usage: input=142, output=67, total=209
```

### Request With Tool Calls
```
[dialogue_node] [INFO] 🤔 Запрос к DeepSeek...
[dialogue_node] [INFO] 🛠️  Отправка запроса с 17 MCP инструментами
[dialogue_node] [INFO] 🏁 Stream завершён: tool_calls (обработано 0 chunks)
[dialogue_node] [INFO] 📊 Token usage: input=1842, output=34, total=1876
[dialogue_node] [INFO] 🔧 LLM запросил выполнение 2 инструментов
... tool execution ...
[dialogue_node] [INFO] 🤖 Рекурсивный запрос к LLM с результатами инструментов
[dialogue_node] [INFO] 🏁 Рекурсивный stream завершён: stop
[dialogue_node] [INFO] 📊 Token usage (recursive): input=1923, output=89, total=2012
```

## Cost Analysis

### Token Pricing (Example: DeepSeek)
- Input: $0.14 per 1M tokens
- Output: $0.28 per 1M tokens

### Calculating Costs from Logs

**Example log entry:**
```
📊 Token usage: input=1842, output=89, total=1931
```

**Cost calculation:**
```
Input cost:  1842 tokens × $0.14 / 1,000,000 = $0.000258
Output cost:   89 tokens × $0.28 / 1,000,000 = $0.000025
Total cost:                                    $0.000283
```

### Automated Analysis Script

```python
import re

def analyze_token_costs(log_file, input_cost_per_1m=0.14, output_cost_per_1m=0.28):
    """Analyze token usage from log file"""
    
    with open(log_file, 'r') as f:
        log_content = f.read()
    
    # Find all token usage entries
    pattern = r'📊 Token usage.*?: input=(\d+), output=(\d+), total=(\d+)'
    matches = re.findall(pattern, log_content)
    
    total_input = 0
    total_output = 0
    total_tokens = 0
    
    for match in matches:
        input_tokens, output_tokens, total = map(int, match)
        total_input += input_tokens
        total_output += output_tokens
        total_tokens += total
    
    input_cost = total_input * input_cost_per_1m / 1_000_000
    output_cost = total_output * output_cost_per_1m / 1_000_000
    total_cost = input_cost + output_cost
    
    print(f"Total requests: {len(matches)}")
    print(f"Total input tokens: {total_input:,}")
    print(f"Total output tokens: {total_output:,}")
    print(f"Total tokens: {total_tokens:,}")
    print(f"Input cost: ${input_cost:.6f}")
    print(f"Output cost: ${output_cost:.6f}")
    print(f"Total cost: ${total_cost:.6f}")
    
    return {
        'requests': len(matches),
        'input_tokens': total_input,
        'output_tokens': total_output,
        'total_tokens': total_tokens,
        'input_cost': input_cost,
        'output_cost': output_cost,
        'total_cost': total_cost
    }

# Usage
analyze_token_costs('clog')
```

## Performance Insights

### Input Tokens Analysis

High input token counts indicate:
- Long conversation history (accumulates over dialogue)
- Large system prompts
- Many tool definitions (17 tools = ~1500 tokens)
- Tool results from previous calls (in agent loop)

**Optimization opportunities:**
- Trim conversation history after N messages
- Simplify system prompts
- Only send relevant tools (not all 17)
- Compress tool result descriptions

### Output Tokens Analysis

High output token counts indicate:
- Long LLM responses
- Complex tool calls (large JSON parameters)
- Multiple tool calls in one request

**Optimization opportunities:**
- Set lower max_tokens limit
- Use more concise prompts
- Request shorter responses

## Monitoring Recommendations

### Real-time Monitoring

Monitor for:
- **Sudden spikes** in token usage (indicates problem)
- **Recursive loops** consuming many tokens (check agent loop iterations)
- **Tool definition overhead** (compare requests with/without tools)

### Historical Analysis

Track over time:
- Average tokens per request
- Cost per user interaction
- Token usage by time of day
- Agent loop efficiency (tokens per iteration)

### Alerts

Set up alerts for:
- Single request exceeding 5000 tokens
- Total daily usage exceeding budget
- Average output tokens > max_tokens setting

## Compatibility

### Supported Providers

✅ **DeepSeek** - Full support for stream_options
✅ **Qwen** - Full support for stream_options
✅ **OpenAI GPT** - Full support (original implementation)

### Fallback Behavior

If provider doesn't support `stream_options`:
- Request still succeeds (ignored parameter)
- Token usage not logged (no error)
- All other functionality works normally

## Testing

### Manual Test

1. Deploy to robot with changes
2. Ask robot a question: "Робокс расскажи анекдот"
3. Check logs for token usage:
```bash
grep "📊 Token usage" clog
```

Expected output:
```
[dialogue_node] 📊 Token usage: input=150, output=67, total=217
```

### Test With Tools

1. Ask robot to use tools: "Робокс покажи анимацию"
2. Check logs for both initial and recursive:
```bash
grep "📊 Token usage" clog
```

Expected output:
```
[dialogue_node] 📊 Token usage: input=1842, output=34, total=1876
[dialogue_node] 📊 Token usage (recursive): input=1923, output=89, total=2012
```

## Troubleshooting

### No Token Usage in Logs

**Problem:** No "📊 Token usage" messages appear

**Possible causes:**
1. Provider doesn't support stream_options
2. Usage data not in chunk (check API version)
3. Error before finish_reason is processed

**Solution:**
- Check if `chunk.usage` exists by adding debug logging
- Verify OpenAI library version: `pip show openai`
- Check provider API documentation

### Wrong Token Counts

**Problem:** Token counts seem incorrect

**Possible causes:**
1. Counting both initial and recursive as single request
2. Missing some requests in analysis
3. Different encoding method than expected

**Solution:**
- Distinguish initial vs recursive in logs (already done)
- Parse logs carefully (separate normal vs recursive)
- Compare with provider's dashboard

## Future Enhancements

### Cumulative Tracking

Add session-level tracking:
```python
class DialogueNode:
    def __init__(self):
        self.session_tokens = {
            'input': 0,
            'output': 0,
            'total': 0,
            'requests': 0
        }
    
    def _log_token_usage(self, prompt_tokens, completion_tokens, total_tokens):
        self.session_tokens['input'] += prompt_tokens
        self.session_tokens['output'] += completion_tokens
        self.session_tokens['total'] += total_tokens
        self.session_tokens['requests'] += 1
        
        self.get_logger().info(
            f"📊 Token usage: input={prompt_tokens}, output={completion_tokens}, total={total_tokens}"
        )
        self.get_logger().info(
            f"   Session total: {self.session_tokens['total']} tokens in {self.session_tokens['requests']} requests"
        )
```

### Cost Tracking

Add cost calculation:
```python
COST_PER_1M_TOKENS = {
    'deepseek': {'input': 0.14, 'output': 0.28},
    'qwen': {'input': 0.40, 'output': 1.20}
}

def _calculate_cost(self, provider, prompt_tokens, completion_tokens):
    costs = COST_PER_1M_TOKENS[provider]
    input_cost = prompt_tokens * costs['input'] / 1_000_000
    output_cost = completion_tokens * costs['output'] / 1_000_000
    total_cost = input_cost + output_cost
    
    self.get_logger().info(
        f"💰 Request cost: ${total_cost:.6f} (in=${input_cost:.6f}, out=${output_cost:.6f})"
    )
```

### Prometheus Metrics

Export metrics for monitoring:
```python
from prometheus_client import Counter, Histogram

llm_tokens_total = Counter(
    'llm_tokens_total',
    'Total LLM tokens used',
    ['provider', 'type']  # type: input/output
)

llm_request_tokens = Histogram(
    'llm_request_tokens',
    'Tokens per LLM request',
    ['provider']
)

# In logging code:
llm_tokens_total.labels(provider='deepseek', type='input').inc(prompt_tokens)
llm_tokens_total.labels(provider='deepseek', type='output').inc(completion_tokens)
llm_request_tokens.labels(provider='deepseek').observe(total_tokens)
```

---

## Summary

✅ Token usage logging implemented
✅ Works for both initial and recursive requests
✅ Compatible with DeepSeek and Qwen
✅ Easy to parse from logs
✅ Enables cost and performance analysis
✅ Non-intrusive to existing functionality
