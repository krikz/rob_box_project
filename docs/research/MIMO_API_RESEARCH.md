# MiMo v2.5 Pro API Research

**Date**: 2026-06-14
**Source**: https://mimo.mi.com/docs/en-US/api/chat/openai-api
**Purpose**: Validate parameter usage in dialogue_node.py, understand quirks and best practices

## Parameter Reference

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `frequency_penalty` | number \| null | [-2.0, 2.0] | 0 | Penalizes new tokens based on existing frequency. Decreases likelihood of verbatim repetition. |
| `presence_penalty` | number \| null | [-2.0, 2.0] | 0 | Penalizes tokens based on whether they appear in text. Increases likelihood of new topics. |
| `temperature` | number | [0, 1.5] | 1.0 (pro) | Sampling temperature. Higher = more random. **Forced to 1.0 in thinking mode.** |
| `top_p` | number | [0.01, 1.0] | 0.95 | Nucleus sampling threshold. **Forced to 0.95 in thinking mode.** |
| `max_completion_tokens` | integer | [1, 131072] | 131072 (pro) | Upper bound for generated tokens (including reasoning tokens). |
| `tool_choice` | string | `auto` only | auto | **Any value other than "auto" is silently removed by backend!** |
| `thinking` | object | enabled/disabled | enabled (pro) | Chain of thought. When disabled, temperature/top_p work normally. |
| `stop` | string \| array \| null | max 4 sequences | null | Stop sequences. Not supported for TTS models. |

## Critical Findings

### 1. frequency_penalty and presence_penalty are TOP-LEVEL parameters

**They go directly in the request body, NOT in extra_body!**

```json
{
    "model": "mimo-v2.5-pro",
    "messages": [...],
    "frequency_penalty": 0.3,
    "presence_penalty": 0.2,
    "thinking": {"type": "disabled"}
}
```

This means in OpenAI Agents SDK, they should be passed via `extra_body` (since ModelSettings
doesn't have dedicated fields), but they ARE standard OpenAI-compatible parameters.

**Current implementation in dialogue_node.py**:
```python
extra_body={
    "thinking": {"type": "disabled"},
    "frequency_penalty": 0.3,
    "presence_penalty": 0.2,
}
```

This is **CORRECT** — `extra_body` merges into the top-level request body.

### 2. tool_choice only supports "auto"

> Note: When a value other than `auto` is passed to `tool_choice`, the backend will remove
> this field by default, and the model response behavior will still be equivalent to the
> `auto` mode.

**Implication**: Do NOT try `tool_choice="required"` or `tool_choice={"type": "function", ...}`.
Only `"auto"` works. Current code uses `"auto"` — correct.

### 3. Thinking mode forces temperature and top_p

> In thinking mode, the mimo-v2.5-pro, mimo-v2.5, mimo-v2-pro and mimo-v2-omni models do
> NOT support customizing the temperature and top_p parameters. Even if these parameters are
> passed in, the actual effective values will be forcibly set by the model to its recommended
> default values of 1.0 and 0.95.

**Implication**: Since we disable thinking (`thinking.type=disabled`), our temperature=0.7
setting IS effective. If we ever enable thinking, temperature would be ignored.

### 4. finish_reason values

| finish_reason | Meaning | Handling |
|---------------|---------|----------|
| `stop` | Natural stop point | Normal — process response |
| `length` | max_completion_tokens reached | Truncated — may need retry or warning |
| `tool_calls` | Model called a tool | Normal — execute tool |
| `content_filter` | Content omitted by safety filter | Log warning, may need rephrase |
| `repetition_truncation` | Model detected repetition | **NEW!** Log warning, retry with different params |

**`repetition_truncation` is critical** — this means MiMo detected the LLM was repeating itself
and cut off the response. This could explain some empty or partial responses!

### 5. reasoning_content in multi-turn

> During the multi-turn tool calls process in thinking mode, the model returns a
> `reasoning_content` field alongside `tool_calls`. To continue the conversation, it is
> recommended to keep all previous `reasoning_content` in the `messages` array for each
> subsequent request to achieve the best performance.

**Implication**: Since we disable thinking, this doesn't apply. But if we ever enable thinking,
we need to preserve reasoning_content in message history.

## Best Practices for Avoiding Repetition

### frequency_penalty (0.3 recommended)
- Positive values penalize tokens that already appeared frequently
- 0.3 = mild penalty — reduces verbatim code block repetition
- Too high (>1.0) may cause incoherent responses
- **Best for**: Preventing copy-paste of same DJ transition code

### presence_penalty (0.2 recommended)
- Positive values penalize ANY token that has appeared at all
- 0.2 = mild nudge toward new topics/patterns
- Too high (>0.8) may cause random topic changes
- **Best for**: Encouraging variety in synth/pattern choices

### Combined usage
- frequency_penalty=0.3 + presence_penalty=0.2 = good balance
- Reduces repetition without sacrificing coherence
- Values from official example: frequency_penalty=0, presence_penalty=0 (defaults)

## Retry Patterns

### When to retry
1. **Empty response** (content=null, no tool_calls) → retry once with same params
2. **finish_reason="repetition_truncation"** → retry once with higher frequency_penalty (0.5)
3. **finish_reason="content_filter"** → log warning, may need to rephrase user input
4. **finish_reason="length"** → response truncated, may need to increase max_completion_tokens

### When NOT to retry
- finish_reason="stop" with valid content → normal
- finish_reason="tool_calls" → normal (execute the tool)
- Multiple consecutive retries → give up after 1 retry

## OpenAI Agents SDK Integration

The OpenAI Agents SDK passes parameters to the OpenAI API client. Key mapping:

| SDK Field | API Parameter | Notes |
|-----------|---------------|-------|
| `ModelSettings.temperature` | `temperature` | Top-level, works when thinking disabled |
| `ModelSettings.max_tokens` | `max_tokens` | Maps to max_completion_tokens |
| `ModelSettings.parallel_tool_calls` | `parallel_tool_calls` | We set to False |
| `ModelSettings.extra_body` | merged into body | **frequency_penalty, presence_penalty go here** |

**Verified**: `extra_body={"thinking": {"type": "disabled"}, "frequency_penalty": 0.3, "presence_penalty": 0.2}`
is the correct way to pass these parameters through the OpenAI Agents SDK.

## Summary of Changes Needed

### dialogue_node.py
1. ✅ frequency_penalty=0.3 and presence_penalty=0.2 in extra_body — **ALREADY CORRECT**
2. ⚠️ Add handling for finish_reason="repetition_truncation" — log warning
3. ⚠️ Add handling for finish_reason="content_filter" — log warning
4. ⚠️ Expand "done" filter to catch variants
5. ⚠️ Add DJ empty response retry logic

### No changes needed for:
- tool_choice (already "auto")
- temperature (already set, effective because thinking disabled)
- thinking (already disabled)
