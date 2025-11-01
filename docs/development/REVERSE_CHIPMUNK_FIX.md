# Fix for "Reverse Chipmunk" Voice Issue

## Problem Description

After PR #20, users reported that the voice sounded "like from hell" (too slow and deep) - the opposite of the chipmunk effect. This was called the "reverse chipmunk" or "обратный бурундук" effect.

## Root Cause

The issue was a **sample rate mismatch without resampling**:

1. **Yandex TTS** returns audio at **22050 Hz**
2. **Silero TTS** returns audio at **24000 Hz**
3. **ReSpeaker** plays audio at **16000 Hz** (hardcoded in tts_node.py line 482)

Without proper resampling, the audio was played at the wrong speed:
- **Yandex:** 22050 Hz audio played at 16000 Hz = **16000/22050 = 0.726x playback rate** = **1.378x SLOWER** than intended
- **Silero:** 24000 Hz audio played at 16000 Hz = **16000/24000 = 0.667x playback rate** = **1.5x SLOWER** than intended

Result: Voice sounded too deep and too slow ("from hell").

## Historical Context

### Original ROBBOX (before PR #20)
- Yandex synthesized at `speed=0.4` (slow speech)
- Audio returned at 22050 Hz
- Played at 44100 Hz (sounddevice on desktop)
- Playback rate: 44100/22050 = **2x FASTER** = "chipmunk effect" (high and fast)
- Effective speed: 0.4 × 2 = 0.8 = slightly slower than normal

### After PR #20 (broken)
- Yandex synthesized at `speed=1.0` (normal speech)
- Audio returned at 22050 Hz
- Played at 16000 Hz (ReSpeaker on robot)
- Playback rate: 16000/22050 = **0.726x** = **1.378x SLOWER**
- Result: "Reverse chipmunk" - too slow and deep

### After This Fix (correct)
- Yandex synthesizes at `speed=1.0` (normal speech)
- Audio returned at 22050 Hz
- **Resampled to 16000 Hz** (preserving duration)
- Played at 16000 Hz
- Playback rate: 16000/16000 = **1.0x** = **normal speed**
- Result: Normal voice, as intended

## Solution

Added `resample_audio()` function that uses linear interpolation to resample audio from the source sample rate (22050 Hz or 24000 Hz) to the target sample rate (16000 Hz) BEFORE playback.

### Implementation

```python
def resample_audio(audio: np.ndarray, orig_sr: int, target_sr: int) -> np.ndarray:
    """Resample audio using linear interpolation"""
    if orig_sr == target_sr:
        return audio
    
    duration = len(audio) / orig_sr
    target_length = int(duration * target_sr)
    
    orig_indices = np.linspace(0, len(audio) - 1, len(audio))
    target_indices = np.linspace(0, len(audio) - 1, target_length)
    
    resampled = np.interp(target_indices, orig_indices, audio)
    return resampled
```

### Usage in tts_node.py

```python
# After synthesis (audio_np is at 22050 Hz or 24000 Hz)
target_rate = 16000  # ReSpeaker

if sample_rate != target_rate:
    audio_np = resample_audio(audio_np, sample_rate, target_rate)
    sample_rate = target_rate

# Now audio is at correct 16000 Hz, ready for playback
```

## Testing

Test file: `src/rob_box_voice/test/test_resampling.py`

Results:
- ✅ Resampling from 22050 Hz to 16000 Hz works correctly
- ✅ Duration is preserved (1.000000 seconds)
- ✅ Sample count is correct (16000 samples for 1 second)
- ✅ Resampling from 24000 Hz to 16000 Hz works correctly
- ✅ Same rate returns same audio (no unnecessary processing)

## Impact

This fix resolves the "reverse chipmunk" issue and ensures that:
1. Voice plays at normal speed (not too fast, not too slow)
2. Voice pitch is correct (not too high, not too deep)
3. Both Yandex and Silero TTS work correctly
4. Chipmunk mode still available via `chipmunk_mode=True` parameter

## Related Issues

- Original issue: "обратный бурундук" (reverse chipmunk)
- PR #20: Fixed original chipmunk but introduced reverse chipmunk
- This PR: Fixes reverse chipmunk with proper resampling
