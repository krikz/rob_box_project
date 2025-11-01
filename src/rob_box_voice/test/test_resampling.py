#!/usr/bin/env python3
"""
Test resampling function for TTS Node

Verifies that audio resampling from 22050 Hz to 16000 Hz works correctly.
"""

import numpy as np
import sys


def resample_audio(audio: np.ndarray, orig_sr: int, target_sr: int) -> np.ndarray:
    """
    Resample audio from original sample rate to target sample rate using linear interpolation.
    
    Args:
        audio: Audio data as numpy array (mono, float32, range -1.0 to 1.0)
        orig_sr: Original sample rate (e.g., 22050)
        target_sr: Target sample rate (e.g., 16000)
    
    Returns:
        Resampled audio at target sample rate
    """
    if orig_sr == target_sr:
        return audio
    
    # Calculate the resampling ratio
    duration = len(audio) / orig_sr
    target_length = int(duration * target_sr)
    
    # Create new time indices for interpolation
    orig_indices = np.linspace(0, len(audio) - 1, len(audio))
    target_indices = np.linspace(0, len(audio) - 1, target_length)
    
    # Linear interpolation
    resampled = np.interp(target_indices, orig_indices, audio)
    
    return resampled


def test_resample_audio():
    """Test resampling function"""
    
    print("🧪 Test Audio Resampling\n")
    
    # Test case 1: Resample from 22050 Hz to 16000 Hz (Yandex case)
    orig_sr = 22050
    target_sr = 16000
    duration = 1.0  # 1 second
    
    # Generate a simple sine wave at 440 Hz (A note)
    t = np.linspace(0, duration, int(orig_sr * duration))
    audio_orig = np.sin(2 * np.pi * 440 * t).astype(np.float32)
    
    print(f"📊 Original audio:")
    print(f"   Sample rate: {orig_sr} Hz")
    print(f"   Length: {len(audio_orig)} samples")
    print(f"   Duration: {len(audio_orig) / orig_sr:.3f} seconds")
    print(f"   Range: [{audio_orig.min():.3f}, {audio_orig.max():.3f}]")
    
    # Resample
    audio_resampled = resample_audio(audio_orig, orig_sr, target_sr)
    
    print(f"\n📊 Resampled audio:")
    print(f"   Sample rate: {target_sr} Hz")
    print(f"   Length: {len(audio_resampled)} samples")
    print(f"   Duration: {len(audio_resampled) / target_sr:.3f} seconds")
    print(f"   Range: [{audio_resampled.min():.3f}, {audio_resampled.max():.3f}]")
    
    # Expected length after resampling
    expected_length = int(duration * target_sr)
    
    # Verify
    print(f"\n✅ Verification:")
    print(f"   Expected length: {expected_length} samples")
    print(f"   Actual length: {len(audio_resampled)} samples")
    print(f"   Difference: {abs(len(audio_resampled) - expected_length)} samples")
    
    # Check that duration is preserved
    orig_duration = len(audio_orig) / orig_sr
    resampled_duration = len(audio_resampled) / target_sr
    duration_error = abs(orig_duration - resampled_duration)
    
    print(f"\n⏱️  Duration preservation:")
    print(f"   Original duration: {orig_duration:.6f} seconds")
    print(f"   Resampled duration: {resampled_duration:.6f} seconds")
    print(f"   Error: {duration_error:.6f} seconds ({duration_error * 1000:.3f} ms)")
    
    # Test case 2: No resampling needed (same rate)
    audio_same = resample_audio(audio_orig, orig_sr, orig_sr)
    print(f"\n🔄 Same rate test:")
    print(f"   Input length: {len(audio_orig)}")
    print(f"   Output length: {len(audio_same)}")
    print(f"   Arrays equal: {np.array_equal(audio_orig, audio_same)}")
    
    # Test case 3: Silero to ReSpeaker (24000 Hz to 16000 Hz)
    silero_sr = 24000
    audio_silero = np.sin(2 * np.pi * 440 * np.linspace(0, 1, silero_sr)).astype(np.float32)
    audio_silero_resampled = resample_audio(audio_silero, silero_sr, target_sr)
    
    print(f"\n🔄 Silero resampling test:")
    print(f"   Input: {len(audio_silero)} samples @ {silero_sr} Hz")
    print(f"   Output: {len(audio_silero_resampled)} samples @ {target_sr} Hz")
    print(f"   Duration preserved: {abs(len(audio_silero)/silero_sr - len(audio_silero_resampled)/target_sr) < 0.001}")
    
    # Success criteria
    success = (
        abs(len(audio_resampled) - expected_length) <= 1  # Within 1 sample
        and duration_error < 0.001  # Within 1ms
        and np.array_equal(audio_orig, audio_same)  # Same rate returns same array
    )
    
    print(f"\n{'✅ PASS: All tests passed!' if success else '❌ FAIL: Some tests failed!'}")
    
    return success


if __name__ == '__main__':
    success = test_resample_audio()
    sys.exit(0 if success else 1)
