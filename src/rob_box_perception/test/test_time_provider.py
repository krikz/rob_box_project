#!/usr/bin/env python3
"""Test for TimeAwarenessProvider - ensures timezone support works correctly."""

import pytest


def test_time_provider_imports():
    """Test that TimeAwarenessProvider can be imported."""
    from rob_box_perception.utils.time_provider import TimeAwarenessProvider
    assert TimeAwarenessProvider is not None


def test_pytz_available():
    """Test that pytz is available for timezone support."""
    from rob_box_perception.utils.time_provider import PYTZ_AVAILABLE
    assert PYTZ_AVAILABLE is True, 'pytz must be installed for timezone support'


def test_moscow_timezone():
    """Test that Moscow timezone works correctly."""
    from rob_box_perception.utils.time_provider import TimeAwarenessProvider

    provider = TimeAwarenessProvider(timezone='Europe/Moscow')
    time_context = provider.get_current_time_context()

    # Verify timezone is set correctly
    assert time_context['timezone'] == 'Europe/Moscow'

    # Verify all required fields are present
    required_fields = [
        'timestamp', 'datetime', 'human_readable', 'time_only', 'date_only',
        'hour', 'minute', 'weekday', 'weekday_ru', 'period', 'period_ru', 'timezone'
    ]
    for field in required_fields:
        assert field in time_context, f'Missing field: {field}'

    # Verify hour is in valid range
    assert 0 <= time_context['hour'] <= 23

    # Verify minute is in valid range
    assert 0 <= time_context['minute'] <= 59

    # Verify period_ru is one of the valid values
    assert time_context['period_ru'] in ['утро', 'день', 'вечер', 'ночь']

    # Verify weekday_ru is valid
    valid_weekdays = [
        'Понедельник', 'Вторник', 'Среда', 'Четверг', 'Пятница',
        'Суббота', 'Воскресенье',
    ]
    assert time_context['weekday_ru'] in valid_weekdays


def test_moscow_timezone_offset():
    """Test that Moscow timezone has correct UTC offset (+3 hours).

    Note: Russia abolished DST in 2014, Moscow permanently uses UTC+3 (MSK).
    """
    from rob_box_perception.utils.time_provider import TimeAwarenessProvider

    provider = TimeAwarenessProvider(timezone='Europe/Moscow')
    time_context = provider.get_current_time_context()

    # Parse the datetime with timezone info
    dt_str = time_context['datetime']
    assert '+03:00' in dt_str, \
        f'Moscow time should have UTC+3 offset (MSK), got: {dt_str}'


def test_time_consistency():
    """Test that time_only matches the hour from human_readable."""
    from rob_box_perception.utils.time_provider import TimeAwarenessProvider

    provider = TimeAwarenessProvider(timezone='Europe/Moscow')
    time_context = provider.get_current_time_context()

    # Extract hour from time_only (format: "HH:MM")
    time_only_hour = int(time_context['time_only'].split(':')[0])

    # Should match the hour field
    assert time_only_hour == time_context['hour'], \
        f"time_only hour ({time_only_hour}) doesn't match hour field ({time_context['hour']})"


def test_timezone_parameter_flexibility():
    """Test that TimeAwarenessProvider accepts different timezone parameters."""
    from rob_box_perception.utils.time_provider import TimeAwarenessProvider

    # Test multiple timezones
    timezones_to_test = [
        ('Europe/Moscow', '+03:00'),
        ('America/New_York', '-04:00', '-05:00'),  # EDT or EST
        ('Asia/Tokyo', '+09:00'),
        ('UTC', '+00:00'),
    ]

    for tz_data in timezones_to_test:
        timezone = tz_data[0]
        expected_offsets = tz_data[1:] if len(tz_data) > 1 else (tz_data[1],)

        provider = TimeAwarenessProvider(timezone=timezone)
        time_context = provider.get_current_time_context()

        # Verify timezone is set correctly
        assert time_context['timezone'] == timezone, \
            f"Expected timezone {timezone}, got {time_context['timezone']}"

        # Verify offset is in expected range
        dt_str = time_context['datetime']
        assert any(offset in dt_str for offset in expected_offsets), \
            f'Timezone {timezone} should have offset in {expected_offsets}, got: {dt_str}'


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
