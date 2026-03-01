"""
Tests for PromptFormatter module
"""

import unittest
import json
from rob_box_perception.core.prompt_formatter import PromptFormatter


class MockPerceptionEvent:
    """Mock PerceptionEvent for testing"""
    
    def __init__(self):
        self.current_time_human = "2025-01-20 15:30:00"
        self.time_period = "afternoon"
        self.time_context_json = json.dumps({
            "period_ru": "день",
            "is_special": False
        })
        self.system_health_status = "HEALTHY"
        self.health_issues = []
        self.internet_available = True
        self.battery_percent = 85.0
        self.is_charging = False
        self.disk_usage_percent = 45.0
        self.has_obstacle_ahead = False
        self.obstacle_distance_meters = 5.0
        self.current_map_name = "main_floor"
        self.nav_goal_description = "kitchen"
        self.nav_status = "active"
        self.apriltag_count = 0
        self.apriltag_summary = ""
        self.has_vision = False
        self.vision_summary = ""
        self.vision_objects_json = ""


class TestPromptFormatter(unittest.TestCase):
    """Test PromptFormatter"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.formatter = PromptFormatter()
        self.ctx = MockPerceptionEvent()
    
    def test_initialization(self):
        """Test PromptFormatter initialization"""
        formatter = PromptFormatter()
        self.assertIsNotNone(formatter)
    
    def test_format_context_summary_basic(self):
        """Test basic context summary formatting"""
        summary = self.formatter.format_context_summary(self.ctx)
        
        self.assertIn("ТЕКУЩЕЕ СОСТОЯНИЕ", summary)
        self.assertIn("день", summary)
        self.assertIn("15:30:00", summary)
        self.assertIn("Здоровье: HEALTHY", summary)
        self.assertIn("Батарея: 85%", summary)
        self.assertIn("Интернет: доступен", summary)
    
    def test_format_context_summary_with_health_issues(self):
        """Test summary with health issues"""
        self.ctx.health_issues = ["Low battery", "High disk usage"]
        
        summary = self.formatter.format_context_summary(self.ctx)
        
        self.assertIn("Проблемы:", summary)
        self.assertIn("Low battery", summary)
        self.assertIn("High disk usage", summary)
    
    def test_format_context_summary_charging(self):
        """Test summary with charging battery"""
        self.ctx.is_charging = True
        
        summary = self.formatter.format_context_summary(self.ctx)
        
        self.assertIn("Заряжается", summary)
    
    def test_format_context_summary_with_obstacle(self):
        """Test summary with obstacle"""
        self.ctx.has_obstacle_ahead = True
        self.ctx.obstacle_distance_meters = 1.5
        
        summary = self.formatter.format_context_summary(self.ctx)
        
        self.assertIn("Препятствие", summary)
        self.assertIn("1.5", summary)
    
    def test_format_context_summary_with_apriltags(self):
        """Test summary with AprilTags"""
        self.ctx.apriltag_count = 3
        
        summary = self.formatter.format_context_summary(self.ctx)
        
        self.assertIn("AprilTag", summary)
        self.assertIn("3", summary)
    
    def test_format_context_summary_with_vision(self):
        """Test summary with vision data"""
        self.ctx.has_vision = True
        self.ctx.vision_summary = "Person standing in hallway, holding cup"
        
        summary = self.formatter.format_context_summary(self.ctx)
        
        self.assertIn("Вижу:", summary)
        self.assertIn("Person standing", summary)
    
    def test_format_context_summary_no_internet(self):
        """Test summary with no internet"""
        self.ctx.internet_available = False
        
        summary = self.formatter.format_context_summary(self.ctx)
        
        self.assertIn("Интернет: недоступен", summary)
    
    def test_format_context_summary_high_disk_usage(self):
        """Test summary with high disk usage"""
        self.ctx.disk_usage_percent = 92.0
        
        summary = self.formatter.format_context_summary(self.ctx)
        
        self.assertIn("Диск: 92%", summary)
    
    def test_format_context_for_prompt_basic(self):
        """Test basic detailed context formatting"""
        prompt = self.formatter.format_context_for_prompt(self.ctx)
        
        self.assertIn("=== ВРЕМЯ ===", prompt)
        self.assertIn("=== ЗДОРОВЬЕ СИСТЕМЫ ===", prompt)
        self.assertIn("=== БАТАРЕЯ ===", prompt)
        self.assertIn("=== СЕНСОРЫ ===", prompt)
        self.assertIn("=== НАВИГАЦИЯ ===", prompt)
    
    def test_format_context_for_prompt_with_thoughts(self):
        """Test detailed context with recent thoughts"""
        thoughts = [
            "Battery level is good",
            "Navigation is active",
            "All systems nominal"
        ]
        
        prompt = self.formatter.format_context_for_prompt(self.ctx, thoughts)
        
        self.assertIn("=== МОИ МЫСЛИ (последние) ===", prompt)
        self.assertIn("Battery level is good", prompt)
        self.assertIn("Navigation is active", prompt)
        self.assertIn("All systems nominal", prompt)
    
    def test_format_context_for_prompt_health_degraded(self):
        """Test detailed context with degraded health"""
        self.ctx.system_health_status = "DEGRADED"
        self.ctx.health_issues = ["Battery low", "Disk full"]
        
        prompt = self.formatter.format_context_for_prompt(self.ctx)
        
        self.assertIn("Статус: DEGRADED", prompt)
        self.assertIn("Проблемы:", prompt)
        self.assertIn("Battery low", prompt)
        self.assertIn("Disk full", prompt)
    
    def test_format_context_for_prompt_with_vision_objects(self):
        """Test detailed context with vision objects"""
        self.ctx.has_vision = True
        self.ctx.vision_summary = "Person in room"
        self.ctx.vision_objects_json = json.dumps([
            {"label": "person", "confidence": 0.95},
            {"label": "chair", "confidence": 0.87},
            {"label": "table", "confidence": 0.76}
        ])
        
        prompt = self.formatter.format_context_for_prompt(self.ctx)
        
        self.assertIn("=== КАМЕРА (OAK-D) ===", prompt)
        self.assertIn("Вижу: Person in room", prompt)
        self.assertIn("Объекты:", prompt)
        self.assertIn("person (95%)", prompt)
        self.assertIn("chair (87%)", prompt)
    
    def test_format_context_for_prompt_special_time(self):
        """Test detailed context with special time"""
        self.ctx.time_context_json = json.dumps({
            "period_ru": "утро",
            "is_special": True,
            "special_description": "Начало рабочего дня"
        })
        
        prompt = self.formatter.format_context_for_prompt(self.ctx)
        
        self.assertIn("Период дня: утро", prompt)
        self.assertIn("Особое время: Начало рабочего дня", prompt)
    
    def test_format_context_for_prompt_no_health_issues(self):
        """Test detailed context with healthy status"""
        prompt = self.formatter.format_context_for_prompt(self.ctx)
        
        self.assertIn("Проблем не обнаружено", prompt)
    
    def test_format_context_for_prompt_obstacle(self):
        """Test detailed context with obstacle"""
        self.ctx.has_obstacle_ahead = True
        self.ctx.obstacle_distance_meters = 0.8
        
        prompt = self.formatter.format_context_for_prompt(self.ctx)
        
        self.assertIn("ПРЕПЯТСТВИЕ ВПЕРЕДИ", prompt)
        self.assertIn("0.8", prompt)
    
    def test_format_context_for_prompt_free_path(self):
        """Test detailed context with free path"""
        self.ctx.has_obstacle_ahead = False
        
        prompt = self.formatter.format_context_for_prompt(self.ctx)
        
        self.assertIn("Путь свободен", prompt)
    
    def test_format_context_for_prompt_apriltags(self):
        """Test detailed context with AprilTags"""
        self.ctx.apriltag_count = 2
        self.ctx.apriltag_summary = "Door markers detected"
        
        prompt = self.formatter.format_context_for_prompt(self.ctx)
        
        self.assertIn("=== APRILTAG ===", prompt)
        self.assertIn("Обнаружено меток: 2", prompt)
        self.assertIn("Описание: Door markers detected", prompt)
    
    def test_format_context_summary_truncates_vision(self):
        """Test that vision summary is truncated in summary"""
        long_vision = "Person standing in the hallway, wearing blue jacket, holding coffee cup, looking at phone, " \
                     "with backpack on floor next to them, near the window with sunlight coming through"
        self.ctx.has_vision = True
        self.ctx.vision_summary = long_vision
        
        summary = self.formatter.format_context_summary(self.ctx)
        
        # Should be truncated to 100 chars
        vision_line = [line for line in summary.split('\n') if 'Вижу:' in line][0]
        # Check that it's truncated (should not contain the last part)
        self.assertNotIn("sunlight coming through", vision_line)
    
    def test_format_context_for_prompt_limits_thoughts(self):
        """Test that only last 3 thoughts are included"""
        thoughts = [
            "Thought 1",
            "Thought 2",
            "Thought 3",
            "Thought 4",
            "Thought 5"
        ]
        
        prompt = self.formatter.format_context_for_prompt(self.ctx, thoughts)
        
        self.assertNotIn("Thought 1", prompt)
        self.assertNotIn("Thought 2", prompt)
        self.assertIn("Thought 3", prompt)
        self.assertIn("Thought 4", prompt)
        self.assertIn("Thought 5", prompt)
    
    def test_format_context_for_prompt_limits_vision_objects(self):
        """Test that only top 5 vision objects are included"""
        objects = [
            {"label": f"object_{i}", "confidence": 0.9 - i * 0.05}
            for i in range(10)
        ]
        self.ctx.has_vision = True
        self.ctx.vision_objects_json = json.dumps(objects)
        
        prompt = self.formatter.format_context_for_prompt(self.ctx)
        
        # Should include first 5
        for i in range(5):
            self.assertIn(f"object_{i}", prompt)
        
        # Should not include last 5
        for i in range(5, 10):
            self.assertNotIn(f"object_{i}", prompt)
    
    def test_format_context_handles_invalid_json(self):
        """Test that invalid JSON is handled gracefully"""
        self.ctx.time_context_json = "invalid json {{{}"
        self.ctx.vision_objects_json = "not valid json"
        
        # Should not raise exception
        try:
            summary = self.formatter.format_context_summary(self.ctx)
            prompt = self.formatter.format_context_for_prompt(self.ctx)
            # Should still include basic time info
            self.assertIn("15:30:00", summary)
        except:
            self.fail("Should handle invalid JSON gracefully")


if __name__ == '__main__':
    unittest.main()
