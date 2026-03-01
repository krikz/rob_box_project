"""
Prompt Formatter for Reflection Node

This module handles formatting perception context data into prompts for LLM consumption.
Provides both detailed context formatting and concise summary formatting.
"""

import json
from typing import Any, List


class PromptFormatter:
    """Formats perception context into LLM prompts"""
    
    def format_context_summary(self, ctx: Any) -> str:
        """
        Format concise context summary (for urgent responses)
        
        Args:
            ctx: PerceptionEvent message with current context
            
        Returns:
            Formatted summary string
        """
        lines = ["=== ТЕКУЩЕЕ СОСТОЯНИЕ ==="]
        
        # Time (if available)
        if hasattr(ctx, 'current_time_human') and ctx.current_time_human:
            try:
                time_data = json.loads(ctx.time_context_json)
                lines.append(f"🕐 Сейчас: {time_data.get('period_ru', ctx.time_period)}, {ctx.current_time_human}")
            except:
                lines.append(f"🕐 Время: {ctx.current_time_human}")
        
        # Health
        lines.append(f"Здоровье: {ctx.system_health_status}")
        if ctx.health_issues:
            lines.append(f"Проблемы: {', '.join(ctx.health_issues)}")
        
        # Internet status
        if hasattr(ctx, 'internet_available'):
            internet_status = "доступен" if ctx.internet_available else "недоступен"
            lines.append(f"🌐 Интернет: {internet_status}")
        
        # Battery
        lines.append(f"Батарея: {ctx.battery_percent:.0f}%")
        if ctx.is_charging:
            lines.append("⚡ Заряжается")
        
        # Disk space
        if hasattr(ctx, 'disk_usage_percent') and ctx.disk_usage_percent > 0:
            lines.append(f"💾 Диск: {ctx.disk_usage_percent:.0f}%")
        
        # Obstacle
        if ctx.has_obstacle_ahead:
            lines.append(f"⚠️ Препятствие: {ctx.obstacle_distance_meters:.1f}м")
        
        # Position/navigation
        if hasattr(ctx, 'current_map_name') and ctx.current_map_name:
            lines.append(f"🗺️  Карта: {ctx.current_map_name}")
        
        if hasattr(ctx, 'nav_goal_description') and ctx.nav_goal_description:
            lines.append(f"🎯 Цель: {ctx.nav_goal_description}")
        
        # AprilTags
        if ctx.apriltag_count > 0:
            lines.append(f"🏷️ Обнаружено AprilTag: {ctx.apriltag_count}")
        
        # Vision
        if ctx.has_vision and hasattr(ctx, 'vision_summary') and ctx.vision_summary:
            lines.append(f"👁️ Вижу: {ctx.vision_summary[:100]}")
        
        return "\n".join(lines)
    
    def format_context_for_prompt(self, ctx: Any, recent_thoughts: List[str] = None) -> str:
        """
        Format detailed context for full prompts (reflection)
        
        Args:
            ctx: PerceptionEvent message with current context
            recent_thoughts: List of recent internal thoughts (optional)
            
        Returns:
            Formatted detailed context string
        """
        lines = []
        
        # ============ Time Context ============
        lines.append("=== ВРЕМЯ ===")
        if hasattr(ctx, 'current_time_human') and ctx.current_time_human:
            try:
                time_data = json.loads(ctx.time_context_json)
                lines.append(f"Текущее время: {ctx.current_time_human}")
                lines.append(f"Период дня: {time_data.get('period_ru', ctx.time_period)}")
                if time_data.get('is_special'):
                    lines.append(f"🌟 Особое время: {time_data.get('special_description', '')}")
            except:
                lines.append(f"Время: {ctx.current_time_human}")
        lines.append("")
        
        # ============ System Health ============
        lines.append("=== ЗДОРОВЬЕ СИСТЕМЫ ===")
        lines.append(f"Статус: {ctx.system_health_status}")
        if ctx.health_issues:
            lines.append("Проблемы:")
            for issue in ctx.health_issues:
                lines.append(f"  - {issue}")
        else:
            lines.append("Проблем не обнаружено")
        
        # Internet status
        if hasattr(ctx, 'internet_available'):
            internet_status = "доступен ✅" if ctx.internet_available else "недоступен ❌"
            lines.append(f"Интернет: {internet_status}")
        lines.append("")
        
        # ============ Battery ============
        lines.append("=== БАТАРЕЯ ===")
        lines.append(f"Уровень: {ctx.battery_percent:.0f}%")
        lines.append(f"Статус: {'Заряжается ⚡' if ctx.is_charging else 'Разряжается 🔋'}")
        lines.append("")
        
        # ============ Disk Space ============
        if hasattr(ctx, 'disk_usage_percent') and ctx.disk_usage_percent > 0:
            lines.append("=== ДИСК ===")
            lines.append(f"Использовано: {ctx.disk_usage_percent:.0f}%")
            if ctx.disk_usage_percent > 80:
                lines.append("⚠️ Мало свободного места!")
            lines.append("")
        
        # ============ Sensors ============
        lines.append("=== СЕНСОРЫ ===")
        if ctx.has_obstacle_ahead:
            lines.append(f"⚠️ ПРЕПЯТСТВИЕ ВПЕРЕДИ: {ctx.obstacle_distance_meters:.1f}м")
        else:
            lines.append("Путь свободен")
        lines.append("")
        
        # ============ Navigation ============
        lines.append("=== НАВИГАЦИЯ ===")
        if hasattr(ctx, 'current_map_name') and ctx.current_map_name:
            lines.append(f"Карта: {ctx.current_map_name}")
        if hasattr(ctx, 'nav_goal_description') and ctx.nav_goal_description:
            lines.append(f"Цель: {ctx.nav_goal_description}")
        if hasattr(ctx, 'nav_status') and ctx.nav_status:
            lines.append(f"Статус: {ctx.nav_status}")
        lines.append("")
        
        # ============ AprilTags ============
        if ctx.apriltag_count > 0:
            lines.append("=== APRILTAG ===")
            lines.append(f"Обнаружено меток: {ctx.apriltag_count}")
            if ctx.apriltag_summary:
                lines.append(f"Описание: {ctx.apriltag_summary}")
            lines.append("")
        
        # ============ Vision ============
        if ctx.has_vision:
            lines.append("=== КАМЕРА (OAK-D) ===")
            if hasattr(ctx, 'vision_summary') and ctx.vision_summary:
                lines.append(f"Вижу: {ctx.vision_summary}")
            if hasattr(ctx, 'vision_objects_json') and ctx.vision_objects_json:
                try:
                    objects = json.loads(ctx.vision_objects_json)
                    if objects:
                        lines.append("Объекты:")
                        for obj in objects[:5]:  # Top 5 objects
                            lines.append(f"  - {obj.get('label', 'Unknown')} ({obj.get('confidence', 0):.0%})")
                except:
                    pass
            lines.append("")
        
        # ============ Memory (Recent Thoughts) ============
        if recent_thoughts:
            lines.append("=== МОИ МЫСЛИ (последние) ===")
            for thought in recent_thoughts[-3:]:  # Last 3 thoughts
                lines.append(f"- {thought}")
            lines.append("")
        
        return "\n".join(lines)
