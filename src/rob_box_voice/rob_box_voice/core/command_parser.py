#!/usr/bin/env python3
"""
command_parser.py - Voice Command Parsing and Intent Classification

Purpose:
    Parses voice commands, classifies intents, and extracts entities
    for robot control without ROS dependencies.

Public Interface:
    - IntentType: Enum for command intent types
    - Command: Data class for parsed commands
    - CommandParser: Main class for command parsing

Example:
    >>> from rob_box_voice.core.command_parser import CommandParser
    >>> parser = CommandParser()
    >>> command = parser.parse('иди к кухне')
    >>> print(command.intent, command.entities)
    IntentType.NAVIGATE {'waypoint': 'кухня'}

Tests:
    See: test/unit/core/test_command_parser.py

Dependencies:
    - re (standard library)
    - typing (standard library)
    - dataclasses (standard library)
    - enum (standard library)
"""

import re
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class IntentType(Enum):
    """Command intent types"""
    NAVIGATE = "navigate"       # Navigation to waypoint
    STOP = "stop"              # Stop movement
    FOLLOW = "follow"          # Follow mode
    STATUS = "status"          # Status request
    MAP = "map"                # Map operations
    VISION = "vision"          # Vision/detection
    UNKNOWN = "unknown"        # Unknown command


@dataclass
class Command:
    """
    Parsed command with intent and entities.
    
    Attributes:
        intent: Classified intent type
        text: Original command text
        entities: Extracted entities (e.g., {'waypoint': 'кухня'})
        confidence: Confidence score (0.0-1.0)
    """
    intent: IntentType
    text: str
    entities: Dict[str, any]
    confidence: float


class CommandParser:
    """
    Parses voice commands and extracts intent + entities.
    
    This class uses pattern matching to classify command intents
    and extract relevant entities (waypoints, directions, etc.)
    
    Attributes:
        patterns: Dictionary mapping IntentType to pattern list
        wake_words: List of wake words to remove
        confidence_base: Base confidence for pattern match
    
    Methods:
        parse: Parse command text and return Command object
        classify_intent: Classify intent and extract entities
        remove_wake_word: Remove wake word from text
        add_pattern: Add custom pattern for intent
        get_patterns: Get patterns for specific intent
    """
    
    def __init__(
        self,
        wake_words: Optional[List[str]] = None,
        confidence_base: float = 0.8
    ):
        """
        Initialize CommandParser.
        
        Args:
            wake_words: List of wake words to remove (default: ['робот', 'робокс'])
            confidence_base: Base confidence score for matches (default: 0.8)
        """
        self.wake_words = wake_words or ['робот', 'робокс', 'робобокс']
        self.confidence_base = confidence_base
        self.patterns = self._build_default_patterns()
    
    def _build_default_patterns(self) -> Dict[IntentType, List[Tuple[str, Optional[str]]]]:
        """
        Build default command patterns.
        
        Returns:
            Dictionary mapping IntentType to list of (pattern, entity_type) tuples
        """
        return {
            # Navigation patterns
            IntentType.NAVIGATE: [
                (r'(двигайся|иди|поезжай|езжай|направляйся)\s+к\s+точке\s+(\d+)', 'waypoint_number'),
                (r'(двигайся|иди|поезжай|езжай)\s+к\s+(дом|кухня|гостиная)', 'waypoint_name'),
                (r'(двигайся|иди|поезжай|езжай|катись|двигай)\s+(вперед|вперёд|назад|влево|вправо)', 'direction'),
                (r'^(вперед|вперёд|назад)$', 'direction'),
                (r'(поверни|повернись|разверн|развернись)\s+(налево|направо|влево|вправо)', 'turn'),
                (r'^(налево|направо|влево|вправо)$', 'turn'),
            ],
            # Stop patterns
            IntentType.STOP: [
                (r'(стой|стоп|остановись|останови|halt|стоять|хватит|замри)', None),
                (r'(отмени|cancel)\s+(движение|навигацию|задание)', None),
            ],
            # Follow patterns
            IntentType.FOLLOW: [
                (r'(следуй|иди)\s+за\s+(мной|человеком)', None),
                (r'(включи|активируй)\s+режим\s+следования', None),
            ],
            # Status patterns
            IntentType.STATUS: [
                (r'(где|куда)\s+(ты|робот)', None),
                (r'(покажи|расскажи)\s+(статус|положение|координаты)', None),
            ],
            # Map patterns
            IntentType.MAP: [
                (r'(покажи|открой|загрузи)\s+карту', None),
                (r'(создай|построй|сделай)\s+карту', None),
            ],
            # Vision patterns
            IntentType.VISION: [
                (r'что\s+(видишь|перед\s+тобой)', None),
                (r'(найди|покажи|обнаружь)\s+(объект|человека|предмет)', None),
            ],
        }
    
    def parse(self, text: str) -> Command:
        """
        Parse command text.
        
        Main entry point for command parsing. Removes wake words,
        classifies intent, and extracts entities.
        
        Args:
            text: Command text to parse
        
        Returns:
            Parsed Command object
        """
        # Normalize text
        text = text.strip().lower()
        
        # Remove wake word
        text = self.remove_wake_word(text)
        
        # Classify intent and extract entities
        return self.classify_intent(text)
    
    def remove_wake_word(self, text: str) -> str:
        """
        Remove wake word from beginning of text.
        
        Args:
            text: Text possibly starting with wake word
        
        Returns:
            Text with wake word removed
        """
        for wake_word in self.wake_words:
            if text.startswith(wake_word):
                text = text[len(wake_word):].strip()
                # Remove common punctuation after wake word
                text = text.lstrip(',.:;!')
                break
        
        return text.strip()
    
    def classify_intent(self, text: str) -> Command:
        """
        Classify command intent and extract entities.
        
        Uses pattern matching to find best matching intent
        and extracts relevant entities from the command.
        
        Args:
            text: Normalized command text
        
        Returns:
            Command object with intent and entities
        """
        best_match = None
        best_confidence = 0.0
        best_intent = IntentType.UNKNOWN
        best_entities = {}
        
        # Check all patterns
        for intent, patterns in self.patterns.items():
            for pattern, entity_type in patterns:
                match = re.search(pattern, text, re.IGNORECASE)
                if match:
                    # Calculate confidence based on match coverage
                    match_length = len(match.group(0))
                    text_length = len(text) if len(text) > 0 else 1
                    confidence = self.confidence_base + (match_length / text_length) * 0.2
                    confidence = min(confidence, 1.0)  # Cap at 1.0
                    
                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_intent = intent
                        best_match = match
                        best_entities = self._extract_entities(match, entity_type)
        
        return Command(
            intent=best_intent,
            text=text,
            entities=best_entities,
            confidence=best_confidence
        )
    
    def _extract_entities(self, match: re.Match, entity_type: Optional[str]) -> Dict[str, any]:
        """
        Extract entities from regex match.
        
        Args:
            match: Regex match object
            entity_type: Type of entity to extract
        
        Returns:
            Dictionary of extracted entities
        """
        entities = {}
        
        if entity_type == 'waypoint_number':
            # Extract waypoint number (group 2)
            entities = {'waypoint': f"точка {match.group(2)}"}
        
        elif entity_type == 'waypoint_name':
            # Extract waypoint name (group 2)
            entities = {'waypoint': match.group(2)}
        
        elif entity_type in ('direction', 'turn'):
            # Direction can be in group 1 (alone) or group 2 (with verb)
            if match.lastindex and match.lastindex >= 2:
                direction = match.group(2)
            else:
                direction = match.group(1)
            entities = {'direction': direction}
        
        return entities
    
    def add_pattern(self, intent: IntentType, pattern: str, entity_type: Optional[str] = None):
        """
        Add custom pattern for intent.
        
        Args:
            intent: Intent type to add pattern for
            pattern: Regex pattern string
            entity_type: Optional entity type for extraction
        """
        if intent not in self.patterns:
            self.patterns[intent] = []
        
        self.patterns[intent].append((pattern, entity_type))
    
    def get_patterns(self, intent: IntentType) -> List[Tuple[str, Optional[str]]]:
        """
        Get patterns for specific intent.
        
        Args:
            intent: Intent type
        
        Returns:
            List of (pattern, entity_type) tuples
        """
        return self.patterns.get(intent, [])


# Example usage
if __name__ == '__main__':
    # Create parser
    parser = CommandParser()
    
    print("🧪 Command parsing tests:")
    
    test_cases = [
        "робот иди к кухне",
        "поезжай к точке 2",
        "вперед",
        "стоп",
        "где ты",
        "неизвестная команда",
    ]
    
    for text in test_cases:
        command = parser.parse(text)
        print(f"\nТекст: {text}")
        print(f"Intent: {command.intent.value}")
        print(f"Entities: {command.entities}")
        print(f"Confidence: {command.confidence:.2f}")
