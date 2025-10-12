#!/usr/bin/env python3
"""
Генерация TTS датасета для ROBBOX через DeepSeek API.

Скрипт:
1. Генерирует список вопросов для робота
2. Отправляет вопросы в DeepSeek с master prompt
3. Получает ответы робота
4. Разбивает ответы на отдельные предложения
5. Очищает от команд (<CMD:.../>)
6. Сохраняет в текстовый файл для записи голоса
"""

import os
import re
import argparse
import json
from pathlib import Path
from typing import List, Optional
import time

try:
    from openai import OpenAI
except ImportError:
    print("❌ Установите OpenAI SDK: pip install openai")
    exit(1)


class RobboxPhraseGenerator:
    """Генератор фраз для ROBBOX через DeepSeek API"""
    
    def __init__(self, api_key: str, master_prompt_path: str):
        """
        Args:
            api_key: DeepSeek API key
            master_prompt_path: Путь к master_prompt.txt
        """
        self.client = OpenAI(
            api_key=api_key,
            base_url="https://api.deepseek.com"
        )
        
        # Загружаем master prompt
        with open(master_prompt_path, 'r', encoding='utf-8') as f:
            self.master_prompt = f.read()
        
        print(f"✅ Master prompt loaded: {len(self.master_prompt)} chars")
    
    def generate_questions(self, num_questions: int = 100) -> List[str]:
        """
        Генерация вопросов для робота через DeepSeek.
        
        Args:
            num_questions: Количество вопросов для генерации
            
        Returns:
            Список вопросов
        """
        print(f"\n[1/4] Generating {num_questions} questions...")
        
        prompt = f"""Сгенерируй {num_questions} разнообразных вопросов и команд для автономного робота РОББОКС.

Категории (равномерно распределить):
1. Приветствия и диалог: "Привет", "Как дела", "Спасибо"
2. Команды движения: "Поезжай вперёд", "Остановись", "Повернись налево"
3. Запросы информации: "Какой заряд батареи", "Где ты находишься", "Что видишь"
4. Навигация: "Поезжай к окну", "Вернись на базу", "Следуй за мной"
5. Сенсоры: "Покажи карту", "Сканируй комнату", "Есть ли препятствия"
6. Эмоции и характер: "Расскажи о себе", "Что ты умеешь", "Ты устал?"
7. Безопасность: "Всё в порядке?", "Можешь проехать?", "Остановись немедленно"
8. Системные запросы: "Какая температура", "Проверь систему", "Включи свет"

Требования:
- Разнообразная длина (короткие и длинные)
- Разговорный русский язык
- Естественные формулировки
- БЕЗ нумерации
- По одному вопросу/команде на строку

Формат вывода:
Привет Роббокс
Поезжай вперёд
Какой заряд батареи
..."""

        try:
            response = self.client.chat.completions.create(
                model="deepseek-chat",
                messages=[
                    {"role": "user", "content": prompt}
                ],
                temperature=0.9,  # Высокая вариативность
                max_tokens=2000,
                stream=False
            )
            
            questions_text = response.choices[0].message.content
            
            # Парсим вопросы (по одному на строку)
            questions = [
                line.strip()
                for line in questions_text.split('\n')
                if line.strip() and not line.strip().startswith('#')
            ]
            
            # Убираем нумерацию если есть
            questions = [
                re.sub(r'^\d+[\.\)]\s*', '', q)
                for q in questions
            ]
            
            print(f"✅ Generated {len(questions)} questions")
            return questions[:num_questions]
            
        except Exception as e:
            print(f"❌ Error generating questions: {e}")
            return []
    
    def get_robot_response(self, question: str) -> Optional[str]:
        """
        Получить ответ робота на вопрос через DeepSeek.
        
        Args:
            question: Вопрос пользователя
            
        Returns:
            Ответ робота или None при ошибке
        """
        try:
            response = self.client.chat.completions.create(
                model="deepseek-chat",
                messages=[
                    {"role": "system", "content": self.master_prompt},
                    {"role": "user", "content": question}
                ],
                temperature=0.7,
                max_tokens=300,
                stream=False
            )
            
            answer = response.choices[0].message.content
            return answer
            
        except Exception as e:
            print(f"⚠️  Error for question '{question}': {e}")
            return None
    
    def generate_responses(self, questions: List[str], delay: float = 1.0) -> List[tuple]:
        """
        Генерация ответов робота на все вопросы.
        
        Args:
            questions: Список вопросов
            delay: Задержка между запросами (секунды)
            
        Returns:
            Список кортежей (вопрос, ответ)
        """
        print(f"\n[2/4] Generating responses for {len(questions)} questions...")
        print(f"This will take ~{len(questions) * delay / 60:.1f} minutes")
        
        qa_pairs = []
        
        for i, question in enumerate(questions, 1):
            print(f"\r[{i}/{len(questions)}] Processing: {question[:50]}...", end='', flush=True)
            
            answer = self.get_robot_response(question)
            
            if answer:
                qa_pairs.append((question, answer))
            
            # Задержка между запросами
            if i < len(questions):
                time.sleep(delay)
        
        print(f"\n✅ Generated {len(qa_pairs)} responses")
        return qa_pairs
    
    def clean_text(self, text: str) -> str:
        """
        Очистка текста от команд и лишних символов.
        
        Args:
            text: Исходный текст
            
        Returns:
            Очищенный текст
        """
        # Убираем команды <CMD:.../>
        text = re.sub(r'<CMD:[^>]+/>', '', text)
        
        # Убираем множественные пробелы
        text = re.sub(r'\s+', ' ', text)
        
        # Убираем пробелы перед знаками препинания
        text = re.sub(r'\s+([.,!?])', r'\1', text)
        
        return text.strip()
    
    def split_into_sentences(self, text: str) -> List[str]:
        """
        Разбивка текста на предложения.
        
        Args:
            text: Текст для разбивки
            
        Returns:
            Список предложений
        """
        # Сначала очищаем
        text = self.clean_text(text)
        
        # Разбиваем по знакам препинания
        sentences = re.split(r'(?<=[.!?])\s+', text)
        
        # Фильтруем короткие и пустые
        sentences = [s.strip() for s in sentences if len(s.strip()) > 3]
        
        return sentences
    
    def generate_dataset(self, qa_pairs: List[tuple]) -> List[str]:
        """
        Генерация финального датасета из пар вопрос-ответ.
        
        Args:
            qa_pairs: Список кортежей (вопрос, ответ)
            
        Returns:
            Список предложений для TTS
        """
        print(f"\n[3/4] Processing {len(qa_pairs)} Q&A pairs...")
        
        all_sentences = []
        
        for question, answer in qa_pairs:
            # Разбиваем ответ на предложения
            sentences = self.split_into_sentences(answer)
            all_sentences.extend(sentences)
        
        # Убираем дубликаты (сохраняя порядок)
        unique_sentences = []
        seen = set()
        
        for sentence in all_sentences:
            sentence_lower = sentence.lower()
            if sentence_lower not in seen:
                seen.add(sentence_lower)
                unique_sentences.append(sentence)
        
        print(f"✅ Generated {len(unique_sentences)} unique sentences")
        return unique_sentences
    
    def save_dataset(self, sentences: List[str], output_file: str):
        """
        Сохранение датасета в файл.
        
        Args:
            sentences: Список предложений
            output_file: Путь к выходному файлу
        """
        print(f"\n[4/4] Saving dataset to {output_file}...")
        
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            for sentence in sentences:
                f.write(sentence + '\n')
        
        print(f"✅ Dataset saved: {len(sentences)} sentences")
        
        # Статистика
        total_chars = sum(len(s) for s in sentences)
        avg_length = total_chars / len(sentences) if sentences else 0
        estimated_duration = len(sentences) * 3  # ~3 секунды на фразу
        
        print(f"\nDataset statistics:")
        print(f"  Total sentences: {len(sentences)}")
        print(f"  Average length: {avg_length:.1f} chars")
        print(f"  Estimated audio: {estimated_duration / 60:.1f} minutes")
        print(f"  Training approach: ", end='')
        
        if estimated_duration < 600:  # < 10 минут
            print("Voice cloning (fast, quality 3-4/5)")
        elif estimated_duration < 1800:  # < 30 минут
            print("Fine-tuning (balanced, quality 4/5) ⭐")
        else:
            print("Full training (slow, quality 4-5/5)")


def main():
    parser = argparse.ArgumentParser(
        description="Generate TTS dataset for ROBBOX using DeepSeek API",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:

  # Generate 200 questions (recommended for fine-tuning)
  python3 generate_phrases_deepseek.py \\
    --api-key sk-xxx \\
    --master-prompt ../prompts/master_prompt.txt \\
    --output expanded_sentences_deepseek.txt \\
    --num-questions 200

  # Generate 1000 questions for full training
  python3 generate_phrases_deepseek.py \\
    --api-key sk-xxx \\
    --master-prompt ../prompts/master_prompt.txt \\
    --output full_dataset_deepseek.txt \\
    --num-questions 1000

  # Save Q&A pairs for reference
  python3 generate_phrases_deepseek.py \\
    --api-key sk-xxx \\
    --master-prompt ../prompts/master_prompt.txt \\
    --output dataset.txt \\
    --save-qa qa_pairs.json

Environment variables:
  DEEPSEEK_API_KEY: API key for DeepSeek

Get API key:
  https://platform.deepseek.com/api_keys
        """
    )
    
    parser.add_argument(
        "--api-key",
        help="DeepSeek API key (or set DEEPSEEK_API_KEY env variable)"
    )
    
    parser.add_argument(
        "--master-prompt",
        default="../prompts/master_prompt.txt",
        help="Path to master_prompt.txt"
    )
    
    parser.add_argument(
        "--output",
        required=True,
        help="Output file for sentences"
    )
    
    parser.add_argument(
        "--num-questions",
        type=int,
        default=200,
        help="Number of questions to generate (default: 200)"
    )
    
    parser.add_argument(
        "--delay",
        type=float,
        default=1.0,
        help="Delay between API requests in seconds (default: 1.0)"
    )
    
    parser.add_argument(
        "--save-qa",
        help="Save Q&A pairs to JSON file for reference"
    )
    
    args = parser.parse_args()
    
    # Получаем API key
    api_key = args.api_key or os.getenv("DEEPSEEK_API_KEY")
    if not api_key:
        print("❌ DeepSeek API key not provided!")
        print("\nOptions:")
        print("  1. Use --api-key argument")
        print("  2. Set DEEPSEEK_API_KEY environment variable")
        print("\nGet API key: https://platform.deepseek.com/api_keys")
        return 1
    
    # Проверяем master prompt
    master_prompt_path = Path(args.master_prompt)
    if not master_prompt_path.exists():
        print(f"❌ Master prompt not found: {master_prompt_path}")
        print("\nExpected location: src/rob_box_voice/prompts/master_prompt.txt")
        return 1
    
    print("=" * 60)
    print("ROBBOX PHRASE GENERATOR - DeepSeek")
    print("=" * 60)
    print(f"Master prompt: {master_prompt_path}")
    print(f"Questions: {args.num_questions}")
    print(f"Output: {args.output}")
    print("=" * 60)
    print()
    
    # Создаём генератор
    generator = RobboxPhraseGenerator(api_key, str(master_prompt_path))
    
    # Генерируем вопросы
    questions = generator.generate_questions(args.num_questions)
    
    if not questions:
        print("❌ Failed to generate questions")
        return 1
    
    # Генерируем ответы
    qa_pairs = generator.generate_responses(questions, args.delay)
    
    if not qa_pairs:
        print("❌ Failed to generate responses")
        return 1
    
    # Сохраняем Q&A пары если нужно
    if args.save_qa:
        qa_path = Path(args.save_qa)
        qa_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(qa_path, 'w', encoding='utf-8') as f:
            json.dump(
                [{"question": q, "answer": a} for q, a in qa_pairs],
                f,
                ensure_ascii=False,
                indent=2
            )
        print(f"✅ Q&A pairs saved to: {qa_path}")
    
    # Генерируем датасет
    sentences = generator.generate_dataset(qa_pairs)
    
    if not sentences:
        print("❌ Failed to generate dataset")
        return 1
    
    # Сохраняем датасет
    generator.save_dataset(sentences, args.output)
    
    print("\n" + "=" * 60)
    print("✅ Dataset generation completed!")
    print("=" * 60)
    print("\nNext steps:")
    print(f"  1. Review generated sentences: cat {args.output}")
    print(f"  2. Record voice with Yandex:")
    print(f"     python3 record_yandex_voice.py --input {args.output} --output ~/robbox_tts_training/datasets/robbox_voice/")
    print(f"  3. Train model:")
    print(f"     python3 ../training/train_piper.py --dataset ~/robbox_tts_training/datasets/robbox_voice --output ~/robbox_tts_training/models/robbox_piper")
    
    return 0


if __name__ == "__main__":
    exit(main())
