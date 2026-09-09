#!/usr/bin/env python3
"""
Тест streaming чата с только SSML (без поля text)
"""

import os
import sys
from robbox_chat_streaming import StreamingChatBot

def main():
    # Загружаем API ключ
    api_key = os.getenv('DEEPSEEK_API_KEY')
    if not api_key:
        print("❌ DEEPSEEK_API_KEY не найден")
        print("Запустите: set -a && source ../.env.secrets && set +a")
        sys.exit(1)
    
    # Создаём бота
    bot = StreamingChatBot()
    
    # Тестовые запросы
    test_queries = [
        "Привет! Кратко представься",
        "Расскажи про город Сочи кратко",
        "Теорема Пифагора с формулой"
    ]
    
    for i, query in enumerate(test_queries, 1):
        print(f"\n{'='*70}")
        print(f"ТЕСТ {i}/{len(test_queries)}: {query}")
        print('='*70)
        
        bot.ask_deepseek_streaming(query)
        
        input("\n⏸️  Нажмите Enter для следующего теста...")
    
    print("\n✅ Все тесты завершены!")

if __name__ == "__main__":
    main()
