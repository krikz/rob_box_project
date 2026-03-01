#!/usr/bin/env python3
"""
Тест эхоподавления: проверка распознавания речи во время речи робота

Этот скрипт проверяет:
1. Робот начинает говорить → STT должен отключиться
2. Во время речи робота пользователь говорит → STT должен игнорировать
3. Робот замолкает → STT включается снова
"""

import time
import subprocess
import sys

class Colors:
    """ANSI цвета для вывода"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def run_ssh_command(command: str) -> tuple[int, str]:
    """Выполнить команду на Vision Pi через SSH"""
    full_command = f"wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 '{command}'"
    result = subprocess.run(
        full_command,
        shell=True,
        capture_output=True,
        text=True
    )
    return result.returncode, result.stdout + result.stderr


def check_container_status() -> bool:
    """Проверка что voice-assistant контейнер запущен"""
    print(f"{Colors.OKBLUE}🔍 Проверка статуса voice-assistant контейнера...{Colors.ENDC}")
    returncode, output = run_ssh_command("docker ps | grep voice-assistant")
    
    if returncode == 0 and "voice-assistant" in output:
        print(f"{Colors.OKGREEN}✅ voice-assistant контейнер работает{Colors.ENDC}")
        return True
    else:
        print(f"{Colors.FAIL}❌ voice-assistant контейнер не запущен!{Colors.ENDC}")
        return False


def monitor_logs_realtime(duration: int = 30):
    """Мониторинг логов в реальном времени"""
    print(f"{Colors.OKCYAN}📊 Мониторинг логов voice-assistant (следующие {duration} секунд)...{Colors.ENDC}")
    print(f"{Colors.WARNING}⚠️  Попробуйте сказать что-нибудь роботу СЕЙЧАС{Colors.ENDC}")
    print(f"{Colors.WARNING}   Затем подождите пока робот начнёт отвечать{Colors.ENDC}")
    print(f"{Colors.WARNING}   И попробуйте говорить ПОКА робот говорит{Colors.ENDC}\n")
    
    # Запускаем docker logs в реальном времени
    full_command = f"wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 'docker logs -f voice-assistant --tail 20'"
    
    try:
        process = subprocess.Popen(
            full_command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        
        start_time = time.time()
        while time.time() - start_time < duration:
            line = process.stdout.readline()
            if not line:
                break
            
            # Подсветка важных событий
            if "🔇 Робот говорит" in line or "🔇 Игнор:" in line:
                print(f"{Colors.FAIL}{line.strip()}{Colors.ENDC}")
            elif "🎙️ Робот замолчал" in line:
                print(f"{Colors.OKGREEN}{line.strip()}{Colors.ENDC}")
            elif "✅ Yandex STT" in line or "✅ ПРИНЯТО" in line:
                print(f"{Colors.OKGREEN}{line.strip()}{Colors.ENDC}")
            elif "🔊 TTS" in line or "🔊 Воспроизведение" in line:
                print(f"{Colors.OKCYAN}{line.strip()}{Colors.ENDC}")
            elif "WARN" in line or "ERROR" in line:
                print(f"{Colors.WARNING}{line.strip()}{Colors.ENDC}")
            else:
                print(line.strip())
        
        process.terminate()
        
    except KeyboardInterrupt:
        print(f"\n{Colors.WARNING}⚠️  Мониторинг прерван пользователем{Colors.ENDC}")


def analyze_echo_cancellation():
    """Анализ логов на предмет работы эхоподавления"""
    print(f"\n{Colors.OKBLUE}🔍 Анализ последних 100 строк логов на наличие проблем...{Colors.ENDC}")
    
    returncode, output = run_ssh_command("docker logs voice-assistant --tail 100")
    
    lines = output.split('\n')
    
    # Счётчики
    robot_speaking_count = 0
    robot_stopped_count = 0
    ignored_during_speech = 0
    recognized_during_speech = 0
    
    is_robot_speaking_now = False
    
    for line in lines:
        # Робот начал говорить
        if "🔇 Робот говорит - распознавание отключено" in line:
            robot_speaking_count += 1
            is_robot_speaking_now = True
        
        # Робот замолчал
        elif "🎙️ Робот замолчал - распознавание включено" in line:
            robot_stopped_count += 1
            is_robot_speaking_now = False
        
        # Игнорирование во время речи робота
        elif "🔇 Игнор: робот говорит" in line:
            ignored_during_speech += 1
        
        # ПРОБЛЕМА: распознавание ПРОШЛО во время речи робота
        elif is_robot_speaking_now and ("✅ Yandex STT" in line or "✅ ПРИНЯТО" in line):
            recognized_during_speech += 1
            print(f"{Colors.FAIL}⚠️  ПРОБЛЕМА: Распознавание прошло во время речи робота!{Colors.ENDC}")
            print(f"   {line.strip()}")
    
    # Результаты
    print(f"\n{Colors.BOLD}📊 Результаты анализа:{Colors.ENDC}")
    print(f"   🔇 Робот начинал говорить: {robot_speaking_count} раз")
    print(f"   🎙️  Робот замолчал: {robot_stopped_count} раз")
    print(f"   ✅ Проигнорировано во время речи: {ignored_during_speech} раз")
    
    if recognized_during_speech > 0:
        print(f"{Colors.FAIL}   ❌ УТЕЧКА: Распознано во время речи: {recognized_during_speech} раз{Colors.ENDC}")
        print(f"{Colors.FAIL}      Эхоподавление НЕ РАБОТАЕТ полностью!{Colors.ENDC}")
        return False
    else:
        print(f"{Colors.OKGREEN}   ✅ Эхоподавление работает корректно!{Colors.ENDC}")
        return True


def main():
    print(f"{Colors.HEADER}{Colors.BOLD}")
    print("=" * 70)
    print("  Тест эхоподавления ReSpeaker (Echo Cancellation Test)")
    print("=" * 70)
    print(f"{Colors.ENDC}\n")
    
    # 1. Проверка контейнера
    if not check_container_status():
        print(f"\n{Colors.FAIL}❌ Тест прерван: voice-assistant не запущен{Colors.ENDC}")
        return 1
    
    print()
    
    # 2. Реалтайм мониторинг
    print(f"{Colors.BOLD}ИНСТРУКЦИЯ:{Colors.ENDC}")
    print(f"1. Сейчас начнётся мониторинг логов в реальном времени")
    print(f"2. Скажите роботу что-нибудь (например: 'Роббокс, привет')")
    print(f"3. Подождите пока робот начнёт отвечать")
    print(f"4. ПОКА РОБОТ ГОВОРИТ, попробуйте сказать ещё что-то")
    print(f"5. Проверьте, появится ли сообщение '{Colors.FAIL}🔇 Игнор: робот говорит{Colors.ENDC}'")
    print()
    
    input(f"{Colors.OKBLUE}Нажмите Enter чтобы начать мониторинг...{Colors.ENDC}")
    
    monitor_logs_realtime(duration=45)
    
    # 3. Анализ результатов
    echo_ok = analyze_echo_cancellation()
    
    print()
    print(f"{Colors.HEADER}{Colors.BOLD}")
    print("=" * 70)
    if echo_ok:
        print(f"{Colors.OKGREEN}  ✅ ТЕСТ ПРОЙДЕН: Эхоподавление работает!{Colors.ENDC}")
    else:
        print(f"{Colors.FAIL}  ❌ ТЕСТ НЕ ПРОЙДЕН: Обнаружены проблемы с эхоподавлением{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}=" * 70)
    print(f"{Colors.ENDC}\n")
    
    return 0 if echo_ok else 1


if __name__ == '__main__':
    sys.exit(main())
