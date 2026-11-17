<#
.SYNOPSIS
    Консоль диалога РОББОКСа на ходе v2 (сухой стенд, без ROS2 и без робота).

.DESCRIPTION
    Выбирает интерпретатор (.venv в корне репозитория → новейший python3
    через py-лаунчер → системный python), переключает консоль в UTF-8 и запускает
    scripts/dialogue/chat.py. Все аргументы прокидываются в скрипт.

.EXAMPLE
    .\scripts\dialogue\run.ps1
    .\scripts\dialogue\run.ps1 --providers deepseek --debug
    .\scripts\dialogue\run.ps1 --once "Сколько у тебя заряда?" --once "Робот, стоп"
    .\scripts\dialogue\run.ps1 --db $HOME\.rob_box\console_memory.e2e.db --variant B
#>

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)

# Консоль в UTF-8: без этого кириллица из промпта и ответов уходит в
# cp866/cp1251 и печатается кракозябрами.
$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch {}

# Интерпретатор: ядру нужен Python 3.10+ (packaging/dialog-core, requires-python).
# Замер #4245: тесты ядра из пакета зелёные на 3.11.0 и 3.12.6, робот - 3.10.
$Python = $null
$VenvPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (Test-Path $VenvPython) {
    $Python = $VenvPython
} else {
    $PyLauncher = Get-Command py -ErrorAction SilentlyContinue
    if ($PyLauncher) {
        $Python = "py"
        $PyArgs = @("-3")
    } else {
        $Fallback = Get-Command python -ErrorAction SilentlyContinue
        if (-not $Fallback) {
            Write-Error "Python не найден. Поставь Python 3.10+ или создай .venv в корне репозитория."
            exit 1
        }
        $Python = $Fallback.Source
    }
}

$AllArgs = @()
if ($PyArgs) { $AllArgs += $PyArgs }
$AllArgs += (Join-Path $ScriptDir "chat.py")
$AllArgs += $args

& $Python @AllArgs
exit $LASTEXITCODE
