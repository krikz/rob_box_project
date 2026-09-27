<#
.SYNOPSIS
    Локальный текстовый чат с диалоговой системой РОББОКСа (без ROS2).

.DESCRIPTION
    Выбирает интерпретатор (.venv в корне репозитория → python3.12+ →
    системный python), переключает консоль в UTF-8 и запускает
    scripts/dialogue/chat.py. Все аргументы прокидываются в скрипт.

.EXAMPLE
    .\scripts\dialogue\run.ps1
    .\scripts\dialogue\run.ps1 --providers deepseek --debug
    .\scripts\dialogue\run.ps1 --once "роббокс, привет" --wake-word
#>

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)

# Консоль в UTF-8: без этого кириллица из промпта и ответов уходит в
# cp866/cp1251 и печатается кракозябрами.
$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch {}

# Интерпретатор: rob_box_core использует mappingproxy как dataclass-default,
# что разрешено только с Python 3.12.
$Python = $null
$VenvPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (Test-Path $VenvPython) {
    $Python = $VenvPython
} else {
    $PyLauncher = Get-Command py -ErrorAction SilentlyContinue
    if ($PyLauncher) {
        $Python = "py"
        $PyArgs = @("-3.12")
    } else {
        $Fallback = Get-Command python -ErrorAction SilentlyContinue
        if (-not $Fallback) {
            Write-Error "Python не найден. Поставь Python 3.12+ или создай .venv в корне репозитория."
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
