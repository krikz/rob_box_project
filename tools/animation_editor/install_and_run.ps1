#!/usr/bin/env pwsh
# Animation Editor - Quick Install & Run Script for Windows
# Usage: irm https://raw.githubusercontent.com/krikz/rob_box_project/feature/voice-assistant/tools/animation_editor/install_and_run.ps1 | iex

Write-Host "🎨 rob_box LED Animation Editor - Quick Installer" -ForegroundColor Cyan
Write-Host "=================================================" -ForegroundColor Cyan
Write-Host ""

# Check Python
Write-Host "⏳ Checking Python..." -ForegroundColor Yellow
try {
    $pythonVersion = python --version 2>&1
    if ($pythonVersion -match "Python (\d+)\.(\d+)") {
        $major = [int]$Matches[1]
        $minor = [int]$Matches[2]
        if ($major -ge 3 -and $minor -ge 8) {
            Write-Host "✓ Python $pythonVersion found" -ForegroundColor Green
        } else {
            Write-Host "❌ Python 3.8+ required, found $pythonVersion" -ForegroundColor Red
            Write-Host "   Download from: https://www.python.org/downloads/" -ForegroundColor Yellow
            exit 1
        }
    }
} catch {
    Write-Host "❌ Python not found!" -ForegroundColor Red
    Write-Host "   Download from: https://www.python.org/downloads/" -ForegroundColor Yellow
    Write-Host "   Make sure to check 'Add Python to PATH' during installation" -ForegroundColor Yellow
    exit 1
}

# Create temporary directory
$tempDir = Join-Path $env:TEMP "rob_box_animation_editor"
if (Test-Path $tempDir) {
    Write-Host "⏳ Cleaning old installation..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force $tempDir
}
New-Item -ItemType Directory -Path $tempDir | Out-Null

# Download repository
Write-Host "⏳ Downloading Animation Editor..." -ForegroundColor Yellow
$repoUrl = "https://github.com/krikz/rob_box_project/archive/refs/heads/feature/voice-assistant.zip"
$zipPath = Join-Path $tempDir "repo.zip"

try {
    Invoke-WebRequest -Uri $repoUrl -OutFile $zipPath -UseBasicParsing
    Write-Host "✓ Downloaded" -ForegroundColor Green
} catch {
    Write-Host "❌ Failed to download: $_" -ForegroundColor Red
    exit 1
}

# Extract
Write-Host "⏳ Extracting..." -ForegroundColor Yellow
try {
    Expand-Archive -Path $zipPath -DestinationPath $tempDir -Force
    $extractedDir = Join-Path $tempDir "rob_box_project-feature-voice-assistant"
    $editorDir = Join-Path $extractedDir "tools\animation_editor"
    Write-Host "✓ Extracted" -ForegroundColor Green
} catch {
    Write-Host "❌ Failed to extract: $_" -ForegroundColor Red
    exit 1
}

# Install dependencies
Write-Host "⏳ Installing dependencies (this may take a minute)..." -ForegroundColor Yellow
Push-Location $editorDir
try {
    python -m pip install --quiet --upgrade pip
    python -m pip install --quiet pillow numpy pyyaml
    Write-Host "✓ Dependencies installed" -ForegroundColor Green
} catch {
    Write-Host "⚠️  Warning: Some dependencies may not have installed correctly" -ForegroundColor Yellow
    Write-Host "   You can manually install with: pip install pillow numpy pyyaml" -ForegroundColor Yellow
}

# Create animations directory
$animDir = Join-Path $editorDir "animations"
if (-not (Test-Path $animDir)) {
    New-Item -ItemType Directory -Path $animDir | Out-Null
    
    # Create subdirectories
    New-Item -ItemType Directory -Path (Join-Path $animDir "manifests") | Out-Null
    New-Item -ItemType Directory -Path (Join-Path $animDir "frames") | Out-Null
}

# Launch
Write-Host ""
Write-Host "✅ Installation complete!" -ForegroundColor Green
Write-Host ""
Write-Host "🚀 Launching Animation Editor..." -ForegroundColor Cyan
Write-Host ""
Write-Host "📌 Installation location: $editorDir" -ForegroundColor Gray
Write-Host "📌 Animations directory: $animDir" -ForegroundColor Gray
Write-Host ""
Write-Host "💡 Quick Start:" -ForegroundColor Yellow
Write-Host "   1. Click 'Add Keyframe' button" -ForegroundColor White
Write-Host "   2. Enable panels (checkboxes on the right)" -ForegroundColor White
Write-Host "   3. Select color from palette (left)" -ForegroundColor White
Write-Host "   4. Click on LED matrix to draw" -ForegroundColor White
Write-Host "   5. Press Play to preview" -ForegroundColor White
Write-Host "   6. Ctrl+S to save" -ForegroundColor White
Write-Host ""
Write-Host "📖 Full documentation: docs/guides/ANIMATION_EDITOR.md" -ForegroundColor Gray
Write-Host ""

try {
    python main.py --animations-dir $animDir
} catch {
    Write-Host ""
    Write-Host "❌ Failed to launch: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "You can manually launch with:" -ForegroundColor Yellow
    Write-Host "   cd $editorDir" -ForegroundColor White
    Write-Host "   python main.py --animations-dir $animDir" -ForegroundColor White
}

Pop-Location
