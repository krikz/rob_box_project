# Animation Editor - Windows One-Liner Installer
# 
# Copy and paste this into PowerShell (run as user, not admin needed):
#
# irm https://raw.githubusercontent.com/krikz/rob_box_project/feature/voice-assistant/tools/animation_editor/install_and_run.ps1 | iex
#
# Alternative (if short URL is set up):
# irm bit.ly/robbox-animator | iex
#
# This will:
# 1. Check Python 3.8+ installation
# 2. Download Animation Editor
# 3. Install dependencies (pillow, numpy, pyyaml)
# 4. Create animations directory
# 5. Launch the editor
#
# Requirements:
# - Windows 10/11
# - Python 3.8 or higher (with pip)
# - Internet connection
#
# Manual installation:
# If the one-liner doesn't work, you can install manually:
#
# 1. Download: https://github.com/krikz/rob_box_project/archive/refs/heads/feature/voice-assistant.zip
# 2. Extract to any folder
# 3. Open PowerShell in tools/animation_editor
# 4. Run: pip install pillow numpy pyyaml
# 5. Run: python main.py --animations-dir ./animations
