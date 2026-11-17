#!/usr/bin/env bash
# run_tests_container.sh : как гонялись тесты таблицы голосов (результат - results/pytest_v.txt).
# Тесты 3875/3931 предполагают POSIX (tmp_path как /tmp/..., mktemp, python3, LF в *.sh): на Windows (Git Bash) они падают
# RC=1 на путях C:/..., на WSL Ubuntu с Python 3.8 не импортируются (tuple[int, ...] в сигнатурах). Эталон - Linux с Python 3.11:
# одноразовый контейнер python:3.11-slim на песочнице 10.1.1.250, куда уложен срез репозитория (tests, .github, scripts/e2e,
# scripts/testing, tts_voice_registry.py, pytest.ini, conftest.py) с приведением *.sh к LF.
#
#   tar -czf vp26test.tgz pytest.ini conftest.py tests .github/workflows/scripts ".github/workflows/L-E2E Voice Test.yml" \
#       .github/e2e scripts/e2e scripts/testing src/rob_box_voice/rob_box_voice/tts_voice_registry.py
#   scp vp26test.tgz ros2@10.1.1.250:/tmp/ ; ssh ros2@10.1.1.250   # затем там:
#   mkdir -p /tmp/vp26t2 && tar -xzf /tmp/vp26test.tgz -C /tmp/vp26t2 && find /tmp/vp26t2 -name "*.sh" -exec sed -i "s/\r$//" {} +
docker run --rm -e PYTHONDONTWRITEBYTECODE=1 -v /tmp/vp26t2:/w -w /w python:3.11-slim bash -c '
  pip install -q pytest requests numpy
  python -m pytest tests/unit/e2e_scripts/test_tts_provider_selection.py \
    tests/unit/e2e_scripts/test_issue_3931_role_prosody.py \
    tests/unit/e2e_scripts/test_issue_3875_synth_retry_cache.py \
    tests/unit/test_gen_night_marathon_voice_guard.py -v -p no:cacheprovider -o addopts=""
  bash scripts/testing/test_e2e_voice_map_tts_voice.sh'
