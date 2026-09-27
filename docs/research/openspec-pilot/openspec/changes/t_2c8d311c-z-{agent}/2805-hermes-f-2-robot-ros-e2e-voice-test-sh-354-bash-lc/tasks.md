## 1. Implementation

- [x] 1.1 Применить фикс в `.github/workflows/scripts/e2e_voice_test.sh:355-357` — `printf %q "$@"` + `eval "$cmd"` (вариант B по ADR-0129).
- [x] 1.2 Прогнать `bash -n` на харнессе — синтаксис OK.
- [x] 1.3 Прогнать регресс-тест — 4/4 PASS.

## 2. Tests

- [x] 2.1 `scripts/agent_flow/tests/test_e2e_voice_robot_ros_quoting.sh` — 4 кейса (P1 простая команда; P2 апостроф; P3 `$(...)`; P4 смесь кавычек/пробелов/`$()`). На старом `$*`-коде 0/4 FAIL, на зафикшенном 4/4 PASS.

## 3. Documentation

- [x] 3.1 ADR-0129 (принят ранее, не в этом PR — reference).
- [x] 3.2 Комментарий в теле `robot_ros()` со ссылкой на issue и почему `$*` сломан.
- [x] 3.3 Заполнен `proposal.md` в openspec change.

## 4. Deployment

- [ ] 4.1 PR открыт от `develop` с head=`z-developer/2805-fix-robot-ros-quoting`.
- [ ] 4.2 CI зелёный (все check-runs SUCCESS).
- [ ] 4.3 Merge-gate влил PR в `develop`.
- [ ] 4.4 Issue #2805 закрыт ссылкой на merged PR.