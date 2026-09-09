## Phase 1.7 — e2e smoke по локальной сети (ручная верификация + автоматизированные проверки)

**Parent / milestone:** Phase 1 (MVP telepresence), см. `docs/plans/2026-08-24-meta-quest-telepresence.md` §Phase 1.7.

**Блок-зависимость:** Phase 1.7 зависит от Phase 1.6 (issue #1641 — Caddy + Docker-сервис + self-signed cert) — нужен развернутый `docker compose up quest` на Vision Pi.

**Контекст:**
- Phase 1.1–1.5 готовы (см. `docs/plans/2026-08-24-meta-quest-telepresence.md` + issue #1639 merged)
- DNS `quest.rob_box.local → 10.1.1.11` уже работает в LAN 192.168.2.0/24 (роутер FriendlyWrt, 25.08.2026) ✅
- Handshake-протокол: см. `docs/architecture/meta-quest-api.md` §1–§7
- Heartbeat: 200 мс (server→client), watchdog 600 мс (3× heartbeat timeout)
- Endpoints: `wss://quest.rob_box.local:8443/quest`, `https://quest.rob_box.local:8443/healthz`

### Что входит в эту карточку

1. **Issue-карточка** с DoD-чеклистом и raw-evidence-местами ✅ (этот issue)
2. **Smoke-скрипт** `local_test/quest_smoke.sh` — автоматизируемые проверки на dev-машине:
   - DNS resolve (`dig/nslookup quest.rob_box.local`)
   - TLS cert info (subject, SAN, expiry)
   - HTTP `/healthz` (status ok, sessions_active, version)
   - WSS handshake HELLO/WELCOME + замер round-trip
   - heartbeat interval measurement (должен быть ~200 мс ±50)
   - SUBSCRIBE → BINARY_FRAME JPEG latency (end-to-end)
   - GOODBYE → server-side close (watchdog < 600 мс при потере ping)
3. **Документация** `docs/guides/quest-e2e-smoke.md` — ручной чеклист для Шифу:
   - Импорт сертификата в Quest (Settings → Privacy → Security → Trusted Sources) — однократно
   - Открыть `https://quest.rob_box.local` в десктоп-браузере, ввести PIN
   - Виден стрим камеры (замерить latency: фото таймера → экран), лидар-оверлей
   - Teleop: стик Quest двигает робота; отпустить grip → робот останавливается ≤ 100 мс
   - Обрыв Wi-Fi (отключить Wi-Fi на Quest/dev-машине) → робот safe-stop ≤ 500 мс, UI «CONNECTION LOST»
   - Скриншоты Quest-интерфейса (4 panels полукругом)

### Out of scope (Phase 2+)

- H.264 (замена JPEG если latency > 200 мс)
- Multi-user / mTLS / TOTP
- Spatial audio
- Spatial mapping (ходьба в AR)

### Definition of Done

- [ ] Issue создан ✅
- [ ] `local_test/quest_smoke.sh` написан и проходит автоматизируемые проверки (`--check-only` режим на dev-машине без развернутого сервера)
- [ ] `docs/guides/quest-e2e-smoke.md` написан с ручным чеклистом
- [ ] Commit + push на `feature/quest-phase-1.7`
- [ ] После деплоя Phase 1.6 — прогнать smoke-скрипт на dev-машине, приложить raw-вывод (логи, latency)
- [ ] Ручная верификация на Quest + роботе — Шифу делает, raw-evidence (скриншоты, замеры latency, видео grip release, видео safe-stop)
- [ ] PR создан, дожидается merge от Шифу

### Acceptance (raw-evidence обязателен, ADR-0018)

- [ ] `bash local_test/quest_smoke.sh --all` output (DNS + healthz + WSS handshake + heartbeat interval + JPEG latency)
- [ ] `docker logs quest` последние 50 строк
- [ ] Скриншоты Quest-сцены (4 panels + LiDAR overlay)
- [ ] Замер latency: фото таймера → экран ≤ 200 мс
- [ ] Замер safe-stop: grip отпущен → робот остановился ≤ 100 мс
- [ ] Замер safe-stop: Wi-Fi обрыв → робот остановился ≤ 500 мс

### Следующий шаг по процессу (agent-flow)

1. `scripts/agent_flow/agent-flow-triage.sh` подхватит label `hermes` → назначит worker'а
2. Worker создаёт ветку `feature/quest-phase-1.7` через `gh issue develop <this> --checkout`
3. Реализация по плану в карточке (~250 строк: smoke-скрипт + docs)
4. PR с raw-evidence (ADR-0018)
5. После merge → ручная верификация Шифу → закрытие MVP Phase 1