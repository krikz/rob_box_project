# Архив старых Zenoh исправлений (PR #177-182)

Эта директория содержит документацию предыдущих попыток исправления Zenoh transport errors (2025-11-10).

⚠️ **ВНИМАНИЕ:** Эти исправления были заменены более эффективным решением в [ZENOH_PORT_CONFLICT_FIX_2025-11-10.md](../ZENOH_PORT_CONFLICT_FIX_2025-11-10.md)

---

## 📚 Исторические документы

### PR #177 - Ethernet Interface Fix

- **[ZENOH_ETHERNET_INTERFACE_FIX_2025-11-10.md](ZENOH_ETHERNET_INTERFACE_FIX_2025-11-10.md)** - Полная документация
- **[ZENOH_ETHERNET_QUICKFIX.md](ZENOH_ETHERNET_QUICKFIX.md)** - Краткий справочник

**Решение:** `#iface=eth0` для принудительной маршрутизации через Ethernet  
**Статус:** ⚠️ Частично эффективно

### PR #179 - Router Connection Fix (не применялся)

- **[ZENOH_FIX_2025-11-10_DEPLOYMENT.md](ZENOH_FIX_2025-11-10_DEPLOYMENT.md)** - Deployment guide
- **[ZENOH_FIX_2025-11-10_MAXIMUM.md](ZENOH_FIX_2025-11-10_MAXIMUM.md)** - Максимальные исправления

**Решение:** Пустые `connect.endpoints` для роутеров  
**Статус:** ❌ Закрыт без слияния

### PR #180 - Localhost Listen Fix

- **[ZENOH_FIX_QUICKSTART.md](ZENOH_FIX_QUICKSTART.md)** - Quick start guide

**Решение:** Добавлен `tcp/localhost:7447` в listen endpoints  
**Статус:** ⚠️ Частично эффективно

### PR #182 - TX Buffer Увеличение

- **[ZENOH_TRANSPORT_FIX_QUICKREF.md](ZENOH_TRANSPORT_FIX_QUICKREF.md)** - Quick reference

**Решение:** TX queue sizes до максимума (16 batches), wait_before_close до 60s  
**Статус:** ⚠️ Частично эффективно

---

## ✅ Текущее решение

**Используйте вместо этих документов:**

- **[ZENOH_PORT_CONFLICT_FIX_2025-11-10.md](../ZENOH_PORT_CONFLICT_FIX_2025-11-10.md)** - Полное исправление
- **[ZENOH_PORT_CONFLICT_QUICKFIX.md](../ZENOH_PORT_CONFLICT_QUICKFIX.md)** - Быстрое развёртывание

**Решение:** Конкретные IP адреса вместо wildcard endpoints  
**Статус:** ✅ ИСПРАВЛЕНО - готово к развёртыванию

---

## 🔍 Зачем хранятся эти документы?

1. **История исследований** - показывает процесс поиска решения
2. **Уроки на будущее** - что НЕ работает и почему
3. **Контекст для анализа** - полезно при появлении похожих проблем
4. **Документация процесса** - демонстрация iterative debugging

---

**См. также:** [ZENOH_FIXES_INDEX.md](../ZENOH_FIXES_INDEX.md) - полный индекс всех Zenoh исправлений
