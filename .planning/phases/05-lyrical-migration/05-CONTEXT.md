# 05-CONTEXT.md — Миграция ROS 2 Humble → Lyrical Luth

> **Дата:** 2026-07-27  
> **Статус:** Decisions captured  
> **План:** `docs/plans/lyrical-migration-plan.md`

---

## Domain Boundary

Миграция всех Docker-сервисов робота с ROS 2 Humble (Ubuntu 22.04) на ROS 2 Lyrical Luth (Ubuntu 26.04 Resolute). Включает: замену дистрибутива во всех Dockerfile'ах, `.env`, скриптах, CI/CD; решение блокера `depthai-ros`; адаптацию под Python 3.14 и системные изменения Resolute.

**Из скоупа:** проверка на реальном роботе (после успешной CI-сборки).

---

## Decisions

### D-01: Стратегия миграции
**Решение:** Humble→Lyrical напрямую от `develop`.  
**Обоснование:** Промежуточный Kilted не даёт ценности (non-LTS). Ветка `feature/kilted` (13 коммитов) — справочный материал, не источник правды.  
**Влияние:** План скорректирован с Kilted→Lyrical на Humble→Lyrical.

### D-02: DepthAI — сборка из исходников
**Решение:** Свой `Dockerfile.depthai` на базе `ros:lyrical-ros-base` + сборка `depthai-core` из `main`.  
**Обоснование:** `luxonis/depthai-ros` не имеет lyrical-тегов. Community-решений нет (проверено: GitHub, Docker Hub — ноль результатов). Ждать Luxonis — неопределённый срок.  
**Параллельно:** Создать Issue в `luxonis/depthai-ros` с запросом официальной сборки.  
**Fallback:** Временно использовать `depthai-ros:kilted-arm64-latest` (разные Ubuntu, рискованно).  
**Риски:** vcpkg может не собраться на Resolute; `depthai-core` ветка `main` может быть нестабильна.

### D-03: Zenoh — внешний пакет (архитектура НЕ меняется)
**Решение:** `ros-lyrical-rmw-zenoh-cpp` — отдельный APT-пакет (v0.10.4), НЕ встроен в base-образ.  
**Обоснование:** Проверено в `ros:lyrical-ros-base`: пакет доступен через `apt-get install`, но не предустановлен. Zenoh-router контейнеры (`docker/main/zenoh-router`, `docker/vision/zenoh-router`) сохраняются.  
**Влияние:** Пункты плана про удаление `LD_LIBRARY_PATH` и zenoh-router удалены.

### D-04: Доступность Lyrical — подтверждена
**Решение:** Миграция технически возможна.  
**Факты:**
- Ubuntu 26.04 Resolute: ✅ вышел 23.04.2026, `ports.ubuntu.com` и `archive.ubuntu.com` доступны
- `ros:lyrical-ros-base`: ✅ доступен на Docker Hub (обновлён 16.07.2026), arm64+amd64
- ROS 2 пакеты: ✅ 2154 пакета `ros-lyrical-*` доступны через apt
- `introlab3it/rtabmap_ros:lyrical-latest`: ✅ доступен (arm64+amd64)
- `luxonis/depthai-ros`: ❌ единственный блокер (см. D-02)

### D-05: Приоритет — перед Milestone 2
**Решение:** Миграция выполняется ДО начала работ по Milestone 2 (навигация: AprilTag, 3D obstacle avoidance).  
**Обоснование:** Навигационные фичи должны разрабатываться на целевом дистрибутиве. Миграция после M2 приведёт к двойной работе.  
**Размещение:** Phase 5 в роадмапе (между Milestone 1 и Milestone 2).

### D-06: База ветки — `develop` (Humble)
**Решение:** `feature/lyrical` базируется на `develop` (текущий Humble), НЕ на `feature/kilted`.  
**Обоснование:** Kilted — экспериментальная ветка, не мержена, не тестировалась на роботе.

---

## Canonical Refs

| Документ | Путь | Релевантность |
|----------|------|---------------|
| План миграции | `docs/plans/lyrical-migration-plan.md` | Основной план (скорректирован) |
| Kilted-миграция (опыт) | `origin/feature/kilted` (13 коммитов) | Reference: какие файлы менялись, какие проблемы возникали |
| Docker стандарты | `docs/development/DOCKER_STANDARDS.md` | ❌ COPY config/scripts, ✅ volumes, network_mode: host |
| Системная архитектура | `docs/architecture/SYSTEM_OVERVIEW.md` | Текущая Docker-топология |
| CI/CD Pipeline | `docs/CI_CD_PIPELINE.md` | Workflow для сборки |
| Текущие .env | `docker/main/.env`, `docker/vision/.env` | `ROS_DISTRO=humble` → заменить на `lyrical` |

---

## Deferred Ideas

- **EventsCBGExecutor / AsyncNode:** Использовать после успешной сборки (оптимизация, не блокер)
- **URDF 1.2:** Обновить модель робота (capsule geometry, quaternions) — отдельная задача
- **Bag circular recording:** Настроить после миграции
- **Удаление zenoh-router:** Отложено до появления нативной Zenoh-интеграции в ROS 2 (Lyrical всё ещё использует внешний `rmw_zenoh_cpp`)

---

## Open Questions

- **Python 3.14:** `externally-managed` поведение, совместимость `pip --break-system-packages`, версии зависимостей (numpy, openai, vosk, silero)
- **APT-cacher-ng:** Совместимость с Resolute репозиториями
- **Системные пакеты:** Какие `jammy`-пакеты переименованы/удалены в `resolute`
- **vcpkg:** Соберётся ли на Ubuntu 26.04 для depthai-core

---

*Контекст создан по результатам discuss-фазы GSD. Следующий шаг: `/gsd-plan-phase 05` для создания детального плана.*
