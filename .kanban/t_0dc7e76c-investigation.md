# t_0dc7e76c — investigation notes (WIP)

> Status: WIP — pre-implementation reconnaissance (ADR-0018 raw-evidence).
> Author: devops (Hermes Agent), 25.08.2026, branch `z-{agent}/1630-...`

## 1. Issue #1630 — текущее состояние (что уже сделано)

| Fix | PR | Commit | Что решает | Что НЕ решает |
|---|---|---|---|---|
| Per-service ci/image-versions-* ветки | #1139 v2 | `85db3f5c` | dev не засоряется SHA-tag коммитами | deploy не подхватывал .image-versions из этих веток |
| Revert #1139 → push в develop | #1244/#1316 | `98de836a` | deploy тянет .image-versions напрямую | main+vision оба коммитят в одну ветку (race) |
| Race-safe push-image-versions.sh | #1388 | `3faa93b4` | retry+rebase, recover от non-fast-forward | нет dedup — каждый retry = новый коммит |
| GITHUB_SHA + verify в registry | #1482 | `d53fed3c` | SHA-tag не «плавает» внутри run | — |
| Mirror в GHCR | #1503 | `adc93c5a` | deploy работает с ghcr.io | — |
| Concurrency + diagnostic gate | #1560/#1566 | `e82726a7` | spam-build'ы develop фильтруются | gate не защищает от race на git push/pull |
| Skip develop post-merge build | #1625/#1626 | `ac550102` | каждый merge не запускает билд | — |
| **Hard-cap + GIT_TERMINAL_TIMEOUT** | **#1628** | **686d7160** | **hang становится visible fail (≤10 min)** | **build всё ещё зависит от `update-image-versions`** |

## 2. Что осталось (acceptance criteria #1630)

| Acceptance | Сейчас | Что делаем |
|---|---|---|
| Push SHA-тегов вынесен в **отдельный workflow** `L-Update Image Versions.yml` | ❌ | Создаём `L-Update Image Versions.yml` |
| Build workflow (`L-Build Main/Vision Pi Services`) НЕ делает git push в develop | ❌ (после #1628 висит ≤10 min, но всё равно блокирует) | Удаляем `update-image-versions` job |
| Build workflow только пушит в registry (ghcr.io / localhost:5000) | ❌ | Build jobs пушат, **отдельный workflow коммитит .image-versions** |
| Update-image-versions запускается отдельно (cron или trigger) | ❌ | workflow_call + workflow_dispatch + cron `0 * * * *` |
| Если SHA-tag push падает — **build не повторяется** | ❌ | Build не зависит от update-image-versions |
| Race-condition dedup | ❌ (только retry+rebase) | Добавляем `gh api` check перед push |
| e2e не зависят от SHA-tag | ✅ уже есть | ADR-0030 — `latest-stable` tag |

## 3. Что **точно НЕ** делаем

- **НЕ возвращаемся к per-service веткам `ci/image-versions-*`** — ретро #1244 показал, что deploy не подтягивает оттуда (см. комментарии в `L-Build Main Pi Services.yml:512-514`). Push остаётся в **текущую ветку** (develop), но через **отдельный workflow**.
- **НЕ убираем timeout-minutes/GIT_TERMINAL_TIMEOUT** из #1628 — оставляем как defense-in-depth.
- **НЕ ломаем `L-Build All & Push to GHCR.yml`** — там уже per-service модель через `commit-image-versions.sh`, её не трогаем.

## 4. Архитектура (target)

```
[push on develop/feature]
        │
        ▼
[L-Build Main/Vision Pi Services.yml]
   ├─ prepare
   ├─ build-* (parallel, push to local+ghcr.io registry)
   ├─ tag+push SHA-images to registry  ← verify_in_registry()
   └─ WRITE .image-versions.dev to disk (NO git push, NO commit)
        │
        └──► [calls workflow_call]
              ▼
              [L-Update Image Versions.yml]
                 ├─ inputs: service=main|vision, docker_tag=dev|test|latest,
                 │           source_sha=${{ github.sha }}
                 ├─ checkout (fetch-depth: 0)
                 ├─ pull --rebase origin develop
                 ├─ READ .image-versions.${docker_tag} from disk
                 ├─ sed SHA tags (deterministic from inputs)
                 ├─ dedup: gh api search for existing commit message
                 │  on develop → if match → skip push, log + exit 0
                 ├─ git commit "ci: {service} SHA tags → ${tag}-${sha}"
                 └─ push with retry+rebase (existing push-image-versions.sh)

[schedule: 0 * * * *]
        │
        └──► [L-Update Image Versions.yml] (workflow_dispatch, retry missed commits)
```

## 5. Файлы под изменение

- `+ .github/workflows/L-Update Image Versions.yml` — новый workflow
- `~ .github/workflows/L-Build Main Pi Services.yml` — удалить `update-image-versions` job, оставить tag_and_push+verify
- `~ .github/workflows/L-Build Vision Pi Services.yml` — то же
- `~ scripts/ci/push-image-versions.sh` — добавить dedup чек (env-driven: `IMAGE_VERSIONS_DEDUP_REMOTE_QUERY`)
- `+ scripts/ci/tests/test_push_image_versions_dedup.sh` — unit-тест для dedup
- `+ docs/adr/0031-sha-tag-push-out-of-build-workflow.md` — задокументировать решение

## 6. First commit (WIP)

Этот файл + ADR + обновление push-image-versions.sh (комментарии с TODO).

## 7. References

- Issue #1630 — main task
- Issue #1388 — race-safe push helper
- Issue #1482 — GITHUB_SHA + verify_in_registry
- Issue #1503 — mirror GHCR
- Issue #1560/#1566 — concurrency gate
- Issue #1625/#1626 — skip develop post-merge build
- Issue/PR #1628 — hard-cap + GIT_TERMINAL_TIMEOUT (merged 12:01 MSK)
- ADR-0013 — incremental delivery
- ADR-0018 — honesty culture
- ADR-0030 — e2e stale-branch guard
