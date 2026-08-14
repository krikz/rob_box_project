#!/bin/bash
# Clean up old images from local registry
# Usage: ./cleanup_registry.sh [--all] [--keep N] [--dry-run]
#
# --all      удалить ВСЕ теги (полный сброс registry)
# --keep N   оставить последние N sha-версий на каждый сервис-вариант,
#            остальные удалить. Rolling-теги (без sha-суффикса: dev/latest/
#            local/test и т.п.) НЕ удаляются. Default N=5.
# --dry-run  только показать, что было бы удалено, ничего не удалять
#
# Группировка сервис-варианта: у тега вида
#   voice-assistant-humble-dev-8c1090b9
# отбрасывается sha-суффикс → группа voice-assistant-humble-dev.
# Внутри группы сортируем по времени создания образа (created из image
# config), оставляем N самых свежих, остальные удаляем. Теги без
# sha-суффикса (rolling: voice-assistant-humble-dev, supercollider-local,
# dev, latest, ...) никогда не удаляются в режиме --keep.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$(dirname "$SCRIPT_DIR")"

# Load environment
if [ -f "$BUILD_DIR/.env" ]; then
    # shellcheck disable=SC1091
    source "$BUILD_DIR/.env"
fi

REGISTRY_URL="${REGISTRY_URL:-http://localhost:${REGISTRY_PORT:-5000}}"
REGISTRY_CONTAINER="${REGISTRY_CONTAINER:-build-registry}"
DRY_RUN=false
DELETE_ALL=false
KEEP_N=5

# Parse arguments
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --all)
            DELETE_ALL=true
            shift
            ;;
        --keep)
            KEEP_N="$2"
            shift 2
            ;;
        --keep=*)
            KEEP_N="${1#*=}"
            shift
            ;;
        *)
            shift
            ;;
    esac
done

# Валидация KEEP_N
if ! [[ "$KEEP_N" =~ ^[0-9]+$ ]] || [ "$KEEP_N" -lt 0 ]; then
    echo "❌ Invalid --keep value: '$KEEP_N' (expected non-negative integer)"
    exit 2
fi

echo "🧹 Registry Cleanup"
echo "==================="
echo ""

if [ "$DRY_RUN" = true ]; then
    echo "🔍 DRY RUN MODE - no images will be deleted"
    echo ""
fi

if [ "$DELETE_ALL" = true ]; then
    echo "⚠️  MODE: --all (delete EVERYTHING)"
else
    echo "ℹ️  MODE: --keep $KEEP_N (rolling tags are preserved)"
fi
echo ""

# Check if registry is accessible
if ! curl -s "$REGISTRY_URL/v2/" > /dev/null 2>&1; then
    echo "❌ Registry is not accessible at $REGISTRY_URL"
    echo "   Make sure the registry service is running."
    exit 1
fi

# Get list of repositories
echo "📦 Repositories in registry:"
REPOS=$(curl -s "$REGISTRY_URL/v2/_catalog" | jq -r '.repositories[]' 2>/dev/null || echo "")

if [ -z "$REPOS" ]; then
    echo "   No repositories found"
    exit 0
fi

echo "$REPOS" | sed 's/^/   - /'
echo ""

DELETED=0

# For each repository, compute candidates to delete
for repo in $REPOS; do
    TAGS=$(curl -s "$REGISTRY_URL/v2/$repo/tags/list" | jq -r '.tags[]' 2>/dev/null || echo "")

    if [ -z "$TAGS" ]; then
        continue
    fi

    echo "📂 Repository: $repo ($(echo "$TAGS" | wc -l) tags)"

    # Build candidate list:
    #  - --all: every tag
    #  - --keep: sha-versions beyond the newest N per service-variant
    CANDIDATES=$(KEEP_N="$KEEP_N" DELETE_ALL="$DELETE_ALL" REGISTRY_URL="$REGISTRY_URL" \
        python3 -c '
import json, os, re, sys, urllib.request
from collections import defaultdict
from urllib.error import HTTPError, URLError

keep_n = int(os.environ.get("KEEP_N", "5"))
delete_all = os.environ.get("DELETE_ALL", "false") == "true"
base = os.environ["REGISTRY_URL"]

def fetch(path):
    req = urllib.request.Request(
        base + path,
        headers={"Accept": "application/vnd.docker.distribution.manifest.v2+json, application/vnd.oci.image.manifest.v1+json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=20) as r:
            return json.load(r)
    except HTTPError as e:
        # 404 = тег ссылается на уже удалённый manifest (битый тег) — кандидат на удаление
        if e.code == 404:
            return "BROKEN"
        return None
    except Exception:
        # сетевая ошибка/таймаут — не трогаем (не можем определить возраст)
        return None

repo = sys.argv[1]
tags = [t.strip() for t in sys.stdin.read().splitlines() if t.strip()]
if not tags:
    sys.exit(0)

sha_re = re.compile(r"^(.*)-([0-9a-f]{7,40})$")

def created_for(tag):
    """created timestamp from image config.
    Returns:
      "BROKEN" — manifest 404 (dangling tag), safe to delete
      ""       — no created in config
      None     — registry unreachable / unknown (do NOT delete)
    """
    m = fetch(f"/v2/{repo}/manifests/{tag}")
    if m == "BROKEN":
        return "BROKEN"
    if not m:
        return None
    cfg = m.get("config") or {}
    cfg_digest = cfg.get("digest")
    if not cfg_digest:
        manifests = m.get("manifests") or []
        if manifests:
            return created_for_digest(manifests[0].get("digest", ""))
        return ""
    return created_for_digest(cfg_digest)

def created_for_digest(digest):
    if not digest:
        return ""
    b = fetch(f"/v2/{repo}/blobs/{digest}")
    if b == "BROKEN" or b is None:
        return ""
    return b.get("created") or ""

if delete_all:
    for t in sorted(tags):
        print(t)
    sys.exit(0)

# Group only sha-versions; rolling tags (no -sha suffix) are never touched
groups = defaultdict(list)
for t in tags:
    m = sha_re.match(t)
    if m:
        groups[m.group(1)].append(t)

out = []
for prefix, group in sorted(groups.items()):
    if len(group) <= keep_n:
        continue
    # sort by created desc, then by tag name for determinism;
    # "BROKEN" and "" (unknown age) sort last → deleted first when over keep_n
    with_created = []
    for t in group:
        c = created_for(t)
        if c is None:
            # registry недоступен для этого тега — не рискуем, пропускаем группу
            with_created = None
            break
        with_created.append((c, t))
    if with_created is None:
        print(f"WARN: registry unreachable for {prefix} — skipping group", file=sys.stderr)
        continue
    with_created.sort(key=lambda x: (x[0], x[1]), reverse=True)
    for c, t in with_created[keep_n:]:
        out.append(t)

for t in sorted(out):
    print(t)
' "$repo" <<< "$TAGS") || true

    COUNT=$(echo "$CANDIDATES" | sed '/^$/d' | wc -l)
    echo "   → $COUNT tag(s) to delete"

    while IFS= read -r tag; do
        [ -z "$tag" ] && continue
        DIGEST=$(curl -s -I -H "Accept: application/vnd.docker.distribution.manifest.v2+json" \
                 "$REGISTRY_URL/v2/$repo/manifests/$tag" 2>/dev/null | \
                 grep -i "Docker-Content-Digest:" | awk '{print $2}' | tr -d '\r' || true)
        if [ -z "$DIGEST" ]; then
            echo "   ⚠️  No digest for $repo:$tag (dangling tag) — skip (use delete_image_from_registry.sh for fs cleanup)"
            continue
        fi
        if [ "$DRY_RUN" = true ]; then
            echo "   [DRY RUN] Would delete: $repo:$tag"
        else
            if curl -s -X DELETE "$REGISTRY_URL/v2/$repo/manifests/$DIGEST" > /dev/null 2>&1; then
                DELETED=$((DELETED+1))
            else
                echo "   ⚠️  Failed to delete: $repo:$tag"
            fi
        fi
    done <<< "$CANDIDATES"
    echo ""
done

if [ "$DRY_RUN" = true ]; then
    echo "🔍 DRY RUN — nothing deleted (preview only)"
elif [ "$DELETE_ALL" = true ] || [ "$DELETED" -gt 0 ]; then
    echo "🔄 Running garbage collection..."
    docker exec "$REGISTRY_CONTAINER" bin/registry garbage-collect /etc/docker/registry/config.yml > /dev/null 2>&1 || \
        echo "   ⚠️  garbage-collect failed (continue)"
    echo "✅ Deleted $DELETED tag(s), GC done"
else
    echo "ℹ️  Nothing deleted (all groups are within keep=$KEEP_N)"
fi
