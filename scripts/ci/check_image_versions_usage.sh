#!/usr/bin/env bash
# Check that every *_TAG declared in docker/{main,vision}/.image-versions.*
# is actually referenced somewhere in the component (compose / dockerfile /
# shell script). Flags phantom env-variables that get written to .image-versions
# but never consumed by anyone — silent tech debt (issue #2425 #2377).
#
# ADR-0090 §3.2 acceptance criterion: this script exits non-zero if any
# *_TAG is unused. Called from CI as a required check.
#
# Usage:
#   check_image_versions_usage.sh            # checks all components
#   check_image_versions_usage.sh docker/main docker/vision
#
# Environment:
#   IV_KNOWN_PHANTOMS  Space-separated list of "TAG:comment" entries that
#                      are intentionally unused (e.g. experimental, disabled).
#                      Whitelisted — script will WARN, not FAIL.
#
# Exit:
#   0 — all *_TAG used (or whitelisted)
#   1 — at least one phantom *_TAG not in whitelist
set -euo pipefail

RC=0
WARN_COUNT=0

if [ "$#" -eq 0 ]; then
  set -- docker/main docker/vision
fi

for component_dir in "$@"; do
  [ -d "$component_dir" ] || continue

  # Find every .image-versions.* file in this component
  for vf in "$component_dir"/.image-versions.*; do
    [ -f "$vf" ] || continue

    while IFS='=' read -r key val; do
      # Strip leading/trailing whitespace
      key="$(echo "$key" | xargs)"
      [ -z "$key" ] && continue
      case "$key" in
        \#*) continue ;;        # comment line
      esac
      # Only check keys that look like image-version vars
      case "$key" in
        *_TAG) ;;
        *) continue ;;
      esac

      # Look for usage of $key (word boundary) anywhere in the component
      # outside the .image-versions files themselves.
      #
      # We search by exact key match (grep -w) so *_TAG_BAK or
      # TWIST_MUX_TAG_FOO won't trigger. This catches the realistic
      # false-positive (suffix decoration) but also helps when someone
      # renames a var without updating references.
      # Exclude scripts/ci/tests/ — test fixtures use phantom-like names by
      # design and would false-positive this check.
      hits=$(grep -rlw --include="*.yml" --include="*.yaml" \
                       --include="*.env" --include="*.sh" \
                       --include="Dockerfile*" --include="*.dockerfile" \
             -e "$key" "$component_dir" scripts/ 2>/dev/null \
        | grep -v "/.image-versions" \
        | grep -Ev "(^|/)scripts/ci/tests/" || true)

      if [ -z "$hits" ]; then
        # Check whitelist (IV_KNOWN_PHANTOMS="TAG1:reason TAG2:reason ...")
        whitelisted=0
        for w in ${IV_KNOWN_PHANTOMS:-}; do
          wkey="${w%%:*}"
          if [ "$wkey" = "$key" ]; then
            whitelisted=1
            wreason="${w#*:}"
            echo "⚠️  $vf: $key=$val — whitelist ($wreason)" >&2
            WARN_COUNT=$((WARN_COUNT + 1))
            break
          fi
        done
        if [ "$whitelisted" -eq 0 ]; then
          echo "❌ $vf: $key=$val — PHANTOM (not used in $component_dir or scripts/)" >&2
          RC=1
        fi
      else
        echo "✓ $vf: $key=$val — used in: $(echo "$hits" | tr '\n' ' ')"
      fi
    done < "$vf"
  done
done

if [ "$RC" -ne 0 ]; then
  echo "" >&2
  echo "❌ phantom env-variables found. Fix: delete the *_TAG from .image-versions.*" >&2
  echo "   (or add to IV_KNOWN_PHANTOMS if intentionally unused)" >&2
elif [ "$WARN_COUNT" -gt 0 ]; then
  echo "" >&2
  echo "⚠️  $WARN_COUNT whitelisted phantom(s). See ADR-0090 §3.2." >&2
fi

exit "$RC"