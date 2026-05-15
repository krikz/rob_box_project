---
phase: 1
slug: documentation-audit
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-05-15
---

# Phase 1 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | Shell scripts (bash assertions) — no code tests, documentation-only phase |
| **Config file** | none |
| **Quick run command** | `grep -rn "ubuntu@" docs/ README.md` (should return 0 results) |
| **Full suite command** | `bash .planning/phases/01-documentation-audit/validate.sh` |
| **Estimated runtime** | ~5 seconds |

---

## Sampling Rate

- **After every task commit:** Run quick check (grep for known-bad patterns)
- **After every plan wave:** Run full validation script
- **Before `/gsd-verify-work`:** Full suite must be green
- **Max feedback latency:** ~5 seconds

---

## Per-Task Verification Map

| Task ID | Plan | Requirement | Test Type | Automated Command | Status |
|---------|------|-------------|-----------|-------------------|--------|
| 01-01-01 | 01-01 | DOCS-06 | automated | `grep -rn "ubuntu@" docs/ README.md` → 0 results | ⬜ pending |
| 01-01-02 | 01-01 | DOCS-06 | automated | `grep -rn "animation-player" docs/ README.md` → 0 results | ⬜ pending |
| 01-01-03 | 01-01 | DOCS-01 | manual | Review TROUBLESHOOTING.md has VESC/Zenoh/LiDAR cases | ⬜ pending |
| 01-02-01 | 01-02 | DOCS-03 | automated | Container list in SYSTEM_OVERVIEW.md matches `grep "container_name:" docker/main/docker-compose.yaml` | ⬜ pending |
| 01-02-02 | 01-02 | DOCS-03 | automated | Container list in SYSTEM_OVERVIEW.md matches `grep "container_name:" docker/vision/docker-compose.yaml` | ⬜ pending |
| 01-03-01 | 01-03 | DOCS-05 | automated | `for d in src/*/; do [ -f "$d/README.md" ] && echo OK || echo MISSING; done` → all OK | ⬜ pending |
| 01-03-02 | 01-03 | DOCS-05 | automated | `grep -rn "jazzy" src/rob_box_voice/README.md` → 0 results | ⬜ pending |
| 01-03-03 | 01-03 | DOCS-05 | manual | rob_box_voice README covers: USE_SKILLS, VoiceMemory, Ollama, SuperCollider | ⬜ pending |
| 01-03-04 | 01-03 | DOCS-05 | manual | rob_box_telegram README covers: nodes, topics, parameters | ⬜ pending |
| 01-03-05 | 01-03 | DOCS-02 | automated | `grep -rn "ubuntu@" README.md` → 0 results | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

No code framework needed — this is a documentation-only phase. All verification is via shell assertions and manual review.

Create validation script at `.planning/phases/01-documentation-audit/validate.sh`:
```bash
#!/bin/bash
# Phase 1 validation script
set -e
echo "=== Phase 1: Documentation Audit Validation ==="

# DOCS-06: No bad SSH username
if grep -rn "ubuntu@" docs/ README.md 2>/dev/null | grep -v ".git"; then
  echo "FAIL: Found 'ubuntu@' in docs — should be 'ros2@'"; exit 1
fi
echo "✅ SSH username: no 'ubuntu@' found"

# DOCS-06: No non-existent container name
if grep -rn "animation-player" docs/ README.md src/*/README.md 2>/dev/null; then
  echo "FAIL: Found 'animation-player' — container is named 'led-matrix'"; exit 1
fi
echo "✅ Container name: no 'animation-player' found"

# DOCS-05: All src/ packages have README
MISSING=0
for d in src/*/; do
  pkg=$(basename "$d")
  if [ ! -f "$d/README.md" ]; then
    echo "FAIL: Missing README: $pkg"
    MISSING=$((MISSING + 1))
  fi
done
if [ $MISSING -gt 0 ]; then exit 1; fi
echo "✅ All src/ packages have README.md"

# DOCS-05: No jazzy references in voice README
if grep -rn "jazzy" src/rob_box_voice/README.md 2>/dev/null; then
  echo "FAIL: Found 'jazzy' in rob_box_voice README — should be 'humble'"; exit 1
fi
echo "✅ Voice README: no jazzy dependency"

# DOCS-03: Key Vision Pi containers in SYSTEM_OVERVIEW
for container in "telegram-bot" "ceiling-camera" "supercollider"; do
  if ! grep -q "$container" docs/architecture/SYSTEM_OVERVIEW.md; then
    echo "FAIL: '$container' not found in SYSTEM_OVERVIEW.md"; exit 1
  fi
done
echo "✅ SYSTEM_OVERVIEW.md: Vision Pi containers present"

echo ""
echo "=== All validation checks passed ==="
```

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| TROUBLESHOOTING.md has current issues | DOCS-01 | Content quality, not grep-testable | Check file covers: Zenoh connectivity, VESC timeout, LiDAR not publishing |
| QUICK_START.md walkthrough works | DOCS-06 | Requires actual robot | Follow guide step-by-step up to `docker compose ps` |
| rob_box_voice README covers new features | DOCS-05 | Semantic content check | Verify: USE_SKILLS, VoiceMemory, MusicSkill, Ollama, SuperCollider/Renardo sections exist |
| rob_box_telegram README is complete | DOCS-05 | New README from scratch | Verify nodes, topics, params table populated correctly from source |
| docs/architecture/ visual accuracy | DOCS-03 | Mermaid diagram requires visual inspection | Render diagram, count services in Main Pi and Vision Pi sections |

---

## Validation Sign-Off

- [ ] All tasks have automated verify or manual verification steps documented
- [ ] Sampling continuity: validate.sh runs after each wave
- [ ] Wave 0 covers validate.sh creation
- [ ] No watch-mode flags
- [ ] Feedback latency < 10s
- [ ] `nyquist_compliant: true` set in frontmatter when all checks pass

**Approval:** pending
