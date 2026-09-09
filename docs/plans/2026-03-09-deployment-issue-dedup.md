# Deployment Issue Deduplication Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Prevent the deployment verification workflow from creating duplicate auto-generated GitHub issues for already-known deployment problems.

**Architecture:** Move deployment-problem normalization and deduplication out of the workflow’s monolithic inline issue step into a small tested helper script. The workflow will collect raw findings, convert them into per-problem records, search open deployment issues for signature markers, and create new issues only for unseen problems.

**Tech Stack:** GitHub Actions YAML, shell, Python 3, GitHub CLI, Markdown docs, pytest/unittest-style script tests.

---

### Task 1: Inspect current workflow contract and choose helper locations

**Files:**
- Modify: `.github/workflows/L-Deploy and Verify.yml`
- Create: `.github/scripts/deployment_issue_dedup.py`
- Create: `.github/scripts/tests/test_deployment_issue_dedup.py`

**Step 1: Write the failing test**

Create a small test module that asserts a helper function can normalize a raw finding into a stable problem record.

Example test target:
- input: container `oak-d`, kind `critical_log`, text with timestamp/run-specific noise
- expected: normalized pattern strips unstable values

**Step 2: Run test to verify it fails**

Run: `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q`

Expected: FAIL because helper script or function does not exist yet.

**Step 3: Write minimal implementation**

Create `.github/scripts/deployment_issue_dedup.py` with:
- a simple CLI entrypoint scaffold
- a pure function for normalization
- a pure function for building a signature from normalized data

**Step 4: Run test to verify it passes**

Run: `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q`

Expected: PASS

**Step 5: Commit**

```bash
git add .github/scripts/deployment_issue_dedup.py .github/scripts/tests/test_deployment_issue_dedup.py
git commit -m "test(deploy): add deployment issue dedup helpers"
```

### Task 2: Add signature stability tests

**Files:**
- Modify: `.github/scripts/tests/test_deployment_issue_dedup.py`
- Modify: `.github/scripts/deployment_issue_dedup.py`

**Step 1: Write the failing test**

Add tests proving:
- same problem in two runs produces the same signature
- different containers produce different signatures
- different problem kinds produce different signatures

**Step 2: Run test to verify it fails**

Run: `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q`

Expected: FAIL on at least one new signature assertion.

**Step 3: Write minimal implementation**

Implement deterministic signature generation using:
- environment
- scope
- container
- kind
- hash of normalized pattern

**Step 4: Run test to verify it passes**

Run: `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q`

Expected: PASS

**Step 5: Commit**

```bash
git add .github/scripts/deployment_issue_dedup.py .github/scripts/tests/test_deployment_issue_dedup.py
git commit -m "test(deploy): verify stable issue signatures"
```

### Task 3: Teach helper to parse raw findings into per-problem records

**Files:**
- Modify: `.github/scripts/deployment_issue_dedup.py`
- Modify: `.github/workflows/L-Deploy and Verify.yml`
- Modify: `.github/scripts/tests/test_deployment_issue_dedup.py`

**Step 1: Write the failing test**

Add tests for transforming raw collected findings into a list of problem records. Include cases for:
- failed container status
- critical log match
- warning log match
- topic-check failure

**Step 2: Run test to verify it fails**

Run: `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q`

Expected: FAIL because parser/output format is incomplete.

**Step 3: Write minimal implementation**

Implement helper functions to accept raw JSON/text input and emit structured records, for example JSON lines.

Update the workflow to write raw findings into temp files instead of directly jumping to one issue body.

**Step 4: Run test to verify it passes**

Run: `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q`

Expected: PASS

**Step 5: Commit**

```bash
git add .github/workflows/L-Deploy\ and\ Verify.yml .github/scripts/deployment_issue_dedup.py .github/scripts/tests/test_deployment_issue_dedup.py
git commit -m "refactor(deploy): emit per-problem findings"
```

### Task 4: Add duplicate-detection tests against existing issue markers

**Files:**
- Modify: `.github/scripts/tests/test_deployment_issue_dedup.py`
- Modify: `.github/scripts/deployment_issue_dedup.py`

**Step 1: Write the failing test**

Add tests for a function that receives:
- a candidate problem signature
- a list of open issues or issue bodies

And returns whether a duplicate already exists.

Cover:
- exact marker match found
- no marker found
- deployment issue without marker should not match

**Step 2: Run test to verify it fails**

Run: `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q`

Expected: FAIL on duplicate-detection behavior.

**Step 3: Write minimal implementation**

Implement marker parsing and duplicate matching using the HTML comment marker:
- `<!-- deploy-signature: ... -->`

**Step 4: Run test to verify it passes**

Run: `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q`

Expected: PASS

**Step 5: Commit**

```bash
git add .github/scripts/deployment_issue_dedup.py .github/scripts/tests/test_deployment_issue_dedup.py
git commit -m "test(deploy): cover duplicate issue detection"
```

### Task 5: Replace monolithic issue creation step with deduplicated per-problem creation

**Files:**
- Modify: `.github/workflows/L-Deploy and Verify.yml`
- Modify: `.github/scripts/deployment_issue_dedup.py`
- Test: `.github/scripts/tests/test_deployment_issue_dedup.py`

**Step 1: Write the failing test**

Add an integration-oriented helper test that simulates:
- two identical findings in one deploy
- one existing matching open issue
- one new unrelated finding

Expected result:
- duplicate is skipped
- only one new issue payload is generated

**Step 2: Run test to verify it fails**

Run: `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q`

Expected: FAIL because end-to-end orchestration is incomplete.

**Step 3: Write minimal implementation**

Update the workflow to:
- call the helper script to build candidate problems
- fetch open deployment issues using `gh`
- call helper script again to decide what is new
- iterate only over new problems and create one issue per problem
- print counts for found/skipped/created

Remove the old single-summary issue logic.

**Step 4: Run test to verify it passes**

Run: `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q`

Expected: PASS

**Step 5: Verify workflow syntax**

Run: `python - <<'PY'
import yaml, pathlib
path = pathlib.Path('.github/workflows/L-Deploy and Verify.yml')
print('ok' if yaml.safe_load(path.read_text()) else 'empty')
PY`

Expected: `ok`

**Step 6: Commit**

```bash
git add .github/workflows/L-Deploy\ and\ Verify.yml .github/scripts/deployment_issue_dedup.py .github/scripts/tests/test_deployment_issue_dedup.py
git commit -m "feat(deploy): deduplicate auto-generated deployment issues"
```

### Task 6: Update deployment documentation

**Files:**
- Modify: `docs/deployment/DEPLOYMENT_WORKFLOW.md`

**Step 1: Write the failing test**

Use a documentation checklist instead of an automated test. Verify the doc still describes the old behavior and is therefore outdated.

Checklist:
- does it still say every problematic deploy creates an issue?
- does it document per-problem signatures?
- does it explain duplicate skipping?

Expected: checklist FAILS before edits.

**Step 2: Write minimal implementation**

Update the docs to describe:
- per-problem issue creation
- signature marker format
- duplicate skip behavior
- practical triage flow

**Step 3: Verify docs are correct**

Read the updated section and confirm it matches the implemented workflow.

**Step 4: Commit**

```bash
git add docs/deployment/DEPLOYMENT_WORKFLOW.md
git commit -m "docs(deploy): document issue deduplication flow"
```

### Task 7: Final verification

**Files:**
- Modify: none
- Test: `.github/scripts/tests/test_deployment_issue_dedup.py`
- Test: `.github/workflows/L-Deploy and Verify.yml`
- Test: `docs/deployment/DEPLOYMENT_WORKFLOW.md`

**Step 1: Run helper tests**

Run: `python -m pytest .github/scripts/tests/test_deployment_issue_dedup.py -q`

Expected: PASS

**Step 2: Run broader relevant tests if created**

Run any additional targeted tests added during implementation.

Expected: PASS

**Step 3: Validate workflow file**

Run: `python - <<'PY'
import yaml, pathlib
path = pathlib.Path('.github/workflows/L-Deploy and Verify.yml')
print('ok' if yaml.safe_load(path.read_text()) else 'empty')
PY`

Expected: `ok`

**Step 4: Inspect git diff**

Run: `git diff --stat HEAD~4..HEAD`

Expected: workflow, helper script, tests, and docs only.

**Step 5: Commit if needed**

```bash
git status
```

Expected: clean working tree
