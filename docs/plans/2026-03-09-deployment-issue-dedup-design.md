# Deployment Issue Deduplication Design

**Date:** 2026-03-09
**Status:** Approved

## Problem

The deployment verification workflow currently creates one new GitHub issue whenever any deployment check reports problems. Because the workflow does not compare new findings against already open deployment issues, repeated deploys generate many near-identical issues for the same underlying failure.

This creates noise, hides genuinely new regressions, and makes the deployment issue tracker hard to use.

## Goal

Change deployment verification so it creates a GitHub issue only for a newly discovered problem. If the same problem already has an open deployment issue, the workflow should skip issue creation for that problem.

## Non-Goals

- Do not auto-close existing issues.
- Do not merge multiple unrelated problems into one large issue.
- Do not redesign the log collection or health-check phases beyond what is needed for deduplication.
- Do not add external services or persistent storage beyond GitHub Issues.

## Current State

The workflow step [Analyze Results and Create Issue](../../.github/workflows/L-Deploy%20and%20Verify.yml#L711-L830) builds one summary issue from aggregate counters and always calls `gh issue create` if any issue-like signal is present.

Current behavior has two structural problems:

1. Findings are aggregated too early, so distinct failures are collapsed into one deploy-level summary.
2. There is no stable identity for a problem, so the workflow cannot tell whether a new deploy found a previously reported failure.

## Proposed Approach

### Summary

Refactor the issue-creation portion of the deployment workflow into a per-problem pipeline:

1. Collect raw findings from container status, topic checks, and log analysis.
2. Normalize each finding into an individual deployment problem record.
3. Compute a stable signature for each problem.
4. Search open deployment issues for that signature.
5. Create a new issue only when no open issue already contains the same signature.

### Problem Record Model

Each detected problem should be represented as a normalized record with fields equivalent to:

- `environment`
- `scope` (`vision` or `main`)
- `container` (or `system` for non-container checks)
- `kind` (`container_status`, `topic_check`, `critical_log`, `warning_log`)
- `severity` (`critical` or `warning`)
- `summary`
- `normalized_pattern`
- `signature`

This shifts the workflow from “one deploy produced some bad counts” to “this deploy found these concrete problems”.

### Signature Format

Use a stable string marker embedded in the issue body:

`deploy-problem:<environment>:<scope>:<container>:<kind>:<hash>`

Where `<hash>` is computed from a normalized pattern derived from the problem text.

Example marker:

`<!-- deploy-signature: deploy-problem:prod:vision:oak-d:critical_log:abcd1234 -->`

The marker lives in the issue body so it is searchable via GitHub CLI/API and does not require extra labels or state.

### Normalization Rules

To avoid duplicates caused by run-specific noise, normalization should strip or collapse unstable values such as:

- timestamps
- workflow run numbers and URLs
- IP addresses when they are incidental
- repeated whitespace
- numeric counters that do not identify a distinct failure mode

For log-derived findings, the workflow should use the first meaningful matching line or a small normalized excerpt rather than the entire collected log block.

Examples:

- `ERROR connection to 10.1.1.21 timed out after 30s` → normalize to connection timeout pattern
- `Traceback ... File ... line 123` → normalize to traceback header + exception type/message
- `container exited with code 1` for `oak-d` → signature remains distinct by container and check type

### Duplicate Detection

Before creating an issue for a problem, the workflow should search open issues labeled `deployment` and inspect them for the exact signature marker.

Decision logic:

- matching open issue exists → skip creation
- no matching open issue exists → create one new issue for that problem

This preserves one open issue per active unique deployment problem.

### Issue Shape

Each created issue should describe one problem, not the entire deploy batch.

Recommended issue content:

- clear title with severity, environment, scope, and container
- signature marker in HTML comment
- normalized summary
- deploy metadata (branch, workflow run, timestamp)
- short raw evidence excerpt
- quick diagnostic commands

This makes the issue actionable and keeps unrelated failures separate.

### Workflow Structure Changes

The workflow should be split into smaller logical stages inside the current verify job:

1. **Collect findings**
   - produce machine-readable files for status/log/topic failures
2. **Normalize findings**
   - convert raw checks into problem records
3. **Deduplicate**
   - search open issues for signatures
4. **Create only new issues**
   - call `gh issue create` only for unmatched problems
5. **Summarize**
   - print how many problems were found, skipped as duplicates, and newly created

## Alternatives Considered

### 1. Deduplicate by title only

Rejected. Small wording changes or branch/environment differences would still create duplicate issues.

### 2. One open issue per container

Rejected. Different failures in the same container would be mixed together and become hard to triage.

### 3. One open issue per environment

Rejected. This preserves the current “bag of problems” behavior and hides new regressions.

## Error Handling

- If issue search fails, the workflow should fail safe by logging the search failure clearly.
- If normalization produces an empty pattern, the workflow should fall back to a conservative raw summary.
- If issue creation for one problem fails, the workflow should continue attempting the remaining new problems and report partial failure in the summary.

## Testing Strategy

Use test-first coverage for the normalization and deduplication helper logic.

Tests should verify:

1. same problem text across different runs produces the same signature
2. unrelated problems produce different signatures
3. duplicate existing issue markers prevent new issue creation
4. one deploy containing repeated identical findings produces one new issue at most
5. workflow-facing helper output remains parseable by shell steps

## Documentation Changes

Update [docs/deployment/DEPLOYMENT_WORKFLOW.md](../../docs/deployment/DEPLOYMENT_WORKFLOW.md) to describe:

- per-problem issue creation
- signature-based deduplication
- skip behavior when the issue already exists
- new expected issue format

## Implementation Notes

Prefer moving the complex logic out of inline shell into a small helper script under `.github/scripts/` or similar, because:

- normalization and hashing are easier to test there
- GitHub issue search/parsing is less error-prone than large inline shell pipelines
- future tuning of exclusion and normalization rules will be simpler

## Success Criteria

The design is successful when:

1. repeated deploys with the same known failure do not create new issues
2. a genuinely new failure creates exactly one new issue
3. multiple different failures in one deploy can create multiple distinct issues
4. deployment docs reflect the new behavior
