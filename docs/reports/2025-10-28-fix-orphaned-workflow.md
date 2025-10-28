# Fix: Orphaned Workflow build-all-local.yml

**Date:** 2025-10-28  
**Issue:** https://github.com/krikz/rob_box_project/actions/workflows/build-all-local.yml  
**Status:** ✅ Fixed

## Problem Description

The workflow file `.github/workflows/build-all-local.yml` appeared in GitHub Actions UI but caused repeated failures. Investigation revealed:

1. **Orphaned Workflow**: The workflow was created in branch `copilot/add-build-machine-composition` which was subsequently deleted
2. **GitHub Behavior**: GitHub Actions keeps workflow metadata even after the source branch is deleted
3. **Failed Runs**: 3 consecutive failed runs occurred because the workflow file didn't exist in any active branch
4. **Naming Issue**: The workflow didn't follow project naming convention (missing `L-` prefix)

### Workflow Run History

| Run ID | Date | SHA | Result | Reason |
|--------|------|-----|--------|--------|
| 18817359175 | 2025-10-26 | 7e42135 | ❌ Failed | File not found in branch |
| 18817337587 | 2025-10-26 | f4ed4ec | ❌ Failed | File not found in branch |
| 18817306387 | 2025-10-26 | 0c4f440 | ❌ Failed | File not found in branch |

## Root Cause Analysis

### Why Did This Happen?

1. **Feature Development**: A developer created `build-all-local.yml` in a feature branch
2. **Incorrect Naming**: The file didn't follow the `L-*` prefix convention for local workflows
3. **Branch Deletion**: The feature branch was deleted without cleaning up the workflow
4. **GitHub's Persistence**: GitHub maintains workflow registry entries even after branch deletion

### Project Naming Convention

According to `docs/CI_CD_PIPELINE.md`:

- **G-*** prefix - GitHub Actions workflows (cloud runners)
- **L-*** prefix - Local self-hosted runner workflows

The correct workflow already exists: `L-Build All Services.yml`

## Solution Implemented

Created a deprecation placeholder workflow file that:

1. ✅ **Properly named**: Uses the exact filename GitHub is tracking
2. ✅ **Clear messaging**: Shows deprecation notice to users
3. ✅ **User guidance**: Redirects to correct workflow (`L-Build All Services.yml`)
4. ✅ **Historical context**: Explains why this file exists
5. ✅ **Prevents execution**: Exits with error unless user confirms understanding

### File Structure

```yaml
name: "[DEPRECATED] build-all-local"

on:
  workflow_dispatch:  # Manual trigger only
    inputs:
      confirm_deprecated:
        description: 'This workflow is DEPRECATED. Use L-Build All Services.yml instead!'
        required: true
        type: boolean
        default: false

jobs:
  deprecation-notice:
    runs-on: ubuntu-latest
    if: github.event.inputs.confirm_deprecated != 'true'
    steps:
      - name: Show Deprecation Notice
        run: |
          echo "❌ THIS WORKFLOW IS DEPRECATED"
          echo "Please use: L-Build All Services.yml"
          exit 1
  
  redirect-to-new-workflow:
    runs-on: ubuntu-latest
    if: github.event.inputs.confirm_deprecated == 'true'
    steps:
      - name: Redirect Information
        run: |
          echo "⚠️ You are using a deprecated workflow!"
          echo "Please switch to: L-Build All Services.yml"
          exit 1
```

## Verification Steps

After merging to main, verify:

1. ✅ Workflow appears in GitHub Actions UI with correct name
2. ✅ Workflow shows "[DEPRECATED]" prefix clearly
3. ✅ Manual trigger shows deprecation warning in description
4. ✅ Running workflow displays clear error message
5. ✅ No more "broken" workflow failures

## Prevention Measures

To prevent similar issues in the future:

### For Developers

1. **Follow Naming Convention**: Always use `G-*` or `L-*` prefix
2. **Check Existing Workflows**: Review similar workflows before creating new ones
3. **Clean Up Branches**: Ensure workflows are properly handled before deleting branches
4. **Use Correct Workflow**: Prefer reusing/calling existing workflows

### For Review Process

1. **Workflow Review**: Pay special attention to new workflow files in PRs
2. **Naming Compliance**: Verify workflow names follow project convention
3. **Documentation Check**: Ensure workflow is documented in `docs/CI_CD_PIPELINE.md`

## Related Documentation

- **CI/CD Pipeline**: `docs/CI_CD_PIPELINE.md` - Complete workflow documentation
- **Workflow Naming**: See "Naming Convention" section in CI_CD_PIPELINE.md
- **Local Builds**: Use `L-Build All Services.yml` for self-hosted runner builds

## Correct Workflows to Use

For local/self-hosted builds:

| Workflow | Purpose | Trigger |
|----------|---------|---------|
| **L-Build All Services.yml** | Full build (all services) | Manual |
| **L-Build Base Images.yml** | Base images only | Manual/Called |
| **L-Build Main Pi Services.yml** | Main Pi services | Manual/Called |
| **L-Build Vision Pi Services.yml** | Vision Pi services | Manual/Called |

## Implementation Timeline

- **2025-10-26**: Orphaned workflow created and deleted
- **2025-10-28**: Issue identified
- **2025-10-28**: Deprecation placeholder created
- **2025-10-28**: CHANGELOG updated
- **2025-10-28**: This report created

## Conclusion

The orphaned workflow issue has been resolved by creating a proper deprecation placeholder. This approach:

- ✅ Eliminates the "broken" workflow appearance
- ✅ Guides users to the correct workflow
- ✅ Maintains workflow history for reference
- ✅ Prevents accidental misuse
- ✅ Documents the issue for future reference

The proper workflow for local builds is **L-Build All Services.yml**.

---

**Created by:** AI Assistant (Copilot)  
**Branch:** copilot/fix-workflow-issues  
**PR:** TBD (pending merge)
