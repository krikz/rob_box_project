# Deployment Workflow SSH Verification Enhancement

**Date:** 2025-10-29  
**Issue:** GitHub Actions run #18896855146 failure investigation  
**Status:** ✅ Resolved  
**Author:** GitHub Copilot

## Problem Statement

The deployment workflow `G: Deploy and Verify` was failing with generic SSH connection errors when trying to connect to Raspberry Pi devices:

```
ssh: connect to host 10.1.1.21 port 22: No route to host
```

**Issues:**
1. Error message didn't clarify whether SSH tools were missing or network was unreachable
2. Workflow would try all deployment steps before failing
3. No clear guidance on how to fix the issue
4. Users couldn't distinguish between:
   - Missing `sshpass` package in runner
   - Network connectivity issues (private IP addresses)
   - Actual device/network problems

## Root Cause

GitHub Actions hosted runners cannot reach private network IP addresses (10.1.1.x range). The Rob Box project uses Raspberry Pi devices on a local network with private IPs:
- Vision Pi: `10.1.1.21`
- Main Pi: `10.1.1.20`

The workflow needs either:
- A self-hosted runner on the same network
- VPN connection to the private network
- SSH bastion host with public IP

## Solution

### 1. Added SSH Verification Step

Added new workflow step `Verify SSH Tools and Connectivity` that runs before any deployment operations:

#### SSH Tools Check
Verifies availability of required tools:
- `ssh` - SSH client
- `sshpass` - Password-based SSH authentication

Example output:
```
🔍 Verifying SSH tools and connectivity...

1. Checking SSH tools availability...
✅ ssh: /usr/bin/ssh
   OpenSSH_9.6p1, OpenSSL 3.0.13
✅ sshpass: /usr/bin/sshpass
```

If tools are missing:
```
❌ ERROR: 'sshpass' command not found in runner
   sshpass was not installed correctly
❌ DEPLOYMENT FAILED: Missing required SSH tools

Required packages are not available in the runner.
Please check the 'Setup SSH and Environment' step for installation errors.
```

#### Network Connectivity Check
Tests network connectivity to target devices:

**For each Raspberry Pi:**
1. ICMP ping test (1 packet, 2 second timeout)
2. SSH port (22) TCP connection test (3 second timeout)

Example output:
```
2. Checking network connectivity...
Testing Vision Pi (10.1.1.21)...
⚠️  Vision Pi is NOT reachable via ICMP (ping)
❌ Vision Pi SSH port (22) is NOT accessible

Testing Main Pi (10.1.1.20)...
⚠️  Main Pi is NOT reachable via ICMP (ping)
❌ Main Pi SSH port (22) is NOT accessible
```

If hosts are unreachable:
```
❌ DEPLOYMENT FAILED: Target hosts unreachable

The Raspberry Pi devices are not accessible from this runner.

Possible reasons:
  • GitHub Actions runners cannot reach private network IPs (10.1.1.x)
  • This workflow requires a self-hosted runner on the same network
  • Devices may be powered off or network is down
  • Firewall blocking connections

Solutions:
  1. Use a self-hosted GitHub Actions runner on the local network
  2. Set up a VPN connection from GitHub runners to your network
  3. Use SSH tunneling or a bastion host with public IP

Target IPs:
  • Vision Pi: 10.1.1.21
  • Main Pi: 10.1.1.20
```

### 2. Updated Documentation

Updated `docs/DEPLOYMENT_WORKFLOW.md` with:

#### New Process Step
Added "2. Verification (проверка готовности)" section documenting:
- SSH tools check process
- Network connectivity check process
- Example outputs for success and failure cases
- Note that check is skipped in dry-run mode

#### Enhanced Troubleshooting
Added two new troubleshooting sections:

**"Missing SSH tools in runner":**
- Symptoms and error messages
- Step-by-step resolution guide
- Reference to installation step

**"Target hosts unreachable":**
- Detailed explanation of the problem
- Three solution options:
  1. **Self-hosted runner (recommended)** - with complete setup instructions
  2. **VPN tunnel** - brief description
  3. **SSH bastion host** - brief description
- Explanation of why GitHub hosted runners can't reach private IPs

## Files Changed

```
.github/workflows/G-Deploy and Verify.yml  (+110 lines)
docs/DEPLOYMENT_WORKFLOW.md                (+117 lines)
docs/reports/2025-10-29-deployment-workflow-ssh-verification.md (new file)
```

## Benefits

1. **Early Failure Detection:** Workflow fails fast with clear error message instead of attempting all steps
2. **Clear Error Messages:** Distinguishes between SSH tool issues and network connectivity issues
3. **Actionable Guidance:** Provides specific solutions for each problem type
4. **Better UX:** Users understand the problem and know how to fix it
5. **Saves Time:** No need to wait through multiple failed deployment steps

## Testing

Tested locally in GitHub Actions runner environment:

```bash
# Test 1: Verify SSH tools are available
✅ ssh: /usr/bin/ssh
✅ sshpass: /usr/bin/sshpass

# Test 2: Verify network connectivity (expected to fail for private IPs)
❌ Host is NOT reachable via ICMP (expected from GitHub runners)
❌ SSH port (22) is NOT accessible (expected from GitHub runners)
```

The logic correctly identifies:
- SSH tools are installed and working
- Network is unreachable (as expected for private IPs from hosted runners)

## Recommendations

### For Production Use

The workflow currently uses GitHub hosted runners which cannot reach the Raspberry Pi devices. To use this workflow in production:

**Option 1: Self-hosted Runner (Recommended)**

Set up a self-hosted runner on a machine in the same network:

```bash
# 1. Download and extract GitHub Actions runner
curl -o actions-runner-linux-x64-2.319.1.tar.gz -L \
  https://github.com/actions/runner/releases/download/v2.319.1/actions-runner-linux-x64-2.319.1.tar.gz
tar xzf ./actions-runner-linux-x64-2.319.1.tar.gz

# 2. Configure runner (get token from GitHub Settings > Actions > Runners)
./config.sh --url https://github.com/krikz/rob_box_project --token YOUR_TOKEN

# 3. Install and start as service
sudo ./svc.sh install
sudo ./svc.sh start

# 4. Update workflow file
# Change: runs-on: ubuntu-latest
# To:     runs-on: [self-hosted, linux]
```

**Option 2: VPN Solution**

Set up WireGuard or OpenVPN to connect GitHub runner to private network.

**Option 3: Bastion Host**

Deploy a public-facing SSH bastion host to proxy connections to Raspberry Pi devices.

### For Development/Testing

Use dry-run mode to test workflow without actual deployment:

```bash
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=develop \
  -f environment=staging \
  -f registry_source=github \
  -f dry_run=true
```

## Related Documentation

- [Deployment Workflow Guide](../DEPLOYMENT_WORKFLOW.md)
- [CI/CD Pipeline](../CI_CD_PIPELINE.md)
- [GitHub Actions Self-hosted Runners](https://docs.github.com/en/actions/hosting-your-own-runners)

## Implementation Details

### Workflow Step Order

```
1. Checkout repository
2. Setup SSH and Environment
   ├─ Install sshpass
   └─ Configure environment variables
3. Verify SSH Tools and Connectivity  ← NEW
   ├─ Check ssh availability
   ├─ Check sshpass availability
   ├─ Test Vision Pi connectivity
   └─ Test Main Pi connectivity
4. [Vision Pi] Deployment steps...
5. [Main Pi] Deployment steps...
6. Health checks...
7. Summary
```

### Exit Codes

- `0` - All checks passed, continue with deployment
- `1` - SSH tools missing or hosts unreachable, fail workflow

### Conditional Execution

The verification step only runs when `dry_run` is `false`:
```yaml
if: '!inputs.dry_run'
```

This ensures dry-run mode can still execute without requiring actual network access.

## Future Enhancements

Potential improvements for future iterations:

1. **Retry Logic:** Add automatic retries with exponential backoff for transient network issues
2. **Health Monitoring:** Integrate with monitoring system to check if devices are actually online before attempting deployment
3. **Notification System:** Send notifications (Slack/email) when deployment verification fails
4. **SSH Key Authentication:** Move from password-based to key-based authentication for better security
5. **Parallel Checks:** Run Vision Pi and Main Pi connectivity checks in parallel to save time
6. **Custom Timeouts:** Make ping and SSH connection timeouts configurable via workflow inputs

## Conclusion

This enhancement significantly improves the deployment workflow by:
- Providing early detection of connectivity issues
- Distinguishing between different types of failures
- Offering clear, actionable solutions
- Saving time by failing fast
- Improving documentation for troubleshooting

The workflow now clearly communicates when self-hosted runners are needed for deployment to private network devices.

## Reference

- GitHub Issue: Investigation requested for run #18896855146
- PR: copilot/investigate-action-run-issues
- Commit: fe2ea23 (Update deployment workflow documentation)
- Commit: 7ee5ff4 (Add SSH tools and connectivity verification)
