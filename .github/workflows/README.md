# GitHub Actions Workflows

This directory contains all automated workflows for the Rob Box project.

## Quick Reference

### 🚀 Deployment Workflows

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| **L: Deploy and Verify** | Manual | Deploy and verify robot system on both Pis |

### 🏗️ Build Workflows

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| **G: Build All Services** | Manual / Push to main | Build all services on GitHub Actions |
| **G: Build Base Images** | Manual / Called by other workflows | Build base Docker images |
| **G: Build Vision Pi Services** | Called by other workflows | Build Vision Pi services |
| **G: Build Main Pi Services** | Called by other workflows | Build Main Pi services |
| **L: Build All Services** | Manual | Build all services on local build machine |
| **L: Build Base Images** | Called by local workflows | Build base images locally |
| **L: Build Vision Pi Services** | Called by local workflows | Build Vision Pi services locally |
| **L: Build Main Pi Services** | Called by local workflows | Build Main Pi services locally |
| **L: Build Single Service** | Manual | Build a single service on local build machine |

### 🔄 CI/CD Automation

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| **G: Auto-merge Feature to Develop** | Push to feature/* | Build and create PR to develop |
| **G: Auto-merge to Main** | Push to develop | Build all and create PR to main |

### ✅ Validation Workflows

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| **G: Validate Docker Compose** | PR / Push | Validate docker-compose.yaml files |
| **G: Lint Code** | PR / Push | Run code linters |
| **G: Run Tests** | PR / Push | Run test suites |

## Naming Convention

- **G-** prefix = GitHub Actions runners (cloud-based)
- **L-** prefix = Local self-hosted runners (build machine)

## Common Use Cases

### Deploy to Production

```bash
gh workflow run "L-Deploy and Verify.yml" \
  -f branch=main \
  -f environment=production
```

### Deploy to Staging for Testing

```bash
gh workflow run "L-Deploy and Verify.yml" \
  -f branch=develop \
  -f environment=staging
```

### Build All Services Manually

```bash
gh workflow run "L-Build All Services.yml"
```

### Build Single Service Locally

```bash
gh workflow run "L-Build Single Service.yml" \
  -f branch=develop \
  -f pi_type=vision \
  -f service=voice-assistant \
  -f push_to_registry=true
```

### Test Workflow Without Deployment

```bash
gh workflow run "L-Deploy and Verify.yml" \
  -f branch=develop \
  -f environment=test \
  -f dry_run=true
```

## Documentation

- **[Deployment Workflow Guide](../../docs/DEPLOYMENT_WORKFLOW.md)** - Complete deployment workflow documentation
- **[CI/CD Pipeline](../../docs/CI_CD_PIPELINE.md)** - Overall CI/CD architecture
- **[Docker Standards](../../docs/development/DOCKER_STANDARDS.md)** - Docker best practices
- **[Agent Guide](../../docs/development/AGENT_GUIDE.md)** - Development guide

## Workflow Statuses

Check workflow status:
```bash
# List recent runs
gh run list --limit 10

# Watch current run
gh run watch

# View specific run
gh run view <run-id> --log
```

## Troubleshooting

### Workflow Failed

1. Check the logs in GitHub Actions UI
2. Look for automatically created issues
3. Review the workflow file for any syntax errors

### SSH Connection Issues

- Ensure Raspberry Pis are powered on and connected to network
- Verify IPs are correct (Vision: 10.1.1.21, Main: 10.1.1.20)
- Check that SSH password is 'open'

### Docker Images Not Found

- Verify images were built successfully in previous workflows
- Check image tags match the selected environment
- Ensure ghcr.io registry is accessible

## Support

For issues or questions:
- Check existing [Issues](https://github.com/krikz/rob_box_project/issues)
- Review [Documentation](../../docs/)
- Create new issue with label `ci/cd`
