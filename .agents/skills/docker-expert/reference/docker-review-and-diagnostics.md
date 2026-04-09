# Docker Review And Diagnostics

Load this file only during review or when triaging a concrete Docker problem.

## Review checklist

### Dockerfile and build stages
- Dependencies copied before source when cache behavior matters.
- Build and runtime stages are separated when appropriate.
- Final stage contains only necessary artifacts.
- `.dockerignore` excludes bulky or irrelevant files.
- Base image choice matches security and size goals.

### Security
- Container runs as non-root.
- Secrets are not baked into layers or plain env vars.
- Attack surface is minimized.
- Health checks exist when operationally useful.

### Compose and orchestration
- Dependency ordering uses health checks where needed.
- Networks isolate backend services when appropriate.
- Volumes match persistence requirements.
- Resource limits or restart policies exist for production setups.

### Performance and size
- Image avoids unnecessary build tools in runtime.
- Cache strategy is intentional.
- Artifact copying is selective.
- Multi-arch support is added only when required.

## Common issue diagnostics

### Slow builds
Symptoms:
- 10+ minute builds
- cache invalidates too often

Likely causes:
- poor layer ordering
- oversized build context
- no cache strategy

### Security findings
Symptoms:
- vulnerability scan failures
- root execution
- leaked secrets

Likely causes:
- outdated base image
- secrets in env or layers
- default user retained

### Large images
Symptoms:
- runtime image is unexpectedly huge
- deploy/pull time is high

Likely causes:
- build tools left in runtime
- too many copied artifacts
- poor base image choice

### Networking failures
Symptoms:
- service discovery breaks
- container-to-container connectivity fails

Likely causes:
- wrong network topology
- missing health checks for readiness
- service naming or port assumptions

### Dev workflow issues
Symptoms:
- hot reload fails
- debugger cannot attach
- local iteration is slow

Likely causes:
- incorrect bind mounts
- bad target stage selection
- environment mismatch between dev and prod
