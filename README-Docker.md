# VLM Workspace Multi-Architecture Docker Setup

This document describes the redesigned Docker development and deployment workflow that supports both AMD64 (development) and ARM64 (deployment) architectures with minimal redundancy.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Single Dockerfile                        │
│  ┌─────────────────┐              ┌─────────────────┐      │
│  │   Development   │              │   Deployment    │      │
│  │     (AMD64)     │              │  (AMD64/ARM64)  │      │
│  │                 │              │                 │      │
│  │ • User creation │              │ • Root user     │      │
│  │ • Dev tools     │              │ • Minimal deps  │      │
│  │ • Local build   │              │ • Registry push │      │
│  └─────────────────┘              └─────────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

## Key Improvements

1. **Single Dockerfile**: Unified multi-architecture Dockerfile with conditional logic
2. **Makefile**: Replaces justfile and build scripts with consistent interface
3. **Simplified Compose**: Single docker-compose.yml for all scenarios
4. **Clean Architecture**: Separate development and deployment concerns
5. **Efficient Builds**: Shared layers between architectures

## Quick Start

### Prerequisites

```bash
# Ensure Docker buildx is available
docker buildx version

# Setup multi-architecture builder
make setup
```

### Development Workflow

```bash
# Start development environment (builds if needed)
make dev

# Or step by step:
make dev-build    # Build development image
make dev-up       # Start container
make dev-shell    # Get shell access

# Stop development environment
make dev-down
```

### Deployment Workflow

```bash
# Build for specific architecture
make deploy-build-amd64    # Build AMD64 deployment image
make deploy-build-arm64    # Build ARM64 deployment image

# Build multi-architecture image
make deploy-build-multiarch

# Run deployment containers
make deploy-run-amd64      # Run AMD64 deployment
make deploy-run-arm64      # Run ARM64 deployment
```

## Makefile Targets

### Setup and Cleanup
- `make setup` - Setup Docker buildx for multi-architecture builds
- `make clean` - Clean up Docker buildx builder
- `make prune` - Remove unused Docker resources

### Development
- `make dev` - Quick start: build and run development environment
- `make dev-build` - Build development image (AMD64, loads locally)
- `make dev-up` - Start development container
- `make dev-down` - Stop development container
- `make dev-shell` - Get shell in development container
- `make dev-logs` - Show development container logs
- `make dev-rebuild` - Rebuild and restart development environment

### Deployment
- `make deploy-build-amd64` - Build deployment image for AMD64
- `make deploy-build-arm64` - Build deployment image for ARM64
- `make deploy-build-multiarch` - Build multi-architecture deployment image
- `make deploy-run-amd64` - Run deployment container (AMD64)
- `make deploy-run-arm64` - Run deployment container (ARM64)
- `make deploy-up-amd64` - Start AMD64 deployment container
- `make deploy-up-arm64` - Start ARM64 deployment container

### Utilities
- `make inspect` - Inspect multi-architecture image
- `make list-builders` - List Docker buildx builders
- `make info` - Show build environment information
- `make help` - Show all available targets

## Configuration

### Environment Variables

```bash
# Registry configuration
export REGISTRY=192.168.31.199:8000
export IMAGE_NAME=vlm_ws
export TAG=latest

# User configuration (auto-detected)
export USER_ID=$(id -u)
export GROUP_ID=$(id -g)
export USER_NAME=$USER

# Runtime configuration
export HOST_IP=192.168.1.100
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/profile.xml
```

### Dockerfile Build Arguments

The unified Dockerfile supports these build arguments:

- `TARGETARCH`: Target architecture (amd64/arm64)
- `USER_ID`: User ID for development builds
- `GROUP_ID`: Group ID for development builds  
- `USER_NAME`: Username for development builds
- `BUILD_TYPE`: Build type (development/deployment)

## File Structure

```
.
├── Dockerfile                    # Unified multi-architecture Dockerfile
├── docker-compose.yml           # Simplified compose configuration
├── Makefile                     # Build system (replaces justfile)
├── README-Docker.md             # This documentation
└── .devcontainer/
    ├── devcontainer.json        # Updated VS Code dev container config
    ├── sources_amd64.list       # AMD64 apt sources
    └── sources_arm64.list       # ARM64 apt sources
```

## Migration from Old Setup

### Removed Files
- `.devcontainer/justfile` → Replaced by `Makefile`
- `.devcontainer/build-multiarch.sh` → Replaced by `Makefile`
- `.devcontainer/compose.yml` → Replaced by `docker-compose.yml`
- `.devcontainer/compose.multi-arch.yml` → Replaced by `docker-compose.yml`
- `.devcontainer/Dockerfile.develop_vlm` → Replaced by `Dockerfile`
- `.devcontainer/Dockerfile.deploy_vlm_*` → Replaced by `Dockerfile`

### Command Migration

| Old Command | New Command |
|-------------|-------------|
| `just develop_vlm_up` | `make dev-up` |
| `just build_arm64` | `make deploy-build-arm64` |
| `just build_multiarch` | `make deploy-build-multiarch` |
| `./build-multiarch.sh setup` | `make setup` |
| `./build-multiarch.sh build-all` | `make deploy-build-multiarch` |

## VS Code Development

The `.devcontainer/devcontainer.json` is updated to use the new unified setup:

- Uses the main `docker-compose.yml`
- Points to the `develop` service
- Automatically runs `make info` after container creation

## Troubleshooting

### Common Issues

1. **Buildx not available**: Install Docker Desktop or enable buildx
2. **Permission denied**: Ensure user is in docker group
3. **Registry push fails**: Check registry credentials and network access
4. **Architecture mismatch**: Verify TARGETARCH build argument

### Debug Commands

```bash
# Check buildx status
make list-builders

# Inspect built image
make inspect

# Show configuration
make info

# Clean up and restart
make clean setup
```
