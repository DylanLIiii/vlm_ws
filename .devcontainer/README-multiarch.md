# Multi-Architecture Deployment Guide

This guide explains how to build and deploy the VLM workspace for both AMD64 (development) and ARM64 (edge devices) architectures.

## Overview

The setup provides:
- **AMD64 images**: For development on your current machine
- **ARM64 images**: For deployment on edge devices (Raspberry Pi, NVIDIA Jetson, etc.)
- **Multi-arch support**: Using Docker buildx for cross-platform builds

## Files Structure

```
.devcontainer/
├── devcontainer.json              # Original dev config (AMD64)
├── devcontainer.deploy.json       # Deployment config (multi-arch)
├── compose.yml                    # Original compose (AMD64)
├── compose.multi-arch.yml         # Multi-arch compose
├── Dockerfile.develop_vlm         # Original development Dockerfile
├── Dockerfile.multi-arch          # Multi-architecture Dockerfile
├── sources.list                   # AMD64 apt sources
├── sources_amd64.list            # AMD64 apt sources (explicit)
├── sources_arm64.list            # ARM64 apt sources
├── build-multiarch.sh            # Build script
└── justfile                      # Updated with multi-arch commands
```

## Quick Start

### 1. Setup Docker Buildx

```bash
# Using the build script
./.devcontainer/build-multiarch.sh setup

# Or using justfile
just setup_buildx
```

### 2. Development (AMD64)

For development, continue using your existing setup:

```bash
# Start development container
just develop_vlm_up

# Or manually
docker compose up -d develop_vlm
```

### 3. Build for ARM64 Deployment

```bash
# Build ARM64 image for edge devices
./.devcontainer/build-multiarch.sh build-arm64

# Or using justfile
just build_arm64

# Build both architectures
./.devcontainer/build-multiarch.sh build-all
```

## Detailed Usage

### Build Script Commands

```bash
# Setup buildx builder
./build-multiarch.sh setup

# Build for development (AMD64, loads locally)
./build-multiarch.sh build-dev

# Build for ARM64 edge devices (pushes to registry)
./build-multiarch.sh build-arm64

# Build multi-architecture image (both AMD64 and ARM64)
./build-multiarch.sh build-all

# Inspect built image
./build-multiarch.sh inspect [image-name]

# Clean up buildx builder
./build-multiarch.sh clean
```

### Justfile Commands

```bash
# Development commands (AMD64)
just develop_vlm           # Run development container
just develop_vlm_up        # Start development container
just develop_vlm_down      # Stop development container
just get_into_vlm          # Get shell in development container

# Multi-arch commands
just setup_buildx          # Setup buildx
just build_multiarch       # Build both architectures
just build_arm64           # Build ARM64 only
just build_amd64           # Build AMD64 only

# Deployment commands
just deploy_vlm            # Run deployment container
just deploy_vlm_up         # Start deployment container
just deploy_vlm_down       # Stop deployment container
just get_into_deploy       # Get shell in deployment container
```

### Environment Variables

You can customize the build process using environment variables:

```bash
# Custom registry
export IMAGE_REGISTRY="your-registry.com"

# Custom image name and tag
export IMAGE_NAME="your-vlm-workspace"
export IMAGE_TAG="v1.0"

# Then build
./build-multiarch.sh build-all
```

## Deployment on Edge Devices

### 1. On Your Development Machine

Build and push the ARM64 image:

```bash
# Build ARM64 image and push to registry
./build-multiarch.sh build-arm64
```

### 2. On Edge Device (ARM64)

Pull and run the image:

```bash
# Pull the ARM64 image
docker pull 192.168.31.199:8000/vlm_ws:latest-arm64

# Run the container
docker run -it --rm \
  --network host \
  -v /path/to/your/workspace:/home/heng.li/repo/vlm_ws \
  -e ROS_DISTRO=humble \
  192.168.31.199:8000/vlm_ws:latest-arm64
```

### 3. Using Docker Compose on Edge Device

Copy `compose.multi-arch.yml` to your edge device and run:

```bash
# Adjust volume paths in compose.multi-arch.yml for your edge device
# Then start the deployment container
docker compose -f compose.multi-arch.yml up -d deploy_vlm
```

## Architecture-Specific Optimizations

The `Dockerfile.multi-arch` includes architecture-specific optimizations:

- **ARM64**: Uses `ubuntu-ports` repositories for better ARM64 support
- **AMD64**: Uses standard Ubuntu repositories
- **Conditional installs**: Some tools are only installed on specific architectures

## Troubleshooting

### 1. Buildx Not Available

```bash
# Install Docker Desktop or enable buildx
docker buildx version
```

### 2. Cross-compilation Issues

```bash
# Enable qemu for cross-platform builds
docker run --privileged --rm tonistiigi/binfmt --install all
```

### 3. Registry Push Issues

```bash
# Login to your registry
docker login 192.168.31.199:8000
```

### 4. Architecture Detection

```bash
# Check current platform
docker info | grep -i platform

# Inspect multi-arch image
docker buildx imagetools inspect 192.168.31.199:8000/vlm_ws:latest
```

## Best Practices

1. **Development**: Use the original `devcontainer.json` and `compose.yml` for development
2. **Testing**: Test ARM64 builds locally using emulation before deploying
3. **Registry**: Use a private registry for storing multi-arch images
4. **Tagging**: Use clear tags to distinguish architectures (`-amd64`, `-arm64`)
5. **CI/CD**: Integrate multi-arch builds into your CI/CD pipeline

## Next Steps

1. Test the multi-arch build process
2. Deploy to your ARM64 edge devices
3. Set up automated builds in CI/CD
4. Consider using manifest lists for seamless multi-arch images
