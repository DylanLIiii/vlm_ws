#!/bin/bash

# Multi-architecture build script for VLM workspace
# This script helps build and deploy containers for both AMD64 and ARM64 architectures

set -e

# Configuration
IMAGE_REGISTRY="${IMAGE_REGISTRY:-192.168.31.199:8000}"
IMAGE_NAME="${IMAGE_NAME:-vlm_ws}"
IMAGE_TAG="${IMAGE_TAG:-latest}"
DOCKERFILE="${DOCKERFILE:-Dockerfile.multi-arch}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Docker buildx is available
check_buildx() {
    if ! docker buildx version &> /dev/null; then
        print_error "Docker buildx is not available. Please install Docker Desktop or enable buildx."
        exit 1
    fi
    print_success "Docker buildx is available"
}

# Setup buildx builder
setup_builder() {
    print_info "Setting up buildx builder for multi-architecture builds..."
    
    # Create a new builder instance if it doesn't exist
    if ! docker buildx ls | grep -q "multiarch"; then
        docker buildx create --name multiarch --driver docker-container --use
        print_success "Created multiarch builder"
    else
        docker buildx use multiarch
        print_info "Using existing multiarch builder"
    fi
    
    # Bootstrap the builder
    docker buildx inspect --bootstrap
    print_success "Buildx builder is ready"
}

# Build function
build_image() {
    local platforms="$1"
    local tag_suffix="$2"
    local push_flag="$3"
    
    local full_image_name="${IMAGE_REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}${tag_suffix}"
    
    print_info "Building image: ${full_image_name}"
    print_info "Platforms: ${platforms}"
    
    local build_args=(
        --platform "${platforms}"
        --build-arg "USER_ID=$(id -u)"
        --build-arg "GROUP_ID=$(id -g)"
        --build-arg "USER_NAME=${USER}"
        -f "${DOCKERFILE}"
        -t "${full_image_name}"
    )
    
    if [[ "${push_flag}" == "true" ]]; then
        build_args+=(--push)
        print_info "Will push to registry after build"
    else
        build_args+=(--load)
        print_info "Will load to local Docker after build"
    fi
    
    docker buildx build "${build_args[@]}" .
    
    print_success "Successfully built ${full_image_name}"
}

# Show usage
usage() {
    cat << EOF
Usage: $0 [COMMAND] [OPTIONS]

Commands:
    setup           Setup buildx builder for multi-architecture builds
    build-dev       Build AMD64 image for development (loads locally)
    build-arm64     Build ARM64 image for edge deployment (pushes to registry)
    build-all       Build both AMD64 and ARM64 images (pushes to registry)
    inspect         Inspect the multi-architecture image
    clean           Clean up buildx builder
    help            Show this help message

Environment Variables:
    IMAGE_REGISTRY  Docker registry (default: 192.168.31.199:8000)
    IMAGE_NAME      Image name (default: vlm_ws)
    IMAGE_TAG       Image tag (default: latest)
    DOCKERFILE      Dockerfile to use (default: Dockerfile.multi-arch)

Examples:
    $0 setup                    # Setup buildx builder
    $0 build-dev                # Build for development (AMD64)
    $0 build-arm64              # Build for ARM64 edge devices
    $0 build-all                # Build for both architectures
    
    IMAGE_TAG=v1.0 $0 build-all # Build with custom tag

EOF
}

# Main command handling
case "${1:-help}" in
    setup)
        check_buildx
        setup_builder
        ;;
    
    build-dev)
        check_buildx
        setup_builder
        build_image "linux/amd64" "-amd64" "false"
        ;;
    
    build-arm64)
        check_buildx
        setup_builder
        build_image "linux/arm64" "-arm64" "true"
        ;;
    
    build-all)
        check_buildx
        setup_builder
        build_image "linux/amd64,linux/arm64" "" "true"
        ;;
    
    inspect)
        if [[ -z "$2" ]]; then
            local image_to_inspect="${IMAGE_REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}"
        else
            local image_to_inspect="$2"
        fi
        print_info "Inspecting image: ${image_to_inspect}"
        docker buildx imagetools inspect "${image_to_inspect}"
        ;;
    
    clean)
        print_info "Cleaning up buildx builder..."
        docker buildx rm multiarch || true
        print_success "Cleanup completed"
        ;;
    
    help|--help|-h)
        usage
        ;;
    
    *)
        print_error "Unknown command: $1"
        usage
        exit 1
        ;;
esac
