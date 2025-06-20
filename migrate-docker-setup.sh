#!/bin/bash

# Migration script for VLM Workspace Docker setup
# This script helps migrate from the old multi-file setup to the new unified setup

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

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

# Check if we're in the right directory
if [ ! -f "Dockerfile" ] || [ ! -f "Makefile" ] || [ ! -f "docker-compose.yml" ]; then
    print_error "New Docker setup files not found. Please run this script from the project root after creating the new files."
    exit 1
fi

print_info "Starting migration to new Docker setup..."

# Create backup directory
BACKUP_DIR=".docker-setup-backup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP_DIR"
print_info "Created backup directory: $BACKUP_DIR"

# Files to backup and remove
OLD_FILES=(
    ".devcontainer/justfile"
    ".devcontainer/build-multiarch.sh"
    ".devcontainer/compose.yml"
    ".devcontainer/compose.multi-arch.yml"
    ".devcontainer/Dockerfile.develop_vlm"
    ".devcontainer/Dockerfile.deploy_vlm_amd64"
    ".devcontainer/Dockerfile.deploy_vlm_arm64"
    ".devcontainer/Dockerfile copy.develop_vlm"
    ".devcontainer/README-multiarch.md"
)

# Backup old files
print_info "Backing up old files..."
for file in "${OLD_FILES[@]}"; do
    if [ -f "$file" ]; then
        cp "$file" "$BACKUP_DIR/"
        print_info "Backed up: $file"
    fi
done

# Ask user for confirmation before removing files
echo ""
print_warning "The following files will be removed:"
for file in "${OLD_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo "  - $file"
    fi
done

echo ""
read -p "Do you want to proceed with removing these files? (y/N): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    print_info "Removing old files..."
    for file in "${OLD_FILES[@]}"; do
        if [ -f "$file" ]; then
            rm "$file"
            print_success "Removed: $file"
        fi
    done
else
    print_info "Skipping file removal. Files are backed up in $BACKUP_DIR"
fi

# Test the new setup
print_info "Testing new Docker setup..."

# Check if Docker buildx is available
if ! docker buildx version >/dev/null 2>&1; then
    print_error "Docker buildx is not available. Please install Docker Desktop or enable buildx."
    exit 1
fi

# Test Makefile
if make help >/dev/null 2>&1; then
    print_success "Makefile is working correctly"
else
    print_error "Makefile test failed"
    exit 1
fi

# Test docker-compose
if docker compose config >/dev/null 2>&1; then
    print_success "docker-compose.yml is valid"
else
    print_error "docker-compose.yml validation failed"
    exit 1
fi

print_success "Migration completed successfully!"

echo ""
print_info "Next steps:"
echo "1. Run 'make setup' to configure Docker buildx"
echo "2. Run 'make dev' to start development environment"
echo "3. Run 'make help' to see all available commands"
echo "4. Check README-Docker.md for detailed documentation"

echo ""
print_info "Command migration reference:"
echo "  Old: just develop_vlm_up     → New: make dev-up"
echo "  Old: just build_arm64        → New: make deploy-build-arm64"
echo "  Old: just build_multiarch    → New: make deploy-build-multiarch"
echo "  Old: ./build-multiarch.sh    → New: make [target]"

echo ""
print_warning "Backup files are stored in: $BACKUP_DIR"
print_warning "You can safely delete this directory after confirming everything works."
