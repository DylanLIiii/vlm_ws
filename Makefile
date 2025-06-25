# VLM Workspace Multi-Architecture Build System
# Supports development on AMD64 and deployment on ARM64

# Configuration
REGISTRY ?= 192.168.31.199:8000
IMAGE_NAME ?= algo/deploy_vlm
TAG ?= latest
USER_ID ?= $(shell id -u)
GROUP_ID ?= $(shell id -g)
USER_NAME ?= $(shell echo $$USER)

# Docker buildx builder name
BUILDER_NAME = vlm-multiarch

# Colors for output
RED = \033[0;31m
GREEN = \033[0;32m
YELLOW = \033[1;33m
BLUE = \033[0;34m
NC = \033[0m

# Helper functions
define print_info
	@echo -e "$(BLUE)[INFO]$(NC) $(1)"
endef

define print_success
	@echo -e "$(GREEN)[SUCCESS]$(NC) $(1)"
endef

define print_warning
	@echo -e "$(YELLOW)[WARNING]$(NC) $(1)"
endef

.PHONY: help
help: ## Show this help message
	@echo "VLM Workspace Multi-Architecture Build System"
	@echo ""
	@echo "Available targets:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

# Setup and cleanup
.PHONY: setup
setup: ## Setup Docker buildx for multi-architecture builds | To build without network as host. I think it can works. Just remove --driver-opt network host
	$(call print_info,"Setting up Docker buildx...")
	@echo '[registry."192.168.31.199:8000"]\n  http = true\n  insecure = true' > buildkitd.toml
	@docker buildx create --name $(BUILDER_NAME) --driver docker-container --use --config buildkitd.toml --driver-opt network=host 2>/dev/null || \
		docker buildx use $(BUILDER_NAME) 2>/dev/null || true
	@docker buildx inspect --bootstrap
	$(call print_success,"Buildx setup completed")

.PHONY: force-setup
force-setup: ## Force remove and recreate Docker buildx builder
	$(call print_info,"Forcing buildx setup...")
	@docker buildx rm $(BUILDER_NAME) 2>/dev/null || true
	@echo '[registry."192.168.31.199:8000"]\n  http = true\n  insecure = true' > buildkitd.toml
	@docker buildx create --name $(BUILDER_NAME) --driver docker-container --use --config buildkitd.toml --driver-opt network=host
	@docker buildx inspect --bootstrap
	$(call print_success,"Buildx force setup completed")

.PHONY: clean
clean: ## Clean up Docker buildx builder
	$(call print_info,"Cleaning up buildx builder...")
	@docker buildx rm $(BUILDER_NAME) 2>/dev/null || true
	$(call print_success,"Cleanup completed")

# Development targets (AMD64)
.PHONY: dev-build
dev-build: setup ## Build development image (AMD64, loads locally)
	$(call print_info,"Building development image for AMD64...")
	@docker buildx build \
		--builder $(BUILDER_NAME) \
		--platform linux/amd64 \
		--build-arg USER_ID=$(USER_ID) \
		--build-arg GROUP_ID=$(GROUP_ID) \
		--build-arg USER_NAME=$(USER_NAME) \
		--build-arg BUILD_TYPE=development \
		--build-arg TARGETARCH=amd64 \
		-t $(IMAGE_NAME):dev-$(TAG) \
		--load \
		.
	$(call print_success,"Development image built successfully")

.PHONY: dev-up
dev-up: ## Start development container
	$(call print_info,"Starting development container...")
	@docker compose up -d develop
	$(call print_success,"Development container started")

.PHONY: dev-down
dev-down: ## Stop development container
	$(call print_info,"Stopping development container...")
	@docker compose down
	$(call print_success,"Development container stopped")

.PHONY: dev-shell
dev-shell: ## Get shell in development container
	@docker compose exec develop bash

.PHONY: dev-logs
dev-logs: ## Show development container logs
	@docker compose logs -f develop

# Deployment targets
.PHONY: deploy-build-amd64
deploy-build-amd64: setup ## Build deployment image for AMD64
	$(call print_info,"Building deployment image for AMD64...")
	@docker buildx build \
		--builder $(BUILDER_NAME) \
		--platform linux/amd64 \
		--build-arg BUILD_TYPE=deployment \
		--build-arg TARGETARCH=amd64 \
		-t $(REGISTRY)/$(IMAGE_NAME):$(TAG)-amd64 \
		.
	$(call print_success,"AMD64 deployment image built")

.PHONY: deploy-build-arm64
deploy-build-arm64: setup ## Build deployment image for ARM64
	$(call print_info,"Building deployment image for ARM64...")
	@docker buildx build \
		--builder $(BUILDER_NAME) \
		--platform linux/arm64 \
		--build-arg BUILD_TYPE=deployment \
		--build-arg TARGETARCH=arm64 \
		-t $(REGISTRY)/$(IMAGE_NAME):$(TAG)-arm64 \
		.
	$(call print_success,"ARM64 deployment image built")

.PHONY: deploy-build-multiarch
deploy-build-multiarch: setup ## Build multi-architecture deployment image
	$(call print_info,"Building multi-architecture deployment image...")
	@docker buildx build \
		--builder $(BUILDER_NAME) \
		--platform linux/amd64,linux/arm64 \
		--build-arg BUILD_TYPE=deployment \
		-t $(REGISTRY)/$(IMAGE_NAME):$(TAG) \
		.
	$(call print_success,"Multi-architecture deployment image built")

# Push targets
.PHONY: push-amd64
push-amd64: deploy-build-amd64 ## Build and push AMD64 deployment image
	$(call print_info,"Pushing AMD64 deployment image...")
	@docker buildx build \
		--builder $(BUILDER_NAME) \
		--platform linux/amd64 \
		--build-arg BUILD_TYPE=deployment \
		--build-arg TARGETARCH=amd64 \
		-t $(REGISTRY)/$(IMAGE_NAME):$(TAG)-amd64 \
		--push \
		.
	$(call print_success,"AMD64 deployment image pushed to registry")

.PHONY: push-arm64
push-arm64: deploy-build-arm64 ## Build and push ARM64 deployment image
	$(call print_info,"Pushing ARM64 deployment image...")
	@docker buildx build \
		--builder $(BUILDER_NAME) \
		--platform linux/arm64 \
		--build-arg BUILD_TYPE=deployment \
		--build-arg TARGETARCH=arm64 \
		-t $(REGISTRY)/$(IMAGE_NAME):$(TAG)-arm64 \
		--push \
		.
	$(call print_success,"ARM64 deployment image pushed to registry")

.PHONY: push-multiarch
push-multiarch: setup ## Build and push multi-architecture deployment image
	$(call print_info,"Building and pushing multi-architecture deployment image...")
	@docker buildx build \
		--builder $(BUILDER_NAME) \
		--platform linux/amd64,linux/arm64 \
		--build-arg BUILD_TYPE=deployment \
		-t $(REGISTRY)/$(IMAGE_NAME):$(TAG) \
		--push \
		.
	$(call print_success,"Multi-architecture deployment image pushed to registry")

.PHONY: deploy-shell-arm64
deploy-shell-arm64: ## Get shell in development container
	@docker compose exec deploy-arm64 bash

# Deployment container management
.PHONY: deploy-run-amd64
deploy-run-amd64: ## Run deployment container (AMD64)
	$(call print_info,"Running AMD64 deployment container...")
	@docker compose run --rm deploy-amd64

.PHONY: deploy-run-arm64
deploy-run-arm64: ## Run deployment container (ARM64)
	$(call print_info,"Running ARM64 deployment container...")
	@docker compose run --rm deploy-arm64

.PHONY: deploy-up-amd64
deploy-up-amd64: ## Start AMD64 deployment container
	@docker compose up -d deploy-amd64

.PHONY: deploy-up-arm64
deploy-up-arm64: ## Start ARM64 deployment container
	@docker compose up -d deploy-arm64

.PHONY: deploy-pull-arm64
deploy-pull-arm64: ## Pull latest ARM64 deployment image and run in background
	$(call print_info,"Pulling latest ARM64 deployment image...")
	@docker pull $(REGISTRY)/$(IMAGE_NAME):$(TAG)-arm64
	$(call print_info,"Starting ARM64 deployment container...")
	@docker compose up -d deploy-arm64
	$(call print_success,"ARM64 deployment container started")

.PHONY: deploy-shell
deploy-shell: ## Enter shell in ARM64 deployment container
	@docker compose exec deploy-arm64 bash

# Utility targets
.PHONY: inspect
inspect: ## Inspect multi-architecture image
	$(call print_info,"Inspecting image: $(REGISTRY)/$(IMAGE_NAME):$(TAG)")
	@docker buildx imagetools inspect $(REGISTRY)/$(IMAGE_NAME):$(TAG)

.PHONY: list-builders
list-builders: ## List Docker buildx builders
	@docker buildx ls

.PHONY: prune
prune: ## Remove unused Docker resources
	$(call print_warning,"Removing unused Docker resources...")
	@docker system prune -f
	@docker buildx prune -f
	$(call print_success,"Docker cleanup completed")

# Quick development workflow
.PHONY: dev
dev: dev-build dev-up ## Quick start: build and run development environment

.PHONY: dev-rebuild
dev-rebuild: dev-down dev-build dev-up ## Rebuild and restart development environment

# Quick deployment workflow
.PHONY: deploy-all
deploy-all: deploy-build-multiarch ## Build and push multi-architecture deployment images

# Environment info
.PHONY: info
info: ## Show build environment information
	@echo "Build Configuration:"
	@echo "  Registry: $(REGISTRY)"
	@echo "  Image: $(IMAGE_NAME)"
	@echo "  Tag: $(TAG)"
	@echo "  User ID: $(USER_ID)"
	@echo "  Group ID: $(GROUP_ID)"
	@echo "  User Name: $(USER_NAME)"
	@echo "  Builder: $(BUILDER_NAME)"
