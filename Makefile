# ==========================================================================
# Makefile: Multi-arch, multi-stage build, cross-compile, and deploy
# 
# Images:
# 	dev-amd64		Full dev image (workstation, x86_64)
# 	dev-arm64		Full dev image (cross-build via QEMU, arm64)
# 	runtime-arm64	Slim runtime image (drone, arm64)
# ==========================================================================

SHELL := /bin/bash
.DEFAULT_GOAL := help
export LC_ALL := C

# Paths
PROJECT_DIR := $(shell cd "$(dir $(lastword $(MAKEFILE_LIST)))" && pwd)
DOCKER_DIR := $(PROJECT_DIR)/docker

IMAGE_NAME := voxl-smip

# Onboard ros2 workspace
ROS2_WS ?= /smip_uav_ws
DEPLOY_DIR := $(DOCKER_DIR)/deploy
ROS2_INSTALL := $(DEPLOY_DIR)/$(ROS2_WS)/install
DATA_LOG := $(DEPLOY_DIR)/data_log
DATA_PULL := $(PROJECT_DIR)/data
IMAGE_TARBALL := $(DOCKER_DIR)/smip-runtime-arm64.tar.gz

# SSH Connection (override with .env file)
VOXL_USER ?= root
VOXL_HOST ?= 10.42.0.184
VOXL_DIR ?= /voxl_docker

-include ${PROJECT_DIR}/.env
export

PKGS ?=

.PHONY: help
help:
	@echo "PROJECT_DIR: $(PROJECT_DIR)"
	@echo "DOCKER DIR: $(DOCKER_DIR)"
	@echo ""
	@echo "Usage: make <target>"
	@echo ""
	@echo "--- SETUP ---"
	@echo " setup-build-tools	Setup QEMU user-static and smip-multiarch-builder for arm64 emulation"
	@echo ""
	@echo "--- BUILD IMAGES ---"
	@echo " build-image-deps		Build only the dependency base stage"
	@echo " build-image-dev		Build the full dev image (native x86_64)"
	@echo " build-image-cross		Build the full dev image for arm64 via QEMU"
	@echo " build-image-runtime		Build the slim runtime image for arm64"
	@echo " clean-images			Erase build images, containers, builders, and artifacts"
	@echo ""
	@echo "--- DEVELOPMENT ---"
	@echo " dev-shell			Open a shell in the native x86 dev container"
	@echo " cross-shell			Open a shell in the arm64 QEMU dev container"
	@echo " build-ws-dev			Run colcon build in the x86_64 container. Optional PKGS=\"pkg1 pkg2\""
	@echo " build-ws-cross			Run colcon build in the arm64 container. Optional PKGS=\"pkg1 pkg2\""
	@echo ""
	@echo "--- DEPLOY TO DRONE ---"
	@echo " deploy				Extract cross-built arm64 install and rsync to drone"
	@echo " clean-deploy			Remove the local deploy directory"
	@echo " runtime-export			Save the slim runtime image to a .tar.gz file"
	@echo " runtime-deploy			Transfer the runtime image .tar.gz to drone and load it"
	@echo ""
	@echo "--- DRONE OPERATIONS ---"
	@echo " voxl-access-setup		Copy SSH key to drone for passwordless access"
	@echo " voxl-start			Start the voxl-drone container"
	@echo " voxl-shell			Attach to the running voxl-drone container"
	@echo " voxl-logs			Show voxl-drone container logs"
	@echo " voxl-stop			Stop the voxl-drone container"
	@echo " voxl-clean			Prune containers, images, volumes, and build cache on drone"
	@echo " voxl-pull-data			Pull recorded data from drone data_log to local data_log"


# =================== SETUP =======================
BUILDER := smip-multiarch-builder

.PHONY: setup-build-tools
setup-build-tools:
	@echo "==> Installing QEMU user-static for multi-arch support"
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
	@echo "==> Creating buildx builder..."
	docker buildx create --name $(BUILDER) --driver docker-container --use 2>/dev/null || docker buildx use $(BUILDER)
	docker buildx inspect --bootstrap
	@echo ""
	@echo "==> Done. You can now build arm64 images on this x86 machine..."


# =================== BUILD IMAGES =======================
.PHONY: build-deps
build-image-deps:
	@echo "==> Building dependency base image..."
	docker buildx build --builder $(BUILDER) \
		--platform linux/amd64 \
		--target smip-dev \
		-f "$(DOCKER_DIR)/Dockerfile" \
		-t "$(IMAGE_NAME):dev-amd64" \
		--load \
		"$(DOCKER_DIR)"
	@echo "==> Built: $(IMAGE_NAME):dev-amd64"
	@echo "	Image size:"
	@docker images "$(IMAGE_NAME):dev-amd64" --format "	{{.Size}}"

.PHONY: build-image-dev
build-image-dev:
	@echo "==> Building dev image for native x86_64..."
	docker buildx build --builder $(BUILDER) \
		--platform linux/amd64 \
		--target smip-dev \
		-f "$(DOCKER_DIR)/Dockerfile" \
		-t "$(IMAGE_NAME):dev-amd64" \
		--load \
		"$(DOCKER_DIR)"
	@echo "==> Built: $(IMAGE_NAME):dev-amd64"
	@echo "	Image size:"
	@docker images "$(IMAGE_NAME):dev-amd64" --format "	{{.Size}}"

.PHONY: build-image-cross
build-image-cross:
	@echo "==> Building dev image for arm64 via QEMU..."
	docker buildx build --builder $(BUILDER) \
		--platform linux/arm64 \
		--target smip-dev \
		-f "$(DOCKER_DIR)/Dockerfile" \
		-t "$(IMAGE_NAME):dev-arm64" \
		--load \
		"$(DOCKER_DIR)"
	@echo "==> Built: $(IMAGE_NAME):dev-arm64"
	@echo "	Image size:"
	@docker images "$(IMAGE_NAME):dev-arm64" --format "	{{.Size}}"

.PHONY: build-image-runtime
build-image-runtime:
	@echo "==> Building runtime image for arm64 (Onboard Image)..."
	docker buildx build --builder $(BUILDER) \
		--platform linux/arm64 \
		--target smip-runtime \
		-f "$(DOCKER_DIR)/Dockerfile" \
		-t "$(IMAGE_NAME):runtime-arm64" \
		--load \
		"$(DOCKER_DIR)"
	@echo "==> Built: $(IMAGE_NAME):runtime-arm64"
	@echo "	Image size:"
	@docker images "$(IMAGE_NAME):runtime-arm64" --format "	{{.Size}}"

.PHONY: clean-images
clean-images:
	@echo "==> This will remove all $(IMAGE_NAME) images, containers, volumes, build cache, and the $(BUILDER) builder."
	@read -p "    Continue? [y/N] " ans && [ "$${ans}" = "y" ] || [ "$${ans}" = "Y" ] || (echo "Aborted."; exit 1)

	@echo "==> Stopping project containers and removing mounted volumes..."
	@docker compose -f "$(DOCKER_DIR)/workstation-compose.yml" down -v --remove-orphans || true
	@echo "==> Removing project images..."
	@docker images --filter "reference=$(IMAGE_NAME):*" -q | xargs -r docker rmi -f || true
	@echo "==> Pruning build cache for $(BUILDER)..."
	@docker buildx prune --builder $(BUILDER) -f || true
	@echo "==> Removing $(BUILDER) buildx builder..."
	@docker buildx stop $(BUILDER) || true
	@docker buildx rm $(BUILDER) || true
	@echo "==> Removing exported image tarball (if any)..."
	@rm -f "$(PROJECT_DIR)/voxl-runtime-arm64.tar.gz"
	@echo ""
	@echo "==> Clean complete!"


# =================== DEVELOPMENT =======================
.PHONY: dev-shell
dev-shell:
	@echo "==> Starting native x86_64 dev container..."
	docker compose -f "$(DOCKER_DIR)/workstation-compose.yml" run --rm dev-amd64

.PHONY: cross-shell
cross-shell:
	@echo "==> Starting arm64 cross-build container (QEMU)..."
	docker compose -f "$(DOCKER_DIR)/workstation-compose.yml" run --rm dev-arm64

.PHONY: build-ws-dev
build-ws-dev:
	$(eval COLCON_ARGS := $(if $(PKGS),--packages-select $(PKGS)))
	@echo "==> Building ros2 workspace in native x86_64 dev container..."
	docker compose -f "$(DOCKER_DIR)/workstation-compose.yml" run --rm dev-amd64 \
		bash -c "source /opt/ros/humble/setup.bash && cd $(ROS2_WS) && colcon build $(COLCON_ARGS)"

.PHONY: build-ws-cross
build-ws-cross:
	$(eval COLCON_ARGS := $(if $(PKGS),--packages-select $(PKGS)))
	@echo "==> Building ros2 workspace in arm64 container (QEMU)..."
	docker compose -f "$(DOCKER_DIR)/workstation-compose.yml" run --rm dev-arm64 \
		bash -c "source /opt/ros/humble/setup.bash && cd $(ROS2_WS) && colcon build $(COLCON_ARGS) --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DCMAKE_INSTALL_DO_STRIP=ON' --event-handlers console_direct+"
	@echo ""
	@echo "==> ARM64 binaries built. Run 'make deploy' to deploy to drone."


# =================== DEPLOYMENT =======================
.PHONY: deploy
deploy:
	@echo "==> Preparing deploy directory..."
	@mkdir -p "$(ROS2_INSTALL)"

	@sudo chown -R $$(id -u):$$(id -g) "$(DEPLOY_DIR)"

	@echo "==> Extracting ARM64 install from smip-cross-install volume"
	@docker compose -f "$(DOCKER_DIR)/workstation-compose.yml" run --rm extract || true

	@if [ -z "$$(ls -A "$(ROS2_INSTALL)" 2>/dev/null)" ]; then \
		echo "Install directory is empty - Have you run build-ws-cross?"; \
		exit 1; \
	else \
		echo "==> Extracted!"; \
	fi

	@echo ""
	@echo "==> Stopping voxl runtime container (if running)..."
	@ssh "$(VOXL_USER)@$(VOXL_HOST)" \
		"[ -f $(VOXL_DIR)/voxl-compose.yml ] && cd $(VOXL_DIR) && docker compose -f voxl-compose.yml down || true"
	
	@echo ""
	@echo "==> Syncing to $(VOXL_USER)@$(VOXL_HOST):$(VOXL_DIR)..."
	@cp "$(DOCKER_DIR)/voxl-compose.yml" "$(DEPLOY_DIR)/voxl-compose.yml"
	@cp "$(DOCKER_DIR)/entrypoint.sh" "$(DEPLOY_DIR)/entrypoint.sh"
	@cp "$(PROJECT_DIR)/.env" "$(DEPLOY_DIR)/.env"

	@rsync -avz --progress --delete \
		--exclude 'data_log/' \
		"$(DEPLOY_DIR)/" \
		"$(VOXL_USER)@$(VOXL_HOST):$(VOXL_DIR)/"

	@ssh "$(VOXL_USER)@$(VOXL_HOST)" "mkdir -p $(VOXL_DIR)/data_log"

	@echo ""
	@echo "==> Deploy complete. Directory layout:"
	@echo "    $(VOXL_DIR)/"
	@echo "    ├── voxl-compose.yml"
	@echo "    ├── data_log/             (persistent, not overwritten by deploy)"
	@echo "    └── smip_uav_ws"
	@echo "        └── install/          (pre-built arm64 binaries)"

.PHONY: clean-deploy
clean-deploy:
	@echo "WARNING: This will remove all content from $(DEPLOY_DIR) including logged data!" 
	@echo "Running 'make build-ws PKGS=\"pkg1 pk2\"' instead will overwrite deployed install for that package"
	@read -p "Continue? [y/n] " ans && [ "$${ans}" = "y" ] || [ "$${ans}" = "Y" ] || (echo "Aborted."; exit 1)
	@echo ""
	@echo "==> Cleaning directory $(DEPLOY_DIR)..."
	@cd "$(DEPLOY_DIR)" && rm -fr *
	@echo ""
	@echo "Done."

.PHONY: runtime-export
runtime-export:
	@echo "==> Exporting runtime image to $(IMAGE_TARBALL)"
	docker save "$(IMAGE_NAME):runtime-arm64" | gzip > "$(IMAGE_TARBALL)"
	@echo "==> Saved: ${IMAGE_TARBALL} ($$(du -h "$(IMAGE_TARBALL)" | cut -f1))"

.PHONY: runtime-deploy
runtime-deploy:
	@echo "==> Deploying runtime image on drone..."

	@if [ ! -f "$(IMAGE_TARBALL)" ]; then \
		echo "Runtime image not exported yet. Build and export before deploying..."; \
		exit 1; \
	fi

	@echo "==> Transferring runtime image to drone..."
	rsync -avz --progress "$(IMAGE_TARBALL)" "$(VOXL_USER)@$(VOXL_HOST):/tmp/"
	
	@echo "==> Loading image on drone... (This may take a while)"
	@ssh -t "$(VOXL_USER)@$(VOXL_HOST)" \
		"docker load < /tmp/smip-runtime-arm64.tar.gz && rm /tmp/smip-runtime-arm64.tar.gz"

	@echo "==> Done. Image loaded on drone"


# =================== DRONE OPERATIONS =======================
.PHONY: voxl-start
voxl-start:
	@echo "==> Starting voxl container on $(VOXL_HOST)..."
	@ssh -t "$(VOXL_USER)@$(VOXL_HOST)" \
		"cd $(VOXL_DIR) && docker compose -f voxl-compose.yml up -d"

.PHONY: voxl-shell
voxl-shell:
	@echo "==> Connecting to voxl container..."
	@running=$$(ssh "$(VOXL_USER)@$(VOXL_HOST)" "docker ps -q -f name=smip-voxl-runtime"); \
	if [ -z "$$running" ]; then \
		echo "==> Container not running. Starting it first..."; \
		$(MAKE) voxl-start; \
	fi
	@ssh -t "$(VOXL_USER)@$(VOXL_HOST)" \
		"docker exec -it smip-voxl-runtime bash"

.PHONY: voxl-logs
voxl-logs:
	@ssh "$(VOXL_USER)@$(VOXL_HOST)" \
		"cd $(VOXL_DIR) && docker compose logs -f --tail=100"

.PHONY: voxl-stop
voxl-stop:
	@echo "==> Stopping smip-voxl-runtime container"
	@ssh -t "$(VOXL_USER)@$(VOXL_HOST)" \
		"cd $(VOXL_DIR) && docker compose -f voxl-compose.yml down"

.PHONY: voxl-access-setup
voxl-access-setup:
	@echo "==> Generating ED25519 SSH key (if not already present)..."
	@test -f ~/.ssh/id_ed25519 && echo "    Key already exists, skipping keygen." || ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -N ""
	@echo "==> Copying public key to $(VOXL_USER)@$(VOXL_HOST)..."
	ssh-copy-id -i ~/.ssh/id_ed25519.pub "$(VOXL_USER)@$(VOXL_HOST)"
	@echo "==> Done. SSH access to drone is now passwordless."

.PHONY: voxl-pull-data
voxl-pull-data:
	@echo "==> Pulling data_log from $(VOXL_USER)@$(VOXL_HOST):$(VOXL_DIR)/data_log/ ..."
	@mkdir -p "$(DATA_PULL)"
	@rsync -avz --progress \
		"$(VOXL_USER)@$(VOXL_HOST):$(VOXL_DIR)/data_log/" \
		"$(DATA_PULL)/"
	@echo ""
	@echo "==> Data pulled to $(DATA_PULL)/"

.PHONY: voxl-clean
voxl-clean:
	@echo "==> Copying clean script to drone..."
	@scp "$(DOCKER_DIR)/drone-clean.sh" "$(VOXL_USER)@$(VOXL_HOST):/tmp/drone-clean.sh"
	@echo "==> Running clean script on drone..."
	@ssh -t "$(VOXL_USER)@$(VOXL_HOST)" "bash /tmp/drone-clean.sh"