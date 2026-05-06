#!/bin/bash
set -euo pipefail

VOXL_DIR="/voxl_docker"
COMPOSE_FILE="voxl-compose.yml"

echo "==> Stopping runtime container (if running)..."
if [ -f "$VOXL_DIR/$COMPOSE_FILE" ]; then
	cd "$VOXL_DIR" && docker compose -f "$COMPOSE_FILE" down || true
else
	echo "    $VOXL_DIR/$COMPOSE_FILE not found, stopping by container name..."
	docker rm -f smip-voxl-runtime 2>/dev/null || true
fi

echo ""
echo "==> Removing stopped containers..."
docker container prune -f

echo ""
echo "==> Removing all unused images..."
docker image prune -a -f

echo ""
echo "==> Removing unused volumes..."
docker volume prune -f

echo ""
echo "==> Removing build cache..."
docker builder prune -f

echo ""
echo "==> Current images on drone:"
docker images

echo ""
echo "==> Disk usage:"
docker system df

echo ""
echo "==> Done."
