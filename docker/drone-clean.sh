#!/bin/bash
set -euo pipefail

VOXL_DIR="/voxl_docker"
RUNTIME_IMAGE="voxl-drone:runtime-arm64"

echo "==> Stopping runtime container (if running)..."
if [ -d "$VOXL_DIR" ]; then
	cd "$VOXL_DIR" && docker compose down || true
else
	echo "    $VOXL_DIR not found, skipping."
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
