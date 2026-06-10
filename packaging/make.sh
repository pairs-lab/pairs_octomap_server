#!/usr/bin/env bash
# Host entrypoint: build ros-noetic-octomap-server_*.deb for pairs_octomap_server.
# Dependency .debs are taken from $PAIRS_PREBUILT_DIR (default: the apt repo pool).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$(dirname "$REPO_DIR")"
OUTPUT_DIR="$SCRIPT_DIR/output"
PREBUILT_DIR="${PAIRS_PREBUILT_DIR:-/home/thanhnc19/vin_dron_ws/pairs-apt/debs}"
IMAGE="pairs-octomap-server-noetic-packaging"
mkdir -p "$OUTPUT_DIR" "$PREBUILT_DIR"
echo ">> building packaging image ($IMAGE)..."
docker build -t "$IMAGE" "$SCRIPT_DIR"
echo ">> building .deb (deps from $PREBUILT_DIR -> /prebuilt)..."
docker run --rm \
  -v "$SRC_DIR":/src:ro \
  -v "$OUTPUT_DIR":/output \
  -v "$PREBUILT_DIR":/prebuilt:ro \
  "$IMAGE"
echo ">> done. Artifacts:"; ls -1 "$OUTPUT_DIR"/*.deb
