#!/usr/bin/env bash
# Build the driver image:  bash docker/build.sh   (tag: dual-arm-allegro:jazzy)
set -euo pipefail
cd "$(dirname "$0")"
docker build -t "${IMAGE:-dual-arm-allegro:jazzy}" .
