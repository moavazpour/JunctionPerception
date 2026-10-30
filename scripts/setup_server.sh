#!/usr/bin/env bash
# Builds the development image on the host that runs the system (requires Docker).
# Usage: scripts/setup_server.sh
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${JP_DEV_IMAGE:-jp-dev:latest}"

docker build --tag "${image}" --file "${repo_root}/docker/Dockerfile.dev" "${repo_root}/docker"
