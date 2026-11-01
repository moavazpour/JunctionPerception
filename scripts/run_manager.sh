#!/usr/bin/env bash
# (Re)starts the site manager (manager/jp_manager.py) as a background container that restarts with
# the host. It serves the console and runs one jp_server per active site inside the container.
# The port is published on JP_SERVER_ADDRESS only (the LAN address of the host); recordings are
# read from JP_DATA_ROOT. Replaces a single-site server started by scripts/run_server.sh.
# Usage: JP_SERVER_ADDRESS=<lan address> JP_DATA_ROOT=<dataset root> scripts/run_manager.sh [args]
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${JP_DEV_IMAGE:-jp-dev:latest}"
port="${JP_SERVER_PORT:-8080}"
name="${JP_MANAGER_CONTAINER:-jp-manager}"
: "${JP_SERVER_ADDRESS:?set JP_SERVER_ADDRESS to the LAN address the console is published on}"
: "${JP_DATA_ROOT:?set JP_DATA_ROOT to the dataset root}"

gpu_args=()
if command -v nvidia-smi >/dev/null 2>&1; then
    gpu_args=(--gpus all)
fi

mkdir -p "${repo_root}/.cache/manager"
docker rm -f jp-server "${name}" >/dev/null 2>&1 || true
docker run -d --name "${name}" --restart unless-stopped \
    --user "$(id -u):$(id -g)" \
    "${gpu_args[@]}" \
    -p "${JP_SERVER_ADDRESS}:${port}:8080" \
    -v "${repo_root}:/workspace" \
    -v "${repo_root}/.cache:/cache" \
    -v "${JP_DATA_ROOT}:/data:ro" -e JP_DATA_ROOT=/data \
    -w /workspace \
    "${image}" python3 -u manager/jp_manager.py "$@" >/dev/null
echo "console started on http://${JP_SERVER_ADDRESS}:${port}"
