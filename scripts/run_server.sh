#!/usr/bin/env bash
# (Re)starts the twin server (jp_server) as a background container that restarts with the host.
# The port is published on JP_SERVER_ADDRESS only (the LAN address of the host); recordings are
# read from JP_DATA_ROOT. Extra arguments are passed to jp_server (e.g. --mode oracle --input ...).
# Usage: JP_SERVER_ADDRESS=<lan address> JP_DATA_ROOT=<dataset root> scripts/run_server.sh [args]
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${JP_DEV_IMAGE:-jp-dev:latest}"
port="${JP_SERVER_PORT:-8080}"
: "${JP_SERVER_ADDRESS:?set JP_SERVER_ADDRESS to the LAN address the server is published on}"
: "${JP_DATA_ROOT:?set JP_DATA_ROOT to the dataset root}"

gpu_args=()
if command -v nvidia-smi >/dev/null 2>&1; then
    gpu_args=(--gpus all)
fi

docker rm -f jp-server >/dev/null 2>&1 || true
docker run -d --name jp-server --restart unless-stopped \
    --user "$(id -u):$(id -g)" \
    "${gpu_args[@]}" \
    -p "${JP_SERVER_ADDRESS}:${port}:8080" \
    -v "${repo_root}:/workspace" \
    -v "${repo_root}/.cache:/cache" \
    -v "${JP_DATA_ROOT}:/data:ro" -e JP_DATA_ROOT=/data \
    -w /workspace \
    "${image}" build/release/src/apps/jp_server "$@" >/dev/null
echo "jp-server started on http://${JP_SERVER_ADDRESS}:${port}"
