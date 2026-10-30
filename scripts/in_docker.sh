#!/usr/bin/env bash
# Runs a command inside the development image with the repository mounted at /workspace.
# When JP_DATA_ROOT is set, that folder is mounted read-only at /data and passed on as JP_DATA_ROOT.
# NVIDIA GPUs are passed through when nvidia-smi is available (disable with JP_DOCKER_GPU=0).
# Usage: scripts/in_docker.sh <command> [args...]
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${JP_DEV_IMAGE:-jp-dev:latest}"
cache_dir="${repo_root}/.cache"

mkdir -p "${cache_dir}/vcpkg-binary" "${cache_dir}/vcpkg-downloads" "${cache_dir}/npm"

extra_args=()
if [[ -n "${JP_DATA_ROOT:-}" ]]; then
    extra_args+=(-v "${JP_DATA_ROOT}:/data:ro" -e JP_DATA_ROOT=/data)
fi
if [[ "${JP_DOCKER_GPU:-1}" != "0" ]] && command -v nvidia-smi >/dev/null 2>&1; then
    extra_args+=(--gpus all)
fi

docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "${repo_root}:/workspace" \
    -v "${cache_dir}:/cache" \
    "${extra_args[@]}" \
    -w /workspace \
    "${image}" "$@"
