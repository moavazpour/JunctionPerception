#!/usr/bin/env bash
# Runs a command inside the development image with the repository mounted at /workspace.
# Usage: scripts/in_docker.sh <command> [args...]
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${JP_DEV_IMAGE:-jp-dev:latest}"
cache_dir="${repo_root}/.cache"

mkdir -p "${cache_dir}/vcpkg-binary" "${cache_dir}/vcpkg-downloads" "${cache_dir}/npm"

docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "${repo_root}:/workspace" \
    -v "${cache_dir}:/cache" \
    -w /workspace \
    "${image}" "$@"
