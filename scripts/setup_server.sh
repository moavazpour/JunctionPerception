#!/usr/bin/env bash
# Builds the development image on the host that runs the system (requires Docker). On a host with
# an NVIDIA GPU (nvidia-smi available) the image is based on CUDA + cuDNN and uses the CUDA build
# of ONNX Runtime; otherwise it is CPU-only.
# Usage: scripts/setup_server.sh
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${JP_DEV_IMAGE:-jp-dev:latest}"

build_args=()
if command -v nvidia-smi >/dev/null 2>&1; then
    build_args=(
        --build-arg BASE_IMAGE=nvidia/cuda:13.2.1-cudnn-runtime-ubuntu24.04
        --build-arg ORT_PACKAGE=onnxruntime-linux-x64-gpu_cuda13-1.31.0
    )
fi

docker build --tag "${image}" "${build_args[@]}" \
    --file "${repo_root}/docker/Dockerfile.dev" "${repo_root}/docker"
