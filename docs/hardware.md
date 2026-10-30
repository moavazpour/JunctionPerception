# Reference hardware

Timings and frame rates reported in this repository were measured on the following machine.

| Component | Details |
|---|---|
| GPU | NVIDIA GeForce RTX 3080 Laptop GPU, 16 GB, compute capability 8.6 |
| NVIDIA driver | 595.99 (supports CUDA up to 13.2) |
| CPU | Intel Core i9-11900H, 8 cores / 16 threads |
| Memory | 32 GB |
| Host OS | Ubuntu 22.04.5 LTS, Docker 29.9 with the NVIDIA Container Toolkit |

## Runtime stack

Everything runs inside the development image (`docker/Dockerfile.dev`). On this machine it is built
on `nvidia/cuda:13.2.1-cudnn-runtime-ubuntu24.04` with the CUDA 13 build of ONNX Runtime 1.31.0,
so inference uses the CUDA execution provider. Without a GPU the image uses the CPU build of ONNX
Runtime and the CPU execution provider.
