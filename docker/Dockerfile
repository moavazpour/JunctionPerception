# x86_64 + NVIDIA dGPU. Default inference: ONNX Runtime on the GPU (CUDA, FP32: the exported model as
# trained). TensorRT 10.3 / CUDA 12.6 stays available (models.backend: tensorrt). The CPU provider is included
# too: the same image runs on hosts without a GPU (models.backend: cpu, or auto-detected).
ARG BASE=nvcr.io/nvidia/tensorrt:24.08-py3
ARG ORT_VERSION=1.20.1

# ONNX Runtime GPU package (CUDA 12 + cuDNN 9, both in the base image); it also contains the CPU provider.
FROM ${BASE} AS deps
ARG ORT_VERSION
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build pkg-config curl ca-certificates \
      libopencv-dev libeigen3-dev libyaml-cpp-dev nlohmann-json3-dev libspdlog-dev libgtest-dev \
      libpaho-mqtt-dev \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /opt/onnxruntime \
 && curl -fsSL https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-gpu-${ORT_VERSION}.tgz \
    | tar -xz -C /opt/onnxruntime --strip-components=1

FROM deps AS build
WORKDIR /src
ARG GIT_SHA=""
COPY CMakeLists.txt VERSION ./
COPY cmake cmake
COPY core core
COPY apps apps
COPY tests tests
RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/jp -DJP_WITH_ONNXRUNTIME=ON -DJP_ORT_CUDA=ON \
      -DJP_GIT_SHA="${GIT_SHA}" \
      -DCMAKE_INSTALL_RPATH=/opt/onnxruntime/lib \
 && cmake --build build -j"$(nproc)" \
 && ctest --test-dir build --output-on-failure \
 && cmake --install build

FROM ${BASE} AS runtime
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      libopencv-core4.5d libopencv-imgproc4.5d libopencv-imgcodecs4.5d libopencv-videoio4.5d libopencv-video4.5d \
      libopencv-features2d4.5d libopencv-calib3d4.5d libyaml-cpp0.7 libspdlog1 libpaho-mqtt1.3 \
      gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /opt/onnxruntime/lib /opt/onnxruntime/lib
COPY --from=build /opt/jp /opt/jp
ENV PATH=/opt/jp/bin:$PATH LD_LIBRARY_PATH=/opt/onnxruntime/lib:$LD_LIBRARY_PATH
# Engines are cached next to the ONNX files in /models; state (calibration, learned map) in /data/state.
# 8081: live camera view (MJPEG) for the web panel.
EXPOSE 8081
VOLUME ["/models", "/data/state", "/config"]
ENTRYPOINT ["junction_perception"]
CMD ["--config", "/config/site.yaml"]
