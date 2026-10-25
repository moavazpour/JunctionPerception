# x86_64 + NVIDIA dGPU. TensorRT 10.3 / CUDA 12.6 to match JetPack 6.1 on Jetson Orin.
ARG BASE=nvcr.io/nvidia/tensorrt:24.08-py3

FROM ${BASE} AS build
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build pkg-config \
      libopencv-dev libeigen3-dev libyaml-cpp-dev nlohmann-json3-dev libspdlog-dev libgtest-dev \
      libpaho-mqtt-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY CMakeLists.txt ./
COPY cmake cmake
COPY core core
COPY apps apps
COPY tests tests
RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/jp \
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
COPY --from=build /opt/jp /opt/jp
ENV PATH=/opt/jp/bin:$PATH
# Engines are cached next to the ONNX files in /models; state (calibration, learned map) in /data/state.
VOLUME ["/models", "/data/state", "/config"]
ENTRYPOINT ["junction_perception"]
CMD ["--config", "/config/site.yaml"]
