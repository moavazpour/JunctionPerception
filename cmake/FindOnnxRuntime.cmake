# Finds a prebuilt ONNX Runtime release (C/C++ API) under ORT_ROOT (CMake or environment variable).
# Provides the imported target OnnxRuntime::OnnxRuntime.

find_path(OnnxRuntime_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS ${ORT_ROOT} $ENV{ORT_ROOT}
    PATH_SUFFIXES include)
find_library(OnnxRuntime_LIBRARY onnxruntime
    HINTS ${ORT_ROOT} $ENV{ORT_ROOT}
    PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OnnxRuntime
    REQUIRED_VARS OnnxRuntime_LIBRARY OnnxRuntime_INCLUDE_DIR)

if(OnnxRuntime_FOUND AND NOT TARGET OnnxRuntime::OnnxRuntime)
    add_library(OnnxRuntime::OnnxRuntime SHARED IMPORTED)
    set_target_properties(OnnxRuntime::OnnxRuntime PROPERTIES
        IMPORTED_LOCATION "${OnnxRuntime_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${OnnxRuntime_INCLUDE_DIR}")
endif()
