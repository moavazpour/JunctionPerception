add_library(jp_core version.cpp)
add_library(jp::core ALIAS jp_core)

target_include_directories(jp_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_definitions(jp_core PRIVATE JP_VERSION="${PROJECT_VERSION}")
target_link_libraries(jp_core PRIVATE jp_warnings)
