if(JP_WITH_TENSORRT OR JP_WITH_ONNXRUNTIME)
  add_executable(junction_perception junction_perception/main.cpp)
  target_link_libraries(junction_perception PRIVATE jp_infer)

  add_executable(jp_bench jp_bench/main.cpp)
  target_link_libraries(jp_bench PRIVATE jp_infer)

  add_executable(jp_calibrate jp_calibrate/main.cpp)
  target_link_libraries(jp_calibrate PRIVATE jp_core)

  install(TARGETS junction_perception jp_bench jp_calibrate RUNTIME DESTINATION bin)
endif()
