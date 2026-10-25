if(JP_WITH_TENSORRT)
  add_executable(junction_perception junction_perception/main.cpp)
  target_link_libraries(junction_perception PRIVATE jp_infer)

  add_executable(jp_bench jp_bench/main.cpp)
  target_link_libraries(jp_bench PRIVATE jp_infer)

  install(TARGETS junction_perception jp_bench RUNTIME DESTINATION bin)
endif()
