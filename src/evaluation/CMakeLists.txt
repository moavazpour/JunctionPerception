add_library(jp_evaluation evaluation.cpp)
add_library(jp::evaluation ALIAS jp_evaluation)
target_link_libraries(jp_evaluation
    PUBLIC jp::core Eigen3::Eigen
    PRIVATE jp::vru jp_warnings)
