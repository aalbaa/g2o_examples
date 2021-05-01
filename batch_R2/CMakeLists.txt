cmake_minimum_required(VERSION 3.10)

project( tutorial_slam_R2)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(g2o REQUIRED)

# # add_definitions( -D_USE_MATH_DEFINES)

# # Specify C++ version
set (CMAKE_CXX_STANDARD 14)

include_directories(${G2O_DIR})
# Add library
add_library(tutorial_slam_R2_library ${G2O_LIB_TYPE}  
  vertex_R2.h vertex_R2.cpp edge_R2.h edge_R2.cpp
  types_tutorial_slam_R2.h types_tutorial_slam_R2.cpp
  g2o_tutorial_slam_R2_api.h
)

# # set_target_properties(tutorial_slam2d_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}tutorial_slam2d)

# target_link_libraries(tutorial_slam2d_library g2o::core g2o::solver_eigen Eigen3::solver_eigen)
target_link_libraries(tutorial_slam_R2_library     
  # g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
  g2o::core g2o::types_slam3d g2o::stuff g2o::types_sba
  Eigen3::Eigen
)

add_executable(tutorial_slam_R2
  tutorial_slam_R2.cpp
)

# set_target_properties(tutorial_slam2d PROPERTIES OUTPUT_NAME tutorial_slam2d${EXE_POSTFIX})

target_link_libraries(tutorial_slam_R2 tutorial_slam_R2_library)
