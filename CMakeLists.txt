cmake_minimum_required(VERSION 3.10)

# Specify g2o's root directory (where it was cloned)
set(G2O_ROOT /home/aalbaali/Documents/Code_base/External/cxx/g2o)
list( APPEND CMAKE_MODULE_PATH ${G2O_ROOT}/cmake_modules )

# This flag is needed sometimes (without it, the executable doesn't run as expected)
# set(BUILD_SHARED_LIBS ON)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(G2O REQUIRED)

# Specify C++ version (minimum 14 required)
set (CMAKE_CXX_STANDARD 17)

# Include the g2o directory
include_directories(${G2O_ROOT})

# Build library
add_library(example_slam_R2_library ${G2O_LIB_TYPE}
  edge_R2_R2.cpp edge_R2_R2.h slam_R2_example_api.h
  types_slam_R2_example.cpp types_slam_R2_example.h
  vertex_R2.cpp vertex_R2.h
)

# Link g2o libraries (I didn't link all of them)
target_link_libraries(example_slam_R2_library     
  g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
  Eigen3::Eigen
)

add_executable(example_slam_R2
  main.cpp
)

# Link the local library to the executable
target_link_libraries(example_slam_R2 example_slam_R2_library)
