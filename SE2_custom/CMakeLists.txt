cmake_minimum_required( VERSION 3.10)
project(example_SE2)

# Append path with Config.cmake dirs/files

# Find the RandomVariable directory
find_package(g2o REQUIRED)
find_package(InEKF)
find_package(RandomVariable)
find_package(manif REQUIRED)
find_package(Eigen3)

# Add the SE2 library
add_library(example_se2_lib STATIC
            edge_se2R2.cpp edge_se2R2.h edge_se2se2.cpp edge_se2se2.h vertex_se2.cpp vertex_se2.h)

# Link the InEKF::SE2 in order to use the definitions in the header            
target_link_libraries(example_se2_lib
    PUBLIC
    InEKF::SE2 MANIF::manif
    g2o::core g2o::stuff g2o::solver_eigen    
)        


# Main executable
add_executable(main main.cpp)

target_link_libraries(main
  PUBLIC    
    InEKF::SE2
    example_se2_lib
  )

# Set required C++ standard flag
set_property(TARGET main PROPERTY CXX_STANDARD 17)
