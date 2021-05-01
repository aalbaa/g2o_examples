cmake_minimum_required( VERSION 3.10)
project(example_SE2)

# Append path with Config.cmake dirs/files

# Find the RandomVariable directory
find_package(RandomVariable)
find_package(InEKF)
find_package(Eigen3 REQUIRED)
find_package(manif REQUIRED)

# Main executable
add_executable(main main.cpp)

target_link_libraries(main
  PUBLIC    
    InEKF::SE2
  )

# Set required C++ standard flag
set_property(TARGET main PROPERTY CXX_STANDARD 17)
