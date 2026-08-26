cmake_minimum_required(VERSION 3.10)
project(lidar_mos)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PCL 1.8 REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(mos
    src/main.cpp
    src/io.cpp
    src/preprocess.cpp
    src/ground.cpp
    src/cluster.cpp
    src/viz.cpp
)
target_link_libraries(mos ${PCL_LIBRARIES})
