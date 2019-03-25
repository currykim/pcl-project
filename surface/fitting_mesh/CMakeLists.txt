cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ConvexHull)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (convex_hull main.cpp)
target_link_libraries (convex_hull ${PCL_LIBRARIES})
