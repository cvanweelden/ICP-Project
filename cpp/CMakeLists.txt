cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(registration)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (registration registration.cpp)
target_link_libraries (registration ${PCL_LIBRARIES})