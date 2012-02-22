cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(registration)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library (util util.cpp )

add_library (registration registration.cpp )
target_link_libraries (registration util ${PCL_LIBRARIES})

add_executable (evaluation evaluation.cpp)
target_link_libraries (evaluation util registration ${PCL_LIBRARIES})

add_executable (register register.cpp)
target_link_libraries (register util registration ${PCL_LIBRARIES})

add_executable (openni_grabber openni_grabber.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})