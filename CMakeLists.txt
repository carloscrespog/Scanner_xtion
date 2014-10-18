cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(scanner_xtion)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (scanner_xtion scanner_xtion.cpp)
target_link_libraries (scanner_xtion ${PCL_LIBRARIES} ViconDataStreamSDK_CPP DebugServices)
