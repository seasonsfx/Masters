cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(range_image_border_extraction)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(PCL 1.7 REQUIRED)
find_pacakge(BOOST COMPONENTS SYSTEM)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (range_image_border_extraction range_image_border_extraction.cpp)
target_link_libraries (range_image_border_extraction ${PCL_LIBRARIES})