project (cloudclean)
cmake_minimum_required (VERSION 2.8.6)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_BUILD_TYPE Release)

find_package(Qt4 REQUIRED)
set(QT_USE_OPENGL TRUE)
include(${QT_USE_FILE})

add_definitions(
        -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
        -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED # Hack
)

if(CMAKE_BUILD_TOOL MATCHES "(msdev|devenv|nmake)")
    add_definitions(
        /W2 # Mute warnings in ms
        -DNOMINMAX # Workaround for windows.h bug
    )
else()
    add_definitions(
        -Wall -std=c++0x
        -fopenmp
        -Wno-deprecated-declarations
        -Wno-unknown-pragmas
        -fstack-protector-all
        #-stdlib=libc++
    )
endif()

list(APPEND QMAKE_CXXFLAGS_RELEASE -O3)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#find_package(Eigen3 REQUIRED PATH "C:\\Program Files\\PCL 1.6.0\\3rdParty\\Eigen" 3.0)
#include_directories(EIGEN3_INCLUDE_DIR)

find_package(PCL 1.7 REQUIRED COMPONENTS
	common
	io
	octree
	kdtree
	search
	features
	filters
    segmentation
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenGL REQUIRED)
find_package(OpenCL REQUIRED)

include_directories(
	ext
	src/common
    src/cloudclean
	${OPENCL_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIR}
)

add_subdirectory(src/cloudclean)
add_subdirectory(src/main)
add_subdirectory(src/plugins)
