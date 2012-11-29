project (cloudclean)
cmake_minimum_required (VERSION 2.8)

set(PROJECT_BINARY_DIR ${PROJECT_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/plugins)

set(CMAKE_BUILD_TYPE Release)
find_package(Qt4 REQUIRED)
set(QT_USE_OPENGL TRUE)
include(${QT_USE_FILE})

add_definitions(
	-Wall -std=c++0x
	-fopenmp
	-Wno-deprecated-declarations
	-Wno-unknown-pragmas
	-fstack-protector-all
        -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
	#-stdlib=libc++
)

link_directories(/usr/local/lib)

list(APPEND QMAKE_CXXFLAGS_RELEASE -O3)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED 3.0)
include_directories(EIGEN3_INCLUDE_DIR)

include_directories(
	external
	common
	/usr/local/include/pcl-1.7
	/usr/include/flann
	/usr/include/eigen3
	/opt/AMDAPP/include
	/opt/NVIDIA_GPU_Computing_SDK/OpenCL/common/inc
)

add_subdirectory(cloudclean)
add_subdirectory(plugins)
