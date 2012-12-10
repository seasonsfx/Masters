project (cloudclean)
cmake_minimum_required (VERSION 2.8)

#set(PROJECT_BINARY_DIR ${PROJECT_SOURCE_DIR}/build)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/plugins)
set(CMAKE_BUILD_TYPE Release)

find_package(Qt4 REQUIRED)
set(QT_USE_OPENGL TRUE)
include(${QT_USE_FILE})

add_definitions(
	-Wall -std=c++0x
	-fopenmp
#	-Wno-deprecated-declarations
#	-Wno-unknown-pragmas
	-fstack-protector-all
    -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
    -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED # Hack
	#-stdlib=libc++
)

# Mute warnings in ms
if(CMAKE_BUILD_TOOL MATCHES "(msdev|devenv|nmake)")
    add_definitions(/W2)
endif()

#link_directories(/usr/local/lib)

list(APPEND QMAKE_CXXFLAGS_RELEASE -O3)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#find_package(Eigen3 REQUIRED PATH "C:\\Program Files\\PCL 1.6.0\\3rdParty\\Eigen" 3.0)
#include_directories(EIGEN3_INCLUDE_DIR)
#set(PCL_DIR "C:/Program Files/PCL/cmake")

find_package(PCL 1.7 REQUIRED COMPONENTS
	common
	io
	octree
	kdtree
	search
	features
	filters
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#list(APPEND QMAKE)

#target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

find_package(OpenCL REQUIRED)

include_directories(
	external
	common
	${OPENCL_INCLUDE_DIRS}
	#/usr/local/include/pcl-1.7
	#/usr/include/flann
	#/usr/include/eigen3
	#/opt/AMDAPP/include
	#/opt/NVIDIA_GPU_Computing_SDK/OpenCL/common/inc
)

add_subdirectory(cloudclean)
#add_subdirectory(plugins)
