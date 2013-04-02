FIND_PACKAGE (OpenCL REQUIRED)

include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
    ${OPENCL_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(normalestimation SHARED ${plugin_SOURCE} )
target_link_libraries(normalestimation ${QT_LIBRARIES} ${OPENCL_LIBRARIES})
