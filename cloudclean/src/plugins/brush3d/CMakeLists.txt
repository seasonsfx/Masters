include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

add_library(brush3d SHARED ${plugin_SOURCE})
target_link_libraries(brush3d ${QT_LIBRARIES})
