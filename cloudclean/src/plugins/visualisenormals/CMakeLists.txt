
include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
    ${OPENCL_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
qt4_add_resources(VRESOURCES_RCC vresources.qrc)
#add_library(visualisenormals SHARED ${plugin_SOURCE} ${VRESOURCES_RCC} ../normalestimation/normalestimation.h)
add_library(visualisenormals SHARED ${plugin_SOURCE} ${VRESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(visualisenormals
    ${QT_LIBRARIES}
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    normalestimator
)

if(WIN32)
    target_link_libraries(visualisenormals ${GLEW_LIBRARIES})
endif()
