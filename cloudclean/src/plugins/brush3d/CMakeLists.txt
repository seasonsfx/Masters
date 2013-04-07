include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
qt4_add_resources(RESOURCES_RCC resources.qrc)
add_library(brush3d SHARED ${plugin_SOURCE} ${RESOURCES_RCC})
target_link_libraries(brush3d
    ${QT_LIBRARIES}
    cloudclean_util
    cloudclean_pluginsystem
)

if(WIN32)
    target_link_libraries(brush3d ${GLEW_LIBRARIES})
endif()
