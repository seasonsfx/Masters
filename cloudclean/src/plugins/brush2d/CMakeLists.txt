include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

QT4_ADD_RESOURCES(brush2d_RESOURCES_RCC brush2d_resources.qrc)
add_library(brush2d SHARED ${plugin_SOURCE} ${brush2d_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(brush2d
    ${QT_LIBRARIES}
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_gui
)

if(WIN32)
    target_link_libraries(brush2d ${GLEW_LIBRARIES})
endif()

INSTALL(TARGETS brush2d DESTINATION ${PLUGIN_INSTALL} COMPONENT "CloudClean_plugins")
