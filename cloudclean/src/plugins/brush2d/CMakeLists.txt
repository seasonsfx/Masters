include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

QT5_ADD_RESOURCES(brush2d_RESOURCES_RCC brush2d_resources.qrc)
add_library(brush2d SHARED ${plugin_SOURCE} ${brush2d_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(brush2d
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_gui
)

if(WIN32)
    target_link_libraries(brush2d ${GLEW_LIBRARIES})
endif()

QT5_USE_MODULES(brush2d Core Gui Widgets OpenGL)

INSTALL(TARGETS brush2d DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
