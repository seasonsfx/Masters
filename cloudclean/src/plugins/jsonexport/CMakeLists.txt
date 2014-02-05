include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(jsonexport_RESOURCES_RCC jsonexport.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(jsonexport SHARED ${plugin_SOURCE} ${jsonexport_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    jsonexport
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(jsonexport Core Gui OpenGL)

INSTALL(TARGETS jsonexport DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
