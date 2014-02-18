include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(testfilter_RESOURCES_RCC testfilter.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(testfilter SHARED ${plugin_SOURCE} ${testfilter_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    testfilter
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(testfilter Core Gui OpenGL)

INSTALL(TARGETS testfilter DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
