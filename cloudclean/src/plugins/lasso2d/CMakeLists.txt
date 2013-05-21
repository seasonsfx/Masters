include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
QT5_ADD_RESOURCES(lasso2d_RESOURCES_RCC lasso2d_resources.qrc)
add_library(lasso2d SHARED ${plugin_SOURCE} ${lasso2d_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(lasso2d
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
)

if(WIN32)
    target_link_libraries(lasso2d ${GLEW_LIBRARIES})
endif()

QT5_USE_MODULES(lasso2d Core Gui OpenGL)

INSTALL(TARGETS lasso2d DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
