include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
    ${OPENCL_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(visualisedepth_RESOURCES_RCC visualisedepth.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(visualisedepth SHARED ${plugin_SOURCE} ${visualisedepth_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(visualisedepth
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
    ${FLANN_LIBRARY}
    ${Boost_LIBRARIES}
    normalestimator
)

QT5_USE_MODULES(visualisedepth Core Gui OpenGL)

INSTALL(TARGETS visualisedepth DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
