include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(markov_RESOURCES_RCC markov.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(markov SHARED ${plugin_SOURCE} ${markov_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
ADD_DEPENDENCIES(markov onlineforest)

target_link_libraries(
    markov
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    onlineforest
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(markov Core Gui OpenGL)

INSTALL(TARGETS markov DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
