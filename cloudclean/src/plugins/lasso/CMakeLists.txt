include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
qt4_add_resources(lasso_RESOURCES_RCC lasso_resources.qrc)
add_library(lasso SHARED ${plugin_SOURCE} ${lasso_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(lasso
    ${QT_LIBRARIES}
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
)

if(WIN32)
    target_link_libraries(lasso ${GLEW_LIBRARIES})
endif()

INSTALL(TARGETS lasso DESTINATION ${PLUGIN_INSTALL} COMPONENT "CloudClean_plugins")
