add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

SET(LIBRARY_OUTPUT_PATH ${PLUGIN_OUTPUT_PATH})

add_subdirectory(brush3d)
add_subdirectory(brush2d)
add_subdirectory(normalestimation)
add_subdirectory(visualisenormals)
