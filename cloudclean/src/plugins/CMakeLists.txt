add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

add_subdirectory(brush3d)
add_subdirectory(brush2d)
add_subdirectory(normalestimation)
add_subdirectory(visualisenormals)
