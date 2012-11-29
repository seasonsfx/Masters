include_directories (
	${QT_QTCORE_INCLUDE_DIR}
	${QT_QTGUI_INCLUDE_DIR}
	${QT_QTCORE_INCLUDE_DIR}
	${QT_QTGUI_INCLUDE_DIR}
	${QT_QTOPENGL_INCLUDE_DIR}
)

include_directories(${PROJECT_SOURCE_DIR}/cloudclean)

ADD_DEFINITIONS(${QT_DEFINITIONS})
ADD_DEFINITIONS(-DQT_PLUGIN)
#ADD_DEFINITIONS(-DQT_NO_DEBUG)
ADD_DEFINITIONS(-DQT_SHARED)

add_subdirectory (edit_flood_fpfh)
add_subdirectory (edit_flood_normals)
add_subdirectory (edit_flood_pca)
add_subdirectory (edit_flood_smoothness)
add_subdirectory (edit_lasso)
add_subdirectory (edit_mincut)
add_subdirectory (edit_mincut2)
add_subdirectory (edit_mincut3)
add_subdirectory (viz_normals)
