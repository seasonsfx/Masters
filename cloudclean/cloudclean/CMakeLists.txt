set(LINK_FLAGS -rdynamic)

add_definitions(
        -rdynamic
)

include_directories(
	${QT_QTCORE_INCLUDE_DIR}
	${QT_QTGUI_INCLUDE_DIR}
	${QT_QTCORE_INCLUDE_DIR}
	${QT_QTGUI_INCLUDE_DIR}
	${QT_QTOPENGL_INCLUDE_DIR}
)

set(cloudclean_SOURCES
	main.cpp
	layerlist.cpp
	layer.cpp
	mainwindow.cpp
	cloudmodel.cpp
	glarea.cpp
	layerview.cpp
	pluginmanager.cpp
	toolbox.cpp
	camera.cpp
	subsampledialog.cpp
	pointpicker.cpp
	../common/utilities.cpp
)

set(cloudclean_HEADERS
	io.h
	layerlist.h
	layer.h
	mainwindow.h
	cloudmodel.h
	glarea.h
	layerview.h
	pluginmanager.h
	toolbox.h
	camera.h
	subsampledialog.h
	pointpicker.h
)

set(cloudclean_RESOURCES cloudclean.qrc)

set(cloudclean_FORMS
	layerview.ui
	subsampledialog.ui
	toolbox.ui
)

QT4_WRAP_CPP(cloudclean_HEADERS_MOC ${cloudclean_HEADERS})
QT4_WRAP_UI(cloudclean_FORMS_HEADERS ${cloudclean_FORMS})
QT4_ADD_RESOURCES(cloudclean_RESOURCES_RCC ${cloudclean_RESOURCES})

MESSAGE("RCC stuff: " ${cloudclean_RESOURCES_RCC})

add_executable(CloudClean
	${cloudclean_SOURCES} 
	${cloudclean_HEADERS_MOC} 
	${cloudclean_FORMS_HEADERS} 
	${cloudclean_RESOURCES_RCC}
)

target_link_libraries(CloudClean
        ${QT_LIBRARIES}
	${QT_QTCORE_LIBRARY}
	${QT_QTGUI_LIBRARY}
	${QT_QTCORE_LIBRARY}
	${QT_QTGUI_LIBRARY}
	${QT_QTOPENGL_LIBRARY}
	pcl_io 
	pcl_common  
	pcl_features  
	pcl_kdtree  
	pcl_visualization 
	pcl_octree  
	pcl_filters 
	pcl_search
	GL 
	GLU 
	OpenCL 
)
