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
	${PCL_INCLUDE_DIRS}
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

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )

add_executable(CloudClean
	${cloudclean_SOURCES} 
	${cloudclean_HEADERS_MOC} 
	${cloudclean_FORMS_HEADERS} 
	${cloudclean_RESOURCES_RCC}
)

if(WIN32)
    find_package(GLEW REQUIRED)
    include_directories( ${GLEW_INCLUDE_DIR} )
    target_link_libraries(CloudClean ${GLEW_LIBRARIES})
    message("---------------------------")
    message("${GLEW_LIBRARIES}")

endif()

target_link_libraries(CloudClean
    ${QT_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
    ${PCL_LIBRARIES}
    ${OPENCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
)
