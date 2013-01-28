add_definitions(-DQT_SHARED)
add_definitions(-DCloudClean_EXPORTS)

include_directories(
        ${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS} # for legacy code
	${PCL_INCLUDE_DIRS}
        ${FLANN_INCLUDE_DIRS}
)

set(cloudclean_SOURCES
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
        cpointcloud.cpp
        cselection.cpp
        clayer.cpp
        cdatamodel.cpp
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
        cpointcloud.h
        cselection.h
        clayer.h
        cdatamodel.h
        cloudclean_global.h
)

set(cloudclean_RESOURCES cloudclean.qrc)

set(cloudclean_FORMS
	layerview.ui
	subsampledialog.ui
	toolbox.ui
)

#QT5_WRAP_CPP(cloudclean_HEADERS_MOC ${cloudclean_HEADERS})
QT5_WRAP_UI(cloudclean_FORMS_HEADERS ${cloudclean_FORMS})
QT5_ADD_RESOURCES(cloudclean_RESOURCES_RCC ${cloudclean_RESOURCES})

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

add_library(cloudclean SHARED
	${cloudclean_SOURCES} 
        #${cloudclean_HEADERS_MOC}
        ${cloudclean_HEADERS}
	${cloudclean_FORMS_HEADERS} 
	${cloudclean_RESOURCES_RCC}
)

if(WIN32)
    find_package(GLEW REQUIRED)
    include_directories( ${GLEW_INCLUDE_DIR} )
    target_link_libraries(cloudclean ${GLEW_LIBRARIES})
endif()

target_link_libraries(cloudclean
    ${PCL_LIBRARIES}
    ${FLANN_LIBRARY}
    ${OPENCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

qt5_use_modules(cloudclean Widgets Gui OpenGL)

INSTALL(TARGETS cloudclean DESTINATION ${LIB_INSTALL_DIR_SUFFIX} COMPONENT CloudClean)
