add_definitions(-DQT_SHARED)
add_definitions(-DCloudClean_EXPORTS)


if(CMAKE_BUILD_TOOL MATCHES "(msdev|devenv|nmake)")

else()
    set(LINK_FLAGS -rdynamic)
    add_definitions(
            -rdynamic
    )
endif()

include_directories(
	${QT_QTCORE_INCLUDE_DIR}
	${QT_QTGUI_INCLUDE_DIR}
	${QT_QTCORE_INCLUDE_DIR}
	${QT_QTGUI_INCLUDE_DIR}
	${QT_QTOPENGL_INCLUDE_DIR}
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

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

add_library(cloudclean SHARED
	${cloudclean_SOURCES} 
	${cloudclean_HEADERS_MOC} 
	${cloudclean_FORMS_HEADERS} 
	${cloudclean_RESOURCES_RCC}
)

if(WIN32)
    find_package(GLEW REQUIRED)
    include_directories( ${GLEW_INCLUDE_DIR} )
    target_link_libraries(cloudclean ${GLEW_LIBRARIES})
endif()

target_link_libraries(cloudclean
    ${QT_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
    ${PCL_LIBRARIES}
    ${FLANN_LIBRARY}
    ${OPENCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

#add_executable(run_cloudclean main.cpp)
#target_link_libraries(run_cloudclean
#    cloudclean
#    ${QT_LIBRARIES}
#    ${QT_QTCORE_LIBRARY}
#    ${QT_QTGUI_LIBRARY}
#    ${QT_QTCORE_LIBRARY}
#    ${QT_QTGUI_LIBRARY}
#    ${QT_QTOPENGL_LIBRARY}
#    ${PCL_LIBRARIES}
#    ${FLANN_LIBRARY}
#    ${OPENCL_LIBRARIES}
#    ${OPENGL_LIBRARIES}
#)
