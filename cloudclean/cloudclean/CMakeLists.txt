project( CloudClean )
cmake_minimum_required( VERSION 2.6 )
set ( CMAKE_BUILD_TYPE Release )
add_definitions ( -Wall )
find_package ( Qt4 REQUIRED )
include ( ${QT_USE_FILE} )
include_directories (
	${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
	${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR} ${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR}
	interfaces
	/usr/include/pcl-1.6/
	/usr/include/flann/
	/usr/include/eigen3/
	/opt/AMDAPP/include
	/opt/NVIDIA_GPU_Computing_SDK/OpenCL/common/inc)
set ( CloudClean_HDRS
	io.h
	MousePoles.h
	helpers.h
	layer.h
	layerview.h
	interfaces/editplugininterface.h
	)

set ( CloudClean_SRCS
	main.cpp
	MousePoles.cpp
	layerlist.cpp
	layer.cpp
	mainwindow.cpp
	cloudmodel.cpp
	glarea.cpp
	layerview.cpp
	lassoselectplugin.cpp
	)

set ( CloudClean_MOCS
	layerlist.h
	mainwindow.h
	cloudmodel.h
	glarea.h
	plugins/lassoselectplugin.h
	)
QT4_WRAP_CPP(MOCS ${CloudClean_MOCS})

add_executable ( CloudClean ${CloudClean_SRCS} ${UIS} ${RSCS} ${TRS} ${MOCS} )
target_link_libraries ( CloudClean  ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY} )
