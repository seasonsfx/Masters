SET (CMAKE_AUTOMOC ON)

INCLUDE_DIRECTORIES(
    ${PCL_INCLUDE_DIRS}
    ${QT_QTCORE_INCLUDE_DIR}
    ${QT_QTGUI_INCLUDE_DIR}
    ${QT_QTOPENGL_INCLUDE_DIR}
)

#
# Build sub projects
#
add_subdirectory(utilities)
add_subdirectory(gui)
add_subdirectory(model)
add_subdirectory(plugins)
add_subdirectory(pluginsystem)

#
# Build main binary
#
FILE (GLOB project_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
SET (project_SRCS ${project_SRCS} )
SET (project_LIBS ${QT_LIBRARIES} cloudclean_gui)
SET (project_BIN ${PROJECT_NAME})

ADD_EXECUTABLE(${project_BIN} ${project_SRCS})
TARGET_LINK_LIBRARIES(${project_BIN} ${project_LIBS})
SET_TARGET_PROPERTIES(${project_BIN}
	PROPERTIES VERSION
	"${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}"
	OUTPUT_NAME ${project_BIN}
	CLEAN_DIRECT_OUTPUT 1)

target_link_libraries(${project_BIN}
    ${QT_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
    ${Boost_LIBRARIES}
    cloudclean_model
    cloudclean_util
    pluginsystem
)
#QT5_USE_MODULES(${project_BIN} Widgets Gui OpenGL)

INSTALL(TARGETS ${project_BIN} DESTINATION bin)
