include_directories(
	../common
	../cloudclean
)


include_directories(
        ${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS} # for legacy code
        ${PCL_INCLUDE_DIRS}
        ${FLANN_INCLUDE_DIRS}
)
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

if(WIN32)
    set(WIN_RESOURCE_FILE cloudclean.rc)
    add_executable(run_cloudclean WIN32 main.cpp ${WIN_RESOURCE_FILE})
else()
    add_executable(run_cloudclean main.cpp)
endif()

target_link_libraries(run_cloudclean
    cloudclean
)

qt5_use_modules(run_cloudclean Widgets Gui OpenGL)

INSTALL(TARGETS run_cloudclean DESTINATION ${BIN_INSTALL_DIR_SUFFIX} COMPONENT CloudClean)
