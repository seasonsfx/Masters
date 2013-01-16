include_directories(
	../common
	../cloudclean
)


include_directories(
        ${QT_QTCORE_INCLUDE_DIR}
        ${QT_QTGUI_INCLUDE_DIR}
        ${QT_QTCORE_INCLUDE_DIR}
        ${QT_QTGUI_INCLUDE_DIR}
        ${QT_QTOPENGL_INCLUDE_DIR}
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
    ${QT_QTMAIN_LIBRARY}
    ${QT_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
)

INSTALL(TARGETS run_cloudclean DESTINATION ${BIN_INSTALL_DIR_SUFFIX} COMPONENT CloudClean)
