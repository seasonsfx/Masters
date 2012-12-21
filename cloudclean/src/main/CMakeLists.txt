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

add_executable(cloudclean_bin main.cpp)

target_link_libraries(cloudclean_bin
    cloudclean
    ${QT_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
)

INSTALL(TARGETS cloudclean_bin DESTINATION bin)
