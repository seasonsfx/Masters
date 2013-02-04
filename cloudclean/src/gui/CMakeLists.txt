add_definitions(-DQT_SHARED)
add_definitions(-DCloudClean_EXPORTS)

include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${QT_QTGUI_INCLUDE_DIR}
    ${QT_QTOPENGL_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB app_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
FILE (GLOB app_FORMS *.ui)

QT4_WRAP_UI(app_FORMS_HEADERS ${app_FORMS})
QT4_ADD_RESOURCES(app_RESOURCES_RCC resources/resources.qrc)

add_library(
    cloudclean_gui SHARED
    ${app_FORMS_HEADERS}
    ${app_SRCS}
    ${app_RESOURCES_RCC}
)

if(WIN32)
    find_package(GLEW REQUIRED)
    include_directories( ${GLEW_INCLUDE_DIR} )
    target_link_libraries(cloudclean_gui ${GLEW_LIBRARIES})
endif()

target_link_libraries(
    cloudclean_gui
    ${QT_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
    ${PCL_LIBRARIES}
    ${FLANN_LIBRARY}
    ${OPENCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

#qt5_use_modules(cloudclean_gui Widgets Gui OpenGL)
