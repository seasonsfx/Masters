FILE (GLOB app_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
FILE (GLOB app_FORMS *.ui)

#QT4_WRAP_UI(app_FORMS_HEADERS ${app_FORMS})
#QT4_ADD_RESOURCES(app_RESOURCES_RCC resources/resources.qrc)

add_library(
    helloworld SHARED
    ${app_SRCS}
    #${app_FORMS_HEADERS}
    #${app_RESOURCES_RCC}
)

target_link_libraries(
    helloworld
    ${QT_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
    ${PCL_LIBRARIES}
    ${FLANN_LIBRARY}
    ${OPENCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

SET (PLUGIN_VERSION 0.1)
SET (PLUGIN_COMPAT_VERSION 0.1)

FILE (GLOB CONFIGINPUT *.pluginspec.in)
STRING (REPLACE ".pluginspec.in" ".pluginspec" CONFIGOUTPUT "${CONFIGINPUT}")
CONFIGURE_FILE ("${CONFIGINPUT}" "LIBRARY_OUTPUT_PATH/${CONFIGOUTPUT}")
