add_definitions(-Dutilities_EXPORTS)

include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB util_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

add_library(cloudclean_util SHARED ${util_SOURCE})
target_link_libraries(
    cloudclean_util
    ${QT_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${PCL_LIBRARIES}
    cloudclean_model
)

#qt5_use_modules(cloudclean_model Core)
