include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB commands_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

add_library(cloudclean_commands STATIC ${commands_SOURCE})
target_link_libraries(cloudclean_commands ${QT_LIBRARIES})
#qt5_use_modules(cloudclean_commands Core)
