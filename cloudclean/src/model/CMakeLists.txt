include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

message("PCL INCLUDE" ${PCL_INCLUDE_DIRS})

FILE (GLOB model_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

add_library(cloudclean_model STATIC ${model_SOURCE})
target_link_libraries(cloudclean_model ${QT_LIBRARIES})
#qt5_use_modules(cloudclean_model Core)
