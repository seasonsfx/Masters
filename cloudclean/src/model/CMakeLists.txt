include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

message("PCL INCLUDE" ${PCL_INCLUDE_DIRS})

set(model_HEADERS
    datamodel.h
    layer.h
    pointcloud.h
    selection.cpp
)

set(model_SOURCE
    datamodel.cpp
    layer.cpp
    pointcloud.cpp
    selection.cpp
)

add_library(cloudclean_model STATIC ${model_HEADERS} ${model_SOURCE})
target_link_libraries(cloudclean_model ${QT_LIBRARIES})
#qt5_use_modules(cloudclean_model Core)
