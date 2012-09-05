include(../../shared.pri)

TARGET = viz_normals
HEADERS += \
    viz_normals.h

QMAKE_CXXFLAGS += -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

SOURCES += \
    viz_normals.cpp

INCLUDEPATH += \
        ../../cloudclean \
        ../../common \
        "/usr/local/include/pcl-1.7/" \
        "/usr/include/flann/" \
        "/usr/include/eigen3/" \
        "/opt/AMDAPP/include" \
        "/opt/NVIDIA_GPU_Computing_SDK/OpenCL/common/inc"

LIBS += -lpcl_io \
        -lpcl_common  \
        -lpcl_features  \
        -lpcl_kdtree  \
        -lpcl_visualization \
        -lpcl_search \
        -lpcl_filters \
        -lGL \
        -lGLU \
        -lOpenCL

RESOURCES += \
    viz_normals.qrc

OTHER_FILES += \
    shaders/normals.vert \
    shaders/normals.geom \
    shaders/normals.frag
