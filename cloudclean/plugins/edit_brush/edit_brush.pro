include(../../shared.pri)

TARGET = edit_brush
HEADERS += edit_brush.h \
    cpu_brush.h


SOURCES += edit_brush.cpp

INCLUDEPATH += \
        ../../cloudclean \
        ../../common \
        ../../external \
        "/usr/local/include/pcl-1.7/" \
        "/usr/include/flann/" \
        "/usr/include/eigen3/" \
        "/opt/AMDAPP/include" \
        "/opt/NVIDIA_GPU_Computing_SDK/OpenCL/common/inc"

LIBS += -lpcl_io \
        -lpcl_common  \
        -lpcl_features  \
        -lpcl_kdtree  \
        -lpcl_octree  \
        -lpcl_visualization \
        -lpcl_search \
        -lpcl_filters \
        -lGL \
        -lGLU \
        -lOpenCL

RESOURCES += \
    edit_brush.qrc
