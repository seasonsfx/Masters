include(../../shared.pri)

TARGET = edit_brush_curvature
HEADERS += \
    edit_brush_curvature.h

QMAKE_CXXFLAGS += -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

SOURCES += \
    edit_brush_curvature.cpp

INCLUDEPATH += \
        ../../cloudclean \
        ../../common \
        "/usr/include/pcl-1.6/" \
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
    edit_brush.qrc
