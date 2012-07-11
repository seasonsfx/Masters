include(../../shared.pri)

TARGET = edit_sample_cpu
HEADERS += edit_sample.h \
    cpu_sample.h


SOURCES += edit_sample.cpp

INCLUDEPATH += \
        ../../cloudclean \
        ../../common \
        "/usr/include/pcl-1.5/" \
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
    edit_sample.qrc

OTHER_FILES += \
    kernels/sample.cl
