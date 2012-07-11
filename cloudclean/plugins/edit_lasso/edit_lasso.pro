include(../../shared.pri)

TARGET = edit_lasso_cpu
HEADERS += edit_lasso.h \
    cpu_lasso.h \
    settings.h


SOURCES += edit_lasso.cpp \
    settings.cpp

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
    edit_lasso.qrc

OTHER_FILES += \
    kernels/lasso.cl

FORMS += \
    settings.ui
