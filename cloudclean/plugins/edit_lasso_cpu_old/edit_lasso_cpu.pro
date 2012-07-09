include(../../shared.pri)

TARGET = edit_lasso_cpu
HEADERS += edit_lasso_cpu.h \
    #../../cloudclean/cloudmodel.h \
    #../../cloudclean/glarea.h


SOURCES += edit_lasso_cpu.cpp \
    #../../cloudclean/cloudmodel.cpp \
    #../../cloudclean/glarea.cpp


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
    edit_lasso_cpu.qrc

OTHER_FILES += \
    kernels/lasso.cl