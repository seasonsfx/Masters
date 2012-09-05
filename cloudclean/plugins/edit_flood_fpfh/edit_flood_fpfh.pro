include(../../shared.pri)

TARGET = edit_flood_fpfh
HEADERS += \
    edit_flood_fpfh.h \
    settings.h \


SOURCES += \
    edit_flood_fpfh.cpp \
    settings.cpp \

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
    edit_brush.qrc

FORMS += \
    settings.ui
