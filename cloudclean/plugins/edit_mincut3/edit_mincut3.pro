include(../../shared.pri)

TARGET = edit_mincut3
HEADERS += \
    edit_mincut.h \
    settings.h \
    mincut.h \

QMAKE_CXXFLAGS += -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

SOURCES += \
    edit_mincut.cpp \
    settings.cpp \
    mincut.cpp

INCLUDEPATH += \
        ../../cloudclean \
        ../../common \
        ../../external \
        "/usr/local/include/pcl-1.7/" \
        "/usr/include/flann/" \
        "/usr/include/eigen3/" \
        "/opt/AMDAPP/include" \
        "/opt/NVIDIA_GPU_Computing_SDK/OpenCL/common/inc" \
        /usr/include/vtk-5.8/

LIBS += -lpcl_io \
        -lpcl_common  \
        -lpcl_features  \
        -lpcl_visualization \
        -lpcl_segmentation \
        -lpcl_search \
        -lpcl_filters \
        -lGL \
        -lGLU \
        -lOpenCL \
        -lboost_system

RESOURCES += \
    edit_mincut.qrc

FORMS += \
    settings.ui
