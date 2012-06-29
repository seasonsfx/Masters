#-------------------------------------------------
#
# Project created by QtCreator 2012-03-17T17:46:31
#
#-------------------------------------------------

include(general.pri)

QT       += core gui opengl

TARGET = CloudClean
TEMPLATE = app

SOURCES += \
    main.cpp \
    MousePoles.cpp \
    layerlist.cpp \
    layer.cpp \
    mainwindow.cpp \
    cloudmodel.cpp \
    glarea.cpp \
    layerview.cpp \
    pluginmanager.cpp

HEADERS  += \
    io.h \
    MousePoles.h \
    helpers.h \
    layerlist.h \
    layer.h \
    mainwindow.h \
    cloudmodel.h \
    glarea.h \
    layerview.h \
    pluginmanager.h \
    interfaces.h


#QT_DEBUG_PLUGINS=1
#DESTDIR = ../distrib

INCLUDEPATH += \
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

OTHER_FILES += \
    shaders/points.vert \
    shaders/points.frag \
    shaders/lasso.vert \
    shaders/lasso.frag \
    cl_kernels/lasso.cl \
    CMakeLists.txt \
    plugins/lasso.cl

FORMS += \
    layerview.ui
