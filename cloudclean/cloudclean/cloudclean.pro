#-------------------------------------------------
#
# Project created by QtCreator 2012-03-17T17:46:31
#
#-------------------------------------------------

include(../general.pri)

QT       += core gui opengl

TARGET = CloudClean
TEMPLATE = app

RCC_DIR = shaders

SOURCES += \
    main.cpp \
    layerlist.cpp \
    layer.cpp \
    mainwindow.cpp \
    cloudmodel.cpp \
    glarea.cpp \
    layerview.cpp \
    pluginmanager.cpp \
    toolbox.cpp \
    camera.cpp \
    subsampledialog.cpp \
    pointpicker.cpp

HEADERS  += \
    io.h \
    layerlist.h \
    layer.h \
    mainwindow.h \
    cloudmodel.h \
    glarea.h \
    layerview.h \
    pluginmanager.h \
    toolbox.h \
    camera.h \
    subsampledialog.h \
    pointpicker.h

QMAKE_LFLAGS += -rdynamic

#DESTDIR = ../distrib

INCLUDEPATH += \
        ../common/ \
        "/usr/local/include/pcl-1.7/" \
        #/usr/include/vtk-5.8/ \
        "/usr/include/flann/" \
        "/usr/include/eigen3/" \
        "/opt/AMDAPP/include" \
        "/opt/NVIDIA_GPU_Computing_SDK/OpenCL/common/inc" \
        "/usr/local/lib"

LIBS += -lpcl_io \
        -lpcl_common  \
        -lpcl_features  \
        -lpcl_kdtree  \
        -lpcl_visualization \
        -lpcl_octree  \
        -lpcl_filters \
        -lGL \
        -lGLU \
        -lOpenCL \
        #-lvtkCommon \
        #-lvtkFiltering \
        #-lvtkRendering \
        #-lcommon

OTHER_FILES += \
    shaders/points.vert \
    shaders/points.frag \
    shaders/lasso.vert \
    shaders/lasso.frag \
    cl_kernels/lasso.cl \
    CMakeLists.txt \
    plugins/lasso.cl \

FORMS += \
    layerview.ui \
    toolbox.ui \
    subsampledialog.ui

RESOURCES += \
    cloudclean.qrc
