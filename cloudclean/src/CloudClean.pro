#-------------------------------------------------
#
# Project created by QtCreator 2012-03-17T17:46:31
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = CloudClean
TEMPLATE = app

#QMAKE_CXX = clang++
#D_QMAKE_CC = clang

QMAKE_CXXFLAGS += -g -std=c++0x -Wall #-stdlib=libc++
QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE -= -Os

Release:DESTDIR = release
Release:OBJECTS_DIR = release/.obj
Release:MOC_DIR = release/.moc
Release:RCC_DIR = release/.rcc
Release:UI_DIR = release/.ui

Debug:DESTDIR = debug
Debug:OBJECTS_DIR = debug/.obj
Debug:MOC_DIR = debug/.moc
Debug:RCC_DIR = debug/.rcc
Debug:UI_DIR = debug/.ui

SOURCES += \
    main.cpp \
    MousePoles.cpp \
    layerlist.cpp \
    layer.cpp \
    mainwindow.cpp \
    cloudmodel.cpp \
    glarea.cpp \
    layerview.cpp \
    lassoselectplugin.cpp

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
    interfaces/editplugininterface.h \
    plugins/lassoselectplugin.h

INCLUDEPATH += \
        "interfaces" \
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
