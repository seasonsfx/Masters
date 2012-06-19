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

QMAKE_CXXFLAGS += -g -std=c++0x #-stdlib=libc++
QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE -= -Os

SOURCES +=  main.cpp cloudclean.cpp viewpane.cpp \
            appdata.cpp \
            glwidget.cpp \
            MousePoles.cpp \
    layerlist.cpp \
    layer.cpp

HEADERS  += cloudclean.h viewpane.h \
            appdata.h \
            io.h \
            glwidget.h \
            MousePoles.h \
            helpers.h \
    layerlist.h \
    layer.h

FORMS    += cloudclean.ui

INCLUDEPATH +=  "/usr/include/pcl-1.5/" \
                "/usr/include/flann/" \
                "/usr/include/eigen3/" \
                "/opt/AMDAPP/include" \
                "/opt/NVIDIA_GPU_Computing_SDK/OpenCL/common/inc" \

LIBS += -lpcl_io \
        -lpcl_common  \
        -lpcl_features  \
        -lpcl_kdtree  \
        -lpcl_visualization \
        -lpcl_search \
        -lGL \
        -lGLU \
        -lOpenCL \

#unix {
#    CONFIG += link_pkgconfig
#    PKGCONFIG += pcl_io-1.5 pcl_common-1.5 pcl_features-1.5 pcl_kdtree-1.5 pcl_visualization-1.5
#}

OTHER_FILES += \
    shaders/points.vert \
    shaders/points.frag \
    shaders/lasso.vert \
    shaders/lasso.frag \
    lasso.cl
