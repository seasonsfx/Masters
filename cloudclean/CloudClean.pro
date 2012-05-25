#-------------------------------------------------
#
# Project created by QtCreator 2012-03-17T17:46:31
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = CloudClean
TEMPLATE = app

QMAKE_CXXFLAGS += -g
QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE -= -Os

SOURCES += main.cpp cloudclean.cpp viewpane.cpp \
    appdata.cpp \
    glwidget.cpp \
    MousePoles.cpp

HEADERS  += cloudclean.h viewpane.h \
    appdata.h \
    io.h \
    glwidget.h \
    MousePoles.h

FORMS    += cloudclean.ui

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += pcl_io-1.5 pcl_common-1.5 pcl_features-1.5 pcl_kdtree-1.5 pcl_visualization-1.5
}
