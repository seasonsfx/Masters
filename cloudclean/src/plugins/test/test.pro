include(../../common.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT       += opengl
TARGET = ../test
HEADERS += \
    test.h
SOURCES += \
    test.cpp

INCLUDEPATH += \
        "../.." \
