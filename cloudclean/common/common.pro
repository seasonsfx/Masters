include(../general.pri)
TEMPLATE = lib
QT += opengl

HEADERS += \
    interfaces.h \
    utilities.h \
    timer.hpp
    #../cloudclean/cloudmodel.h \
    #../cloudclean/glarea.h

SOURCES+= \
    #../cloudclean/cloudmodel.cpp \
    #../cloudclean/glarea.cpp

