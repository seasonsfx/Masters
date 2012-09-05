include(../general.pri)
TEMPLATE = lib
QT += opengl

HEADERS += \
    interfaces.h \
    utilities.h \

SOURCES+=
    #../cloudclean/cloudmodel.cpp \
    #../cloudclean/glarea.cpp \


INCLUDEPATH += "/usr/include/eigen3/" \
                /usr/local/include/pcl-1.7/ \
                "/opt/AMDAPP/include" \
                "/opt/NVIDIA_GPU_Computing_SDK/OpenCL/common/inc" \
                "../cloudclean" \


#LIBS += -lpcl_common  \
#        -lpcl_search \
#        -lGL \
#        -lGLU \
#        -lCL
