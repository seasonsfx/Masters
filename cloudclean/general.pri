QMAKE_CXX = clang++
D_QMAKE_CC = clang++

#QMAKE_CXX = g++-4.5
#D_QMAKE_CC = g++-4.5

QMAKE_CXXFLAGS += -std=c++0x -Wall #-g #-stdlib=libc++
QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE -= -Os

#QT_DEBUG_PLUGINS=1
CONFIG += debug
