#QMAKE_CXX = clang++
#D_QMAKE_CC = clang++

#QMAKE_CXX = g++-4.5
#D_QMAKE_CC = g++-4.5

QMAKE_CXXFLAGS += -std=c++0x -fopenmp -Wall -Wno-deprecated-declarations -Wno-unknown-pragmas  -fstack-protector-all #-stdlib=libc++
QMAKE_CXXFLAGS_RELEASE -= -O3
#QMAKE_CXXFLAGS_RELEASE -= -Os

#QT_DEBUG_PLUGINS=1
#CONFIG += debug
