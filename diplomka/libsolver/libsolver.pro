QT       -= core gui

TARGET = libsolver
TEMPLATE = lib
CONFIG += staticlib

QMAKE_CXXFLAGS += -std=c++14

QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE += -O3
QMAKE_CXXFLAGS_RELEASE += -DNDEBUG

SOURCES += \
    world.cpp \
    action.cpp \
    predictor.cpp \
    solvers.cpp \
    log_sinks.cpp

HEADERS += \
    world.hpp \
    action.hpp \
    predictor.hpp \
    solvers.hpp \
    a_star.hpp
unix {
    target.path = /usr/lib
    INSTALLS += target
}
