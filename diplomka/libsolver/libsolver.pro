#-------------------------------------------------
#
# Project created by QtCreator 2015-04-29T20:00:23
#
#-------------------------------------------------

QT       -= core gui

TARGET = libsolver
TEMPLATE = lib
CONFIG += staticlib

QMAKE_CXXFLAGS += -std=c++14

SOURCES += \
    world.cpp \
    action.cpp \
    solvers.cpp

HEADERS += \
    world.hpp \
    action.hpp \
    solvers.hpp
unix {
    target.path = /usr/lib
    INSTALLS += target
}
