#-------------------------------------------------
#
# Project created by QtCreator 2015-04-26T21:36:46
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = diplomka
TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++14

SOURCES += main.cpp\
        mainwindow.cpp \
    map.cpp

HEADERS  += mainwindow.hpp \
    map.hpp

FORMS    += mainwindow.ui
