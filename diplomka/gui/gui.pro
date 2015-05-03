#-------------------------------------------------
#
# Project created by QtCreator 2015-04-29T19:59:04
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = gui
TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++14

SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.hpp

FORMS    += mainwindow.ui

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../libsolver/release/ -llibsolver
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../libsolver/debug/ -llibsolver
else:unix: LIBS += -L$$OUT_PWD/../libsolver/ -llibsolver

INCLUDEPATH += $$PWD/../libsolver
DEPENDPATH += $$PWD/../libsolver

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../libsolver/release/liblibsolver.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../libsolver/debug/liblibsolver.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../libsolver/release/libsolver.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../libsolver/debug/libsolver.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../libsolver/liblibsolver.a
