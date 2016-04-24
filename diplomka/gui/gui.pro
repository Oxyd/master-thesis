QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = gui
TEMPLATE = app

CONFIG += c++14
QMAKE_CXXFLAGS_GNUCXX14 = -std=c++14

QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE += -O3
QMAKE_CXXFLAGS_RELEASE += -DNDEBUG

LIBS += -lboost_filesystem -lboost_system

SOURCES  += bottom_bar_controller.cpp \
            main.cpp \
            mainwindow.cpp \
            scenario_edit.cpp \
            world_scene.cpp \
            zoomable_graphics_view.cpp

HEADERS  += bottom_bar_controller.hpp \
            mainwindow.hpp \
            scenario_edit.hpp \
            world_scene.hpp \
            zoomable_graphics_view.hpp

FORMS    += mainwindow.ui edit.ui

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
