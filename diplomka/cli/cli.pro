QT -= core gui

TARGET = cli
TEMPLATE = app

CONFIG += c++14
QMAKE_CXXFLAGS_GNUCXX14 = -std=c++14

QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE += -O3
QMAKE_CXXFLAGS_RELEASE += -DNDEBUG

LIBS += -lboost_program_options -lboost_filesystem -lboost_system

SOURCES = main.cpp

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
