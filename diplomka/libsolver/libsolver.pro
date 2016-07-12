QT       -= core gui

TARGET = libsolver
TEMPLATE = lib
CONFIG += staticlib c++14

QMAKE_CXXFLAGS_GNUCXX14 = -std=c++14

QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE += -O3
QMAKE_CXXFLAGS_RELEASE += -DNDEBUG
QMAKE_CXXFLAGS_RELEASE += -Wno-maybe-uninitialized

INCLUDEPATH += /usr/include/eigen3

SOURCES += \
    world.cpp \
    action.cpp \
    predictor.cpp \
    solvers.cpp \
    log_sinks.cpp \
    greedy.cpp \
    separate_paths_solver.cpp \
    lra.cpp \
    whca.cpp \
    operator_decomposition.cpp

HEADERS += \
    world.hpp \
    action.hpp \
    predictor.hpp \
    solvers.hpp \
    a_star.hpp \
    greedy.hpp \
    separate_paths_solver.hpp \
    lra.hpp \
    whca.hpp \
    operator_decomposition.hpp

unix {
    target.path = /usr/lib
    INSTALLS += target
}
