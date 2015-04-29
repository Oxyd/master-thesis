TEMPLATE = subdirs

SUBDIRS += \
    gui \
    libsolver

gui.depends += libsolver
