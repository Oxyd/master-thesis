TEMPLATE = subdirs

SUBDIRS += \
    gui \
    cli \
    libsolver

gui.depends += libsolver
cli.depends += libsolver
