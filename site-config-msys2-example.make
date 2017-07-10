LDFLAGS += -L/msys64/mingw64/lib
eigen_include_dir = /msys64/mingw64/include/eigen3
boost_system_lib = boost_system-mt
boost_filesystem_lib = boost_filesystem-mt
boost_program_options_lib = boost_program_options-mt
qt_include_dir = /msys64/mingw64/include
moc = moc
uic = uic

gui_ldflags = -Wl,-subsystem,windows

post_build = ldd $1 | sed -n '/\/mingw64\//s/.*=> \([^ ]*\).*/\1/p' | \
             while read f; do \
               cp "$$f" $(dir $1); \
             done
