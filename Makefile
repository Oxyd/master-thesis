CXXFLAGS += -Wall -Wextra -std=c++14 -pedantic

-include site-config.make

eigen_include_dir ?= /usr/include/eigen3
qt_include_dir ?= /usr/include/qt5
moc ?= moc-qt5
uic ?= /usr/lib64/qt5/bin/uic

mode ?= debug

CXXFLAGS += -fPIC

ifeq ($(mode),debug)
	CXXFLAGS += -ggdb3 -D_GLIBCXX_DEBUG -D_GLIBCXX_DEBUG_PEDANTIC
endif

project_dir = diplomka
build_dir = build/$(mode)
bin_dir = bin/$(mode)

find_sources = $(wildcard $(1)/*.cpp)
find_headers = $(wildcard $(1)/*.hpp)
find_uis = $(wildcard $(1)/*.ui)
object_names = $(addprefix $(build_dir)/,$(1:.cpp=.o))
depfile_names = $(addprefix $(build_dir)/,$(1:.cpp=.d))
ui_header_names = $(foreach ui,$(1),$(build_dir)/$(dir $(ui))ui_$(basename $(notdir $(ui))).h)

libsolver_dir = $(project_dir)/libsolver
libsolver_sources = $(call find_sources,$(libsolver_dir))
libsolver_objects = $(call object_names,$(libsolver_sources))
libsolver_depfiles = $(call depfile_names,$(libsolver_sources))
libsolver_build_dirs = $(dir $(libsolver_objects))
libsolver_lib = $(build_dir)/libsolver.a
libsolver_libname = solver
libsolver_includes = -isystem$(eigen_include_dir)

cli_dir = $(project_dir)/cli
cli_sources = $(call find_sources,$(cli_dir))
cli_objects = $(call object_names,$(cli_sources))
cli_depfiles = $(call depfile_names,$(cli_sources))
cli_build_dirs = $(dir $(cli_objects))
cli_executable = $(bin_dir)/cli
cli_includes = -I$(libsolver_dir)
cli_ldlibs = -lboost_program_options -lboost_filesystem -lboost_system

gui_dir = $(project_dir)/gui
gui_sources = $(call find_sources,$(gui_dir))
gui_moc_sources = $(call find_headers,$(gui_dir))
gui_ui_sources = $(call find_uis,$(gui_dir))
gui_generated_headers = $(call ui_header_names,$(gui_ui_sources))
gui_objects = $(call object_names,$(gui_sources) $(gui_moc_sources:.hpp=.moc.cpp))
gui_depfiles = $(call depfile_names,$(gui_sources))
gui_build_dirs = $(dir $(gui_objects))
gui_executable = $(bin_dir)/gui
gui_qt_modules = Core Gui Widgets
gui_includes = -I$(libsolver_dir) -I$(qt_include_dir) $(addprefix -I$(qt_include_dir)/Qt,$(gui_qt_modules))
gui_includes += $(sort $(addprefix -I,$(dir $(gui_generated_headers))))
gui_includes += -I$(gui_dir)
gui_libs = $(addprefix -lQt5,$(gui_qt_modules))
gui_libs += -lboost_filesystem -lboost_system

.PHONY: all
all: cli gui

.PHONY: clean
clean:
	rm -f $(libsolver_objects)
	rm -f $(libsolver_lib)
	rm -f $(libsolver_depfiles)
	rm -f $(cli_objects)
	rm -f $(cli_executable)
	rm -f $(cli_depfiles)
	rm -f $(gui_objects)
	rm -f $(gui_generated_headers)
	rm -f $(gui_executable)
	rm -f $(gui_depfiles)

.PHONY: cli
cli: $(cli_executable)

.PHONY: gui
gui: $(gui_executable)

$(libsolver_lib) : $(libsolver_objects)
	rm -f $(libsolver_lib)
	$(AR) cqs $@ $^

link = $(CXX) $(1) $(LDFLAGS) $(LDLIBS) -l$(libsolver_libname) -o $@

$(cli_executable) : LDFLAGS += -L$(build_dir)
$(cli_executable) : LDLIBS += $(cli_ldlibs)
$(cli_executable) : $(cli_objects) $(libsolver_lib) | $(bin_dir)
	$(call link,$(cli_objects))

$(gui_executable) : LDFLAGS += -L$(build_dir)
$(gui_executable) : LDLIBS += $(gui_libs)
$(gui_executable) : incdirs += $(gui_includes)
$(gui_executable) : CXXFLAGS += $(gui_includes)
$(gui_executable) : $(gui_objects) $(libsolver_lib) | $(bin_dir)
	$(call link,$(gui_objects))

$(build_dir)/%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ -MMD -MP

$(build_dir)/%.moc.cpp : %.hpp
	$(moc) $(incdirs) $< -o $@

$(build_dir)/%.ui : %.ui
	cp $< $@

ui_%.h : %.ui
	$(uic) $< -o $@

$(libsolver_objects) : CXXFLAGS += $(libsolver_includes)
$(libsolver_objects) : | $(libsolver_build_dirs)

$(cli_objects) : CXXFLAGS += $(cli_includes)
# Boost.ProgramOptions won't link if we compile our .cpp with debugging stdlib
$(cli_objects) : CXXFLAGS := $(filter-out -D_GLIBCXX_DEBUG -D_GLIBCXX_DEBUG_PEDANTIC,$(CXXFLAGS))
$(cli_objects) : | $(cli_build_dirs)

$(gui_objects) : CXXFLAGS += $(gui_includes)
$(gui_objects) : | $(gui_build_dirs)
$(gui_objects) : $(gui_generated_headers)
$(gui_generated_headers) : | $(gui_build_dirs)

$(sort $(libsolver_build_dirs) $(cli_build_dirs) $(gui_build_dirs) $(bin_dir)):
	mkdir -p $@

-include $(libsolver_depfiles)
-include $(cli_depfiles)
-include $(gui_depfiles)
