CXXFLAGS += -Wall -Wextra -std=c++14 -pedantic

mode ?= opt

-include site-config.make

eigen_include_dir ?= /usr/include/eigen3
qt_include_dir ?= /usr/include/qt5
moc ?= moc-qt5
uic ?= /usr/lib64/qt5/bin/uic
boost_system_lib ?= boost_system
boost_filesystem_lib ?= boost_filesystem
boost_program_options_lib ?= boost_program_options

gui_ldflags ?=

CXXFLAGS += -fPIC

ifeq ($(mode),opt)
	CXXFLAGS += -O3 -DNDEBUG
endif

ifeq ($(mode),profile)
	CXXFLAGS += -O3 -DNDEBUG -ggdb3
endif

ifeq ($(mode),debug)
	CXXFLAGS += -ggdb3
endif

project_dir = src
build_dir = build/$(mode)
bin_dir = bin/$(mode)

find_sources = $(wildcard $(1)/*.cpp)
find_headers = $(wildcard $(1)/*.hpp)
find_uis = $(wildcard $(1)/*.ui)
object_names = $(addprefix $(build_dir)/,$(1:.cpp=.o))
depfile_names = $(addprefix $(build_dir)/,$(1:.cpp=.d))
ui_header_names = $(foreach ui,$(1),$(build_dir)/$(dir $(ui))ui_$(basename $(notdir $(ui))).h)

define make_subproj
$(1)_dir = $(project_dir)/$(1)
$(1)_sources = $$(call find_sources,$$($(1)_dir))
$(1)_objects = $$(call object_names,$$($(1)_sources))
$(1)_depfiles = $$(call depfile_names,$$($(1)_sources))
endef

$(eval $(call make_subproj,libsolver))
libsolver_lib = $(build_dir)/libsolver.a
libsolver_libname = solver
libsolver_includes = -isystem$(eigen_include_dir)

$(eval $(call make_subproj,cli))
cli_executable = $(bin_dir)/cli
cli_includes = -I$(libsolver_dir)
cli_ldlibs = -l$(boost_program_options_lib) -l$(boost_filesystem_lib) -l$(boost_system_lib)

$(eval $(call make_subproj,gui))
gui_objects += $(call object_names,$(gui_moc_sources))
gui_moc_headers = $(call find_headers,$(gui_dir))
gui_moc_sources = $(gui_moc_headers:.hpp=.moc.cpp)
gui_ui_sources = $(call find_uis,$(gui_dir))
gui_generated_headers = $(call ui_header_names,$(gui_ui_sources))
gui_executable = $(bin_dir)/gui
gui_qt_modules = Core Gui Widgets
gui_includes = -I$(libsolver_dir) -I$(qt_include_dir) $(addprefix -I$(qt_include_dir)/Qt,$(gui_qt_modules))
gui_includes += $(sort $(addprefix -I,$(dir $(gui_generated_headers))))
gui_includes += -I$(gui_dir)
gui_libs = $(addprefix -lQt5,$(gui_qt_modules))
gui_libs += -l$(boost_filesystem_lib) -l$(boost_system_lib)

outputs = $(libsolver_objects) $(libsolver_lib) $(libsolver_depfiles)
outputs += $(cli_objects) $(cli_executable) $(cli_depfiles)
outputs += $(gui_objects) $(gui_generated_headers) $(gui_executable) $(gui_depfiles)

delete ?= rm -f $1
make_dir ?= mkdir -p $1

post_build ?=

.PHONY: all
all: cli gui

.PHONY: clean
clean:
	$(call delete,$(outputs))

.PHONY: cli
cli: $(cli_executable)

.PHONY: gui
gui: $(gui_executable)

$(libsolver_lib) : $(libsolver_objects)
	$(call delete,$(libsolver_lib))
	$(AR) cqs $@ $^

link = $(CXX) $(1) $(LDFLAGS) -l$(libsolver_libname) $(LDLIBS) -o $@

$(cli_executable) : LDFLAGS += -L$(build_dir)
$(cli_executable) : LDLIBS += $(cli_ldlibs)
$(cli_executable) : $(cli_objects) $(libsolver_lib)
	$(call link,$(cli_objects))
	$(call post_build,$(cli_executable))

$(gui_executable) : LDFLAGS += -L$(build_dir)
$(gui_executable) : LDFLAGS += $(gui_ldflags)
$(gui_executable) : LDLIBS += $(gui_libs)
$(gui_executable) : incdirs += $(gui_includes)
$(gui_executable) : CXXFLAGS += $(gui_includes)
$(gui_executable) : $(gui_objects) $(libsolver_lib)
	$(call link,$(gui_objects))
	$(call post_build,$(gui_executable))

$(build_dir)/%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ -MMD -MP

$(build_dir)/%.moc.cpp : %.hpp
	$(moc) $(incdirs) $< -o $@

$(build_dir)/%.ui : %.ui
	cp $< $@

ui_%.h : %.ui
	$(uic) $< -o $@

$(libsolver_objects) : CXXFLAGS += $(libsolver_includes)

$(cli_objects) : CXXFLAGS += $(cli_includes)
# Boost.ProgramOptions won't link if we compile our .cpp with debugging stdlib
$(cli_objects) : CXXFLAGS := $(filter-out -D_GLIBCXX_DEBUG -D_GLIBCXX_DEBUG_PEDANTIC,$(CXXFLAGS))

$(gui_objects) : CXXFLAGS += $(gui_includes)
$(gui_objects) : $(gui_generated_headers)

define depend_on_dir
$(1) : | $(dir $(1))
endef

$(foreach out,$(outputs) $(cli_executable) $(gui_executable),$(eval $(call depend_on_dir,$(out))))

$(sort $(foreach out,$(outputs),$(dir $(out)))):
	$(call make_dir,$@)

-include $(libsolver_depfiles)
-include $(cli_depfiles)
-include $(gui_depfiles)
