# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/neda/iCube/gazebo-yarp-plugins

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/neda/iCube/gazebo-yarp-plugins

# Include any dependencies generated for this target.
include plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/depend.make

# Include the progress variables for this target.
include plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/progress.make

# Include the compile flags for this target's objects.
include plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/flags.make

plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o: plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/flags.make
plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o: plugins/showmodelcom/src/ShowModelCoM.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/neda/iCube/gazebo-yarp-plugins/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o -c /home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom/src/ShowModelCoM.cc

plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.i"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom/src/ShowModelCoM.cc > CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.i

plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.s"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom/src/ShowModelCoM.cc -o CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.s

plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o.requires:
.PHONY : plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o.requires

plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o.provides: plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o.requires
	$(MAKE) -f plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/build.make plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o.provides.build
.PHONY : plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o.provides

plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o.provides.build: plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o

# Object files for target gazebo_yarp_showmodelcom
gazebo_yarp_showmodelcom_OBJECTS = \
"CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o"

# External object files for target gazebo_yarp_showmodelcom
gazebo_yarp_showmodelcom_EXTERNAL_OBJECTS =

plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/build.make
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.66
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.66
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libYARP_math.so.2.3.66
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libYARP_dev.so.2.3.66
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libYARP_init.so.2.3.66
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libYARP_name.so.2.3.66
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_building.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_viewers.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_model.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_selection_buffer.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_bullet.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_simbody.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_player.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_ode.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering_deferred.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_skyx.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_math.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.66
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.66
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
plugins/showmodelcom/libgazebo_yarp_showmodelcom.so: plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libgazebo_yarp_showmodelcom.so"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gazebo_yarp_showmodelcom.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/build: plugins/showmodelcom/libgazebo_yarp_showmodelcom.so
.PHONY : plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/build

plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/requires: plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/src/ShowModelCoM.cc.o.requires
.PHONY : plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/requires

plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/clean:
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom && $(CMAKE_COMMAND) -P CMakeFiles/gazebo_yarp_showmodelcom.dir/cmake_clean.cmake
.PHONY : plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/clean

plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/depend:
	cd /home/neda/iCube/gazebo-yarp-plugins && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/neda/iCube/gazebo-yarp-plugins /home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom /home/neda/iCube/gazebo-yarp-plugins /home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom /home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : plugins/showmodelcom/CMakeFiles/gazebo_yarp_showmodelcom.dir/depend

