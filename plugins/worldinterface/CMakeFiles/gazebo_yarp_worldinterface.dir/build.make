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
include plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/depend.make

# Include the progress variables for this target.
include plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/progress.make

# Include the compile flags for this target's objects.
include plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/flags.make

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/flags.make
plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o: plugins/worldinterface/src/worldinterface.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/neda/iCube/gazebo-yarp-plugins/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o -c /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface/src/worldinterface.cpp

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.i"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface/src/worldinterface.cpp > CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.i

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.s"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface/src/worldinterface.cpp -o CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.s

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o.requires:
.PHONY : plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o.requires

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o.provides: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o.requires
	$(MAKE) -f plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/build.make plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o.provides.build
.PHONY : plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o.provides

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o.provides.build: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/flags.make
plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o: plugins/worldinterface/src/worldinterfaceserverimpl.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/neda/iCube/gazebo-yarp-plugins/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o -c /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface/src/worldinterfaceserverimpl.cpp

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.i"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface/src/worldinterfaceserverimpl.cpp > CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.i

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.s"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface/src/worldinterfaceserverimpl.cpp -o CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.s

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o.requires:
.PHONY : plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o.requires

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o.provides: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o.requires
	$(MAKE) -f plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/build.make plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o.provides.build
.PHONY : plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o.provides

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o.provides.build: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/flags.make
plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o: plugins/worldinterface/src/worldproxy.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/neda/iCube/gazebo-yarp-plugins/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o -c /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface/src/worldproxy.cpp

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.i"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface/src/worldproxy.cpp > CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.i

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.s"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface/src/worldproxy.cpp -o CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.s

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o.requires:
.PHONY : plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o.requires

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o.provides: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o.requires
	$(MAKE) -f plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/build.make plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o.provides.build
.PHONY : plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o.provides

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o.provides.build: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o

# Object files for target gazebo_yarp_worldinterface
gazebo_yarp_worldinterface_OBJECTS = \
"CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o" \
"CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o" \
"CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o"

# External object files for target gazebo_yarp_worldinterface
gazebo_yarp_worldinterface_EXTERNAL_OBJECTS =

plugins/worldinterface/libgazebo_yarp_worldinterface.so: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o
plugins/worldinterface/libgazebo_yarp_worldinterface.so: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o
plugins/worldinterface/libgazebo_yarp_worldinterface.so: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o
plugins/worldinterface/libgazebo_yarp_worldinterface.so: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/build.make
plugins/worldinterface/libgazebo_yarp_worldinterface.so: thrift/worldinterface/libgazebo_yarp_rpc_worldinterface.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: libraries/singleton/libgazebo_yarp_singleton.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.66
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.66
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libYARP_math.so.2.3.66
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libYARP_dev.so.2.3.66
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libYARP_init.so.2.3.66
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libYARP_name.so.2.3.66
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_building.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_viewers.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_model.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_selection_buffer.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_bullet.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_simbody.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_player.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_ode.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering_deferred.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_skyx.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_math.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.66
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.66
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
plugins/worldinterface/libgazebo_yarp_worldinterface.so: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libgazebo_yarp_worldinterface.so"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gazebo_yarp_worldinterface.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/build: plugins/worldinterface/libgazebo_yarp_worldinterface.so
.PHONY : plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/build

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/requires: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterface.cpp.o.requires
plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/requires: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldinterfaceserverimpl.cpp.o.requires
plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/requires: plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/src/worldproxy.cpp.o.requires
.PHONY : plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/requires

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/clean:
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface && $(CMAKE_COMMAND) -P CMakeFiles/gazebo_yarp_worldinterface.dir/cmake_clean.cmake
.PHONY : plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/clean

plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/depend:
	cd /home/neda/iCube/gazebo-yarp-plugins && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/neda/iCube/gazebo-yarp-plugins /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface /home/neda/iCube/gazebo-yarp-plugins /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface /home/neda/iCube/gazebo-yarp-plugins/plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : plugins/worldinterface/CMakeFiles/gazebo_yarp_worldinterface.dir/depend

