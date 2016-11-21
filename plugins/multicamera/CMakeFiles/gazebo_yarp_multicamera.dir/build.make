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
include plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/depend.make

# Include the progress variables for this target.
include plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/progress.make

# Include the compile flags for this target's objects.
include plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/flags.make

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/flags.make
plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o: plugins/multicamera/src/MultiCamera.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/neda/iCube/gazebo-yarp-plugins/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o -c /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera/src/MultiCamera.cc

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.i"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera/src/MultiCamera.cc > CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.i

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.s"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera/src/MultiCamera.cc -o CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.s

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o.requires:
.PHONY : plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o.requires

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o.provides: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o.requires
	$(MAKE) -f plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/build.make plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o.provides.build
.PHONY : plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o.provides

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o.provides.build: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/flags.make
plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o: plugins/multicamera/src/MultiCameraDriver.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/neda/iCube/gazebo-yarp-plugins/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o -c /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera/src/MultiCameraDriver.cpp

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.i"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera/src/MultiCameraDriver.cpp > CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.i

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.s"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera/src/MultiCameraDriver.cpp -o CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.s

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o.requires:
.PHONY : plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o.requires

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o.provides: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o.requires
	$(MAKE) -f plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/build.make plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o.provides.build
.PHONY : plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o.provides

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o.provides.build: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o

# Object files for target gazebo_yarp_multicamera
gazebo_yarp_multicamera_OBJECTS = \
"CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o" \
"CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o"

# External object files for target gazebo_yarp_multicamera
gazebo_yarp_multicamera_EXTERNAL_OBJECTS =

plugins/multicamera/libgazebo_yarp_multicamera.so: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o
plugins/multicamera/libgazebo_yarp_multicamera.so: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o
plugins/multicamera/libgazebo_yarp_multicamera.so: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/build.make
plugins/multicamera/libgazebo_yarp_multicamera.so: libraries/singleton/libgazebo_yarp_singleton.so
plugins/multicamera/libgazebo_yarp_multicamera.so: plugins/multicamera/libgazebo_yarp_MultiCameraPlugin.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.66
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.66
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libYARP_math.so.2.3.66
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libYARP_dev.so.2.3.66
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libYARP_init.so.2.3.66
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libYARP_name.so.2.3.66
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_building.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_viewers.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_model.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_selection_buffer.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_bullet.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_simbody.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_player.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_ode.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering_deferred.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_skyx.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_math.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.66
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.66
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
plugins/multicamera/libgazebo_yarp_multicamera.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
plugins/multicamera/libgazebo_yarp_multicamera.so: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libgazebo_yarp_multicamera.so"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gazebo_yarp_multicamera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/build: plugins/multicamera/libgazebo_yarp_multicamera.so
.PHONY : plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/build

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/requires: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCamera.cc.o.requires
plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/requires: plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/src/MultiCameraDriver.cpp.o.requires
.PHONY : plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/requires

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/clean:
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera && $(CMAKE_COMMAND) -P CMakeFiles/gazebo_yarp_multicamera.dir/cmake_clean.cmake
.PHONY : plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/clean

plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/depend:
	cd /home/neda/iCube/gazebo-yarp-plugins && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/neda/iCube/gazebo-yarp-plugins /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera /home/neda/iCube/gazebo-yarp-plugins /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera /home/neda/iCube/gazebo-yarp-plugins/plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : plugins/multicamera/CMakeFiles/gazebo_yarp_multicamera.dir/depend

