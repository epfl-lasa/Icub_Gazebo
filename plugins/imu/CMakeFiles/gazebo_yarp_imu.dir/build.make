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
include plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/depend.make

# Include the progress variables for this target.
include plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/progress.make

# Include the compile flags for this target's objects.
include plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/flags.make

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/flags.make
plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o: plugins/imu/src/IMU.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/neda/iCube/gazebo-yarp-plugins/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/imu && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o -c /home/neda/iCube/gazebo-yarp-plugins/plugins/imu/src/IMU.cc

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.i"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/imu && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/neda/iCube/gazebo-yarp-plugins/plugins/imu/src/IMU.cc > CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.i

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.s"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/imu && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/neda/iCube/gazebo-yarp-plugins/plugins/imu/src/IMU.cc -o CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.s

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o.requires:
.PHONY : plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o.requires

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o.provides: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o.requires
	$(MAKE) -f plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/build.make plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o.provides.build
.PHONY : plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o.provides

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o.provides.build: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/flags.make
plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o: plugins/imu/src/IMUDriver.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/neda/iCube/gazebo-yarp-plugins/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/imu && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o -c /home/neda/iCube/gazebo-yarp-plugins/plugins/imu/src/IMUDriver.cpp

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.i"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/imu && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/neda/iCube/gazebo-yarp-plugins/plugins/imu/src/IMUDriver.cpp > CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.i

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.s"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/imu && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/neda/iCube/gazebo-yarp-plugins/plugins/imu/src/IMUDriver.cpp -o CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.s

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o.requires:
.PHONY : plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o.requires

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o.provides: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o.requires
	$(MAKE) -f plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/build.make plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o.provides.build
.PHONY : plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o.provides

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o.provides.build: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o

# Object files for target gazebo_yarp_imu
gazebo_yarp_imu_OBJECTS = \
"CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o" \
"CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o"

# External object files for target gazebo_yarp_imu
gazebo_yarp_imu_EXTERNAL_OBJECTS =

plugins/imu/libgazebo_yarp_imu.so: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o
plugins/imu/libgazebo_yarp_imu.so: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o
plugins/imu/libgazebo_yarp_imu.so: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/build.make
plugins/imu/libgazebo_yarp_imu.so: libraries/singleton/libgazebo_yarp_singleton.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.66
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.66
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libYARP_math.so.2.3.66
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libYARP_dev.so.2.3.66
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libYARP_init.so.2.3.66
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libYARP_name.so.2.3.66
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_building.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_viewers.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui_model.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_selection_buffer.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_bullet.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_simbody.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_player.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics_ode.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering_deferred.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_skyx.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_math.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libYARP_sig.so.2.3.66
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libYARP_OS.so.2.3.66
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
plugins/imu/libgazebo_yarp_imu.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
plugins/imu/libgazebo_yarp_imu.so: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libgazebo_yarp_imu.so"
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/imu && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gazebo_yarp_imu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/build: plugins/imu/libgazebo_yarp_imu.so
.PHONY : plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/build

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/requires: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMU.cc.o.requires
plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/requires: plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/src/IMUDriver.cpp.o.requires
.PHONY : plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/requires

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/clean:
	cd /home/neda/iCube/gazebo-yarp-plugins/plugins/imu && $(CMAKE_COMMAND) -P CMakeFiles/gazebo_yarp_imu.dir/cmake_clean.cmake
.PHONY : plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/clean

plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/depend:
	cd /home/neda/iCube/gazebo-yarp-plugins && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/neda/iCube/gazebo-yarp-plugins /home/neda/iCube/gazebo-yarp-plugins/plugins/imu /home/neda/iCube/gazebo-yarp-plugins /home/neda/iCube/gazebo-yarp-plugins/plugins/imu /home/neda/iCube/gazebo-yarp-plugins/plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : plugins/imu/CMakeFiles/gazebo_yarp_imu.dir/depend

