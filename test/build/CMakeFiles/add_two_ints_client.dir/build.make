# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dimilar/ros/external/shulei/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dimilar/ros/external/shulei/test/build

# Include any dependencies generated for this target.
include CMakeFiles/add_two_ints_client.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/add_two_ints_client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/add_two_ints_client.dir/flags.make

CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: CMakeFiles/add_two_ints_client.dir/flags.make
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: ../src/add_two_ints_client.cpp
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: ../manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/core/roslang/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/core/genmsg_cpp/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/tools/rospack/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/core/roslib/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/3rdparty/xmlrpcpp/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/core/rosconsole/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/core/roscpp/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/core/rospy/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/std_msgs/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/tools/rosclean/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/tools/rosgraph/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/core/rosmaster/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/core/rosout/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/tools/roslaunch/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/test/rostest/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/tools/topic_tools/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/tools/rosbag/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/tools/rosmsg/manifest.xml
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/core/roslib/msg_gen/generated
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/core/roscpp/msg_gen/generated
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/core/roscpp/srv_gen/generated
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/std_msgs/msg_gen/generated
CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o: /opt/ros/cturtle/ros/tools/topic_tools/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dimilar/ros/external/shulei/test/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o -c /home/dimilar/ros/external/shulei/test/src/add_two_ints_client.cpp

CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/dimilar/ros/external/shulei/test/src/add_two_ints_client.cpp > CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.i

CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/dimilar/ros/external/shulei/test/src/add_two_ints_client.cpp -o CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.s

CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o.requires:
.PHONY : CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o.requires

CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o.provides: CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o.requires
	$(MAKE) -f CMakeFiles/add_two_ints_client.dir/build.make CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o.provides.build
.PHONY : CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o.provides

CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o.provides.build: CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o
.PHONY : CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o.provides.build

# Object files for target add_two_ints_client
add_two_ints_client_OBJECTS = \
"CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o"

# External object files for target add_two_ints_client
add_two_ints_client_EXTERNAL_OBJECTS =

../bin/add_two_ints_client: CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o
../bin/add_two_ints_client: CMakeFiles/add_two_ints_client.dir/build.make
../bin/add_two_ints_client: CMakeFiles/add_two_ints_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/add_two_ints_client"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/add_two_ints_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/add_two_ints_client.dir/build: ../bin/add_two_ints_client
.PHONY : CMakeFiles/add_two_ints_client.dir/build

CMakeFiles/add_two_ints_client.dir/requires: CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.o.requires
.PHONY : CMakeFiles/add_two_ints_client.dir/requires

CMakeFiles/add_two_ints_client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/add_two_ints_client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/add_two_ints_client.dir/clean

CMakeFiles/add_two_ints_client.dir/depend:
	cd /home/dimilar/ros/external/shulei/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dimilar/ros/external/shulei/test /home/dimilar/ros/external/shulei/test /home/dimilar/ros/external/shulei/test/build /home/dimilar/ros/external/shulei/test/build /home/dimilar/ros/external/shulei/test/build/CMakeFiles/add_two_ints_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/add_two_ints_client.dir/depend

