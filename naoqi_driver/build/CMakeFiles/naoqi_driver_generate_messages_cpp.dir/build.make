# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/yoha/cmake-3.27.9-linux-x86_64/bin/cmake

# The command to remove a file.
RM = /home/yoha/cmake-3.27.9-linux-x86_64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver/build

# Utility rule file for naoqi_driver_generate_messages_cpp.

# Include any custom commands dependencies for this target.
include CMakeFiles/naoqi_driver_generate_messages_cpp.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/naoqi_driver_generate_messages_cpp.dir/progress.make

CMakeFiles/naoqi_driver_generate_messages_cpp: devel/include/naoqi_driver/AudioCustomMsg.h

devel/include/naoqi_driver/AudioCustomMsg.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
devel/include/naoqi_driver/AudioCustomMsg.h: /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver/msg/AudioCustomMsg.msg
devel/include/naoqi_driver/AudioCustomMsg.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/yoha/workspace/pepper_rob_ws/src/naoqi_driver/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from naoqi_driver/AudioCustomMsg.msg"
	cd /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver && /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver/msg/AudioCustomMsg.msg -Inaoqi_driver:/home/yoha/workspace/pepper_rob_ws/src/naoqi_driver/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p naoqi_driver -o /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver/build/devel/include/naoqi_driver -e /opt/ros/noetic/share/gencpp/cmake/..

naoqi_driver_generate_messages_cpp: CMakeFiles/naoqi_driver_generate_messages_cpp
naoqi_driver_generate_messages_cpp: devel/include/naoqi_driver/AudioCustomMsg.h
naoqi_driver_generate_messages_cpp: CMakeFiles/naoqi_driver_generate_messages_cpp.dir/build.make
.PHONY : naoqi_driver_generate_messages_cpp

# Rule to build all files generated by this target.
CMakeFiles/naoqi_driver_generate_messages_cpp.dir/build: naoqi_driver_generate_messages_cpp
.PHONY : CMakeFiles/naoqi_driver_generate_messages_cpp.dir/build

CMakeFiles/naoqi_driver_generate_messages_cpp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/naoqi_driver_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/naoqi_driver_generate_messages_cpp.dir/clean

CMakeFiles/naoqi_driver_generate_messages_cpp.dir/depend:
	cd /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver/build /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver/build /home/yoha/workspace/pepper_rob_ws/src/naoqi_driver/build/CMakeFiles/naoqi_driver_generate_messages_cpp.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/naoqi_driver_generate_messages_cpp.dir/depend
