# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build

# Utility rule file for clean_test_results_ros_numpy.

# Include the progress variables for this target.
include ros_numpy/CMakeFiles/clean_test_results_ros_numpy.dir/progress.make

ros_numpy/CMakeFiles/clean_test_results_ros_numpy:
	cd /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/ros_numpy && /usr/bin/python /opt/ros/kinetic/share/catkin/cmake/test/remove_test_results.py /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/test_results/ros_numpy

clean_test_results_ros_numpy: ros_numpy/CMakeFiles/clean_test_results_ros_numpy
clean_test_results_ros_numpy: ros_numpy/CMakeFiles/clean_test_results_ros_numpy.dir/build.make

.PHONY : clean_test_results_ros_numpy

# Rule to build all files generated by this target.
ros_numpy/CMakeFiles/clean_test_results_ros_numpy.dir/build: clean_test_results_ros_numpy

.PHONY : ros_numpy/CMakeFiles/clean_test_results_ros_numpy.dir/build

ros_numpy/CMakeFiles/clean_test_results_ros_numpy.dir/clean:
	cd /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/ros_numpy && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_ros_numpy.dir/cmake_clean.cmake
.PHONY : ros_numpy/CMakeFiles/clean_test_results_ros_numpy.dir/clean

ros_numpy/CMakeFiles/clean_test_results_ros_numpy.dir/depend:
	cd /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/src /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/src/ros_numpy /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/ros_numpy /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/ros_numpy/CMakeFiles/clean_test_results_ros_numpy.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ros_numpy/CMakeFiles/clean_test_results_ros_numpy.dir/depend

