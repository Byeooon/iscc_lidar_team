# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/foscar/hwamok_lidar/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/foscar/hwamok_lidar/build

# Utility rule file for clean_test_results_velodyne_driver.

# Include the progress variables for this target.
include velodyne/velodyne_driver/CMakeFiles/clean_test_results_velodyne_driver.dir/progress.make

velodyne/velodyne_driver/CMakeFiles/clean_test_results_velodyne_driver:
	cd /home/foscar/hwamok_lidar/build/velodyne/velodyne_driver && /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/remove_test_results.py /home/foscar/hwamok_lidar/build/test_results/velodyne_driver

clean_test_results_velodyne_driver: velodyne/velodyne_driver/CMakeFiles/clean_test_results_velodyne_driver
clean_test_results_velodyne_driver: velodyne/velodyne_driver/CMakeFiles/clean_test_results_velodyne_driver.dir/build.make

.PHONY : clean_test_results_velodyne_driver

# Rule to build all files generated by this target.
velodyne/velodyne_driver/CMakeFiles/clean_test_results_velodyne_driver.dir/build: clean_test_results_velodyne_driver

.PHONY : velodyne/velodyne_driver/CMakeFiles/clean_test_results_velodyne_driver.dir/build

velodyne/velodyne_driver/CMakeFiles/clean_test_results_velodyne_driver.dir/clean:
	cd /home/foscar/hwamok_lidar/build/velodyne/velodyne_driver && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_velodyne_driver.dir/cmake_clean.cmake
.PHONY : velodyne/velodyne_driver/CMakeFiles/clean_test_results_velodyne_driver.dir/clean

velodyne/velodyne_driver/CMakeFiles/clean_test_results_velodyne_driver.dir/depend:
	cd /home/foscar/hwamok_lidar/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/foscar/hwamok_lidar/src /home/foscar/hwamok_lidar/src/velodyne/velodyne_driver /home/foscar/hwamok_lidar/build /home/foscar/hwamok_lidar/build/velodyne/velodyne_driver /home/foscar/hwamok_lidar/build/velodyne/velodyne_driver/CMakeFiles/clean_test_results_velodyne_driver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : velodyne/velodyne_driver/CMakeFiles/clean_test_results_velodyne_driver.dir/depend

