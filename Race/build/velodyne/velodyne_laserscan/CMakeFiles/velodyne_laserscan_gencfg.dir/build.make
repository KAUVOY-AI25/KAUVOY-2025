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
CMAKE_SOURCE_DIR = /home/kauvoy/Race/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kauvoy/Race/build

# Utility rule file for velodyne_laserscan_gencfg.

# Include the progress variables for this target.
include velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg.dir/progress.make

velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg: /home/kauvoy/Race/devel/include/velodyne_laserscan/VelodyneLaserScanConfig.h
velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg: /home/kauvoy/Race/devel/lib/python3/dist-packages/velodyne_laserscan/cfg/VelodyneLaserScanConfig.py


/home/kauvoy/Race/devel/include/velodyne_laserscan/VelodyneLaserScanConfig.h: /home/kauvoy/Race/src/velodyne/velodyne_laserscan/cfg/VelodyneLaserScan.cfg
/home/kauvoy/Race/devel/include/velodyne_laserscan/VelodyneLaserScanConfig.h: /opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.py.template
/home/kauvoy/Race/devel/include/velodyne_laserscan/VelodyneLaserScanConfig.h: /opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kauvoy/Race/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating dynamic reconfigure files from cfg/VelodyneLaserScan.cfg: /home/kauvoy/Race/devel/include/velodyne_laserscan/VelodyneLaserScanConfig.h /home/kauvoy/Race/devel/lib/python3/dist-packages/velodyne_laserscan/cfg/VelodyneLaserScanConfig.py"
	cd /home/kauvoy/Race/build/velodyne/velodyne_laserscan && ../../catkin_generated/env_cached.sh /home/kauvoy/Race/build/velodyne/velodyne_laserscan/setup_custom_pythonpath.sh /home/kauvoy/Race/src/velodyne/velodyne_laserscan/cfg/VelodyneLaserScan.cfg /opt/ros/noetic/share/dynamic_reconfigure/cmake/.. /home/kauvoy/Race/devel/share/velodyne_laserscan /home/kauvoy/Race/devel/include/velodyne_laserscan /home/kauvoy/Race/devel/lib/python3/dist-packages/velodyne_laserscan

/home/kauvoy/Race/devel/share/velodyne_laserscan/docs/VelodyneLaserScanConfig.dox: /home/kauvoy/Race/devel/include/velodyne_laserscan/VelodyneLaserScanConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/kauvoy/Race/devel/share/velodyne_laserscan/docs/VelodyneLaserScanConfig.dox

/home/kauvoy/Race/devel/share/velodyne_laserscan/docs/VelodyneLaserScanConfig-usage.dox: /home/kauvoy/Race/devel/include/velodyne_laserscan/VelodyneLaserScanConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/kauvoy/Race/devel/share/velodyne_laserscan/docs/VelodyneLaserScanConfig-usage.dox

/home/kauvoy/Race/devel/lib/python3/dist-packages/velodyne_laserscan/cfg/VelodyneLaserScanConfig.py: /home/kauvoy/Race/devel/include/velodyne_laserscan/VelodyneLaserScanConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/kauvoy/Race/devel/lib/python3/dist-packages/velodyne_laserscan/cfg/VelodyneLaserScanConfig.py

/home/kauvoy/Race/devel/share/velodyne_laserscan/docs/VelodyneLaserScanConfig.wikidoc: /home/kauvoy/Race/devel/include/velodyne_laserscan/VelodyneLaserScanConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/kauvoy/Race/devel/share/velodyne_laserscan/docs/VelodyneLaserScanConfig.wikidoc

velodyne_laserscan_gencfg: velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg
velodyne_laserscan_gencfg: /home/kauvoy/Race/devel/include/velodyne_laserscan/VelodyneLaserScanConfig.h
velodyne_laserscan_gencfg: /home/kauvoy/Race/devel/share/velodyne_laserscan/docs/VelodyneLaserScanConfig.dox
velodyne_laserscan_gencfg: /home/kauvoy/Race/devel/share/velodyne_laserscan/docs/VelodyneLaserScanConfig-usage.dox
velodyne_laserscan_gencfg: /home/kauvoy/Race/devel/lib/python3/dist-packages/velodyne_laserscan/cfg/VelodyneLaserScanConfig.py
velodyne_laserscan_gencfg: /home/kauvoy/Race/devel/share/velodyne_laserscan/docs/VelodyneLaserScanConfig.wikidoc
velodyne_laserscan_gencfg: velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg.dir/build.make

.PHONY : velodyne_laserscan_gencfg

# Rule to build all files generated by this target.
velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg.dir/build: velodyne_laserscan_gencfg

.PHONY : velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg.dir/build

velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg.dir/clean:
	cd /home/kauvoy/Race/build/velodyne/velodyne_laserscan && $(CMAKE_COMMAND) -P CMakeFiles/velodyne_laserscan_gencfg.dir/cmake_clean.cmake
.PHONY : velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg.dir/clean

velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg.dir/depend:
	cd /home/kauvoy/Race/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kauvoy/Race/src /home/kauvoy/Race/src/velodyne/velodyne_laserscan /home/kauvoy/Race/build /home/kauvoy/Race/build/velodyne/velodyne_laserscan /home/kauvoy/Race/build/velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : velodyne/velodyne_laserscan/CMakeFiles/velodyne_laserscan_gencfg.dir/depend

