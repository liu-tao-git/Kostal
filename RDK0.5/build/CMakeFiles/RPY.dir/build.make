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


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/flexiv/Documents/Case/2022/Kostal/RDK0.5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/flexiv/Documents/Case/2022/Kostal/RDK0.5/build

# Include any dependencies generated for this target.
include CMakeFiles/RPY.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RPY.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RPY.dir/flags.make

CMakeFiles/RPY.dir/source/RPY.cpp.o: CMakeFiles/RPY.dir/flags.make
CMakeFiles/RPY.dir/source/RPY.cpp.o: ../source/RPY.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/flexiv/Documents/Case/2022/Kostal/RDK0.5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RPY.dir/source/RPY.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RPY.dir/source/RPY.cpp.o -c /home/flexiv/Documents/Case/2022/Kostal/RDK0.5/source/RPY.cpp

CMakeFiles/RPY.dir/source/RPY.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RPY.dir/source/RPY.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/flexiv/Documents/Case/2022/Kostal/RDK0.5/source/RPY.cpp > CMakeFiles/RPY.dir/source/RPY.cpp.i

CMakeFiles/RPY.dir/source/RPY.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RPY.dir/source/RPY.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/flexiv/Documents/Case/2022/Kostal/RDK0.5/source/RPY.cpp -o CMakeFiles/RPY.dir/source/RPY.cpp.s

# Object files for target RPY
RPY_OBJECTS = \
"CMakeFiles/RPY.dir/source/RPY.cpp.o"

# External object files for target RPY
RPY_EXTERNAL_OBJECTS =

source/RPY: CMakeFiles/RPY.dir/source/RPY.cpp.o
source/RPY: CMakeFiles/RPY.dir/build.make
source/RPY: ../lib/libFlexivRdk.a
source/RPY: ../thirdparty/Ginko/lib/linux_64bit/libGinkgo_Driver.so
source/RPY: CMakeFiles/RPY.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/flexiv/Documents/Case/2022/Kostal/RDK0.5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable source/RPY"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RPY.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RPY.dir/build: source/RPY

.PHONY : CMakeFiles/RPY.dir/build

CMakeFiles/RPY.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RPY.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RPY.dir/clean

CMakeFiles/RPY.dir/depend:
	cd /home/flexiv/Documents/Case/2022/Kostal/RDK0.5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/flexiv/Documents/Case/2022/Kostal/RDK0.5 /home/flexiv/Documents/Case/2022/Kostal/RDK0.5 /home/flexiv/Documents/Case/2022/Kostal/RDK0.5/build /home/flexiv/Documents/Case/2022/Kostal/RDK0.5/build /home/flexiv/Documents/Case/2022/Kostal/RDK0.5/build/CMakeFiles/RPY.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RPY.dir/depend
