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
CMAKE_SOURCE_DIR = /home/ae/Desktop/kolstar_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ae/Desktop/kolstar_test/build

# Include any dependencies generated for this target.
include CMakeFiles/showplanlist.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/showplanlist.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/showplanlist.dir/flags.make

CMakeFiles/showplanlist.dir/source/showplanlist.cpp.o: CMakeFiles/showplanlist.dir/flags.make
CMakeFiles/showplanlist.dir/source/showplanlist.cpp.o: ../source/showplanlist.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ae/Desktop/kolstar_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/showplanlist.dir/source/showplanlist.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/showplanlist.dir/source/showplanlist.cpp.o -c /home/ae/Desktop/kolstar_test/source/showplanlist.cpp

CMakeFiles/showplanlist.dir/source/showplanlist.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/showplanlist.dir/source/showplanlist.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ae/Desktop/kolstar_test/source/showplanlist.cpp > CMakeFiles/showplanlist.dir/source/showplanlist.cpp.i

CMakeFiles/showplanlist.dir/source/showplanlist.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/showplanlist.dir/source/showplanlist.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ae/Desktop/kolstar_test/source/showplanlist.cpp -o CMakeFiles/showplanlist.dir/source/showplanlist.cpp.s

# Object files for target showplanlist
showplanlist_OBJECTS = \
"CMakeFiles/showplanlist.dir/source/showplanlist.cpp.o"

# External object files for target showplanlist
showplanlist_EXTERNAL_OBJECTS =

source/showplanlist: CMakeFiles/showplanlist.dir/source/showplanlist.cpp.o
source/showplanlist: CMakeFiles/showplanlist.dir/build.make
source/showplanlist: ../lib/libFlexivRdk.a
source/showplanlist: ../thirdparty/Ginko/lib/linux_64bit/libGinkgo_Driver.so
source/showplanlist: CMakeFiles/showplanlist.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ae/Desktop/kolstar_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable source/showplanlist"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/showplanlist.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/showplanlist.dir/build: source/showplanlist

.PHONY : CMakeFiles/showplanlist.dir/build

CMakeFiles/showplanlist.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/showplanlist.dir/cmake_clean.cmake
.PHONY : CMakeFiles/showplanlist.dir/clean

CMakeFiles/showplanlist.dir/depend:
	cd /home/ae/Desktop/kolstar_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ae/Desktop/kolstar_test /home/ae/Desktop/kolstar_test /home/ae/Desktop/kolstar_test/build /home/ae/Desktop/kolstar_test/build /home/ae/Desktop/kolstar_test/build/CMakeFiles/showplanlist.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/showplanlist.dir/depend

