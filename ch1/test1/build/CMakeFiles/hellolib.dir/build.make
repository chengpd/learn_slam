# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/cp/learn_slam/ch1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cp/learn_slam/ch1/build

# Include any dependencies generated for this target.
include CMakeFiles/hellolib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hellolib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hellolib.dir/flags.make

CMakeFiles/hellolib.dir/hello.cpp.o: CMakeFiles/hellolib.dir/flags.make
CMakeFiles/hellolib.dir/hello.cpp.o: ../hello.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cp/learn_slam/ch1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hellolib.dir/hello.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hellolib.dir/hello.cpp.o -c /home/cp/learn_slam/ch1/hello.cpp

CMakeFiles/hellolib.dir/hello.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hellolib.dir/hello.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cp/learn_slam/ch1/hello.cpp > CMakeFiles/hellolib.dir/hello.cpp.i

CMakeFiles/hellolib.dir/hello.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hellolib.dir/hello.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cp/learn_slam/ch1/hello.cpp -o CMakeFiles/hellolib.dir/hello.cpp.s

CMakeFiles/hellolib.dir/hello.cpp.o.requires:

.PHONY : CMakeFiles/hellolib.dir/hello.cpp.o.requires

CMakeFiles/hellolib.dir/hello.cpp.o.provides: CMakeFiles/hellolib.dir/hello.cpp.o.requires
	$(MAKE) -f CMakeFiles/hellolib.dir/build.make CMakeFiles/hellolib.dir/hello.cpp.o.provides.build
.PHONY : CMakeFiles/hellolib.dir/hello.cpp.o.provides

CMakeFiles/hellolib.dir/hello.cpp.o.provides.build: CMakeFiles/hellolib.dir/hello.cpp.o


# Object files for target hellolib
hellolib_OBJECTS = \
"CMakeFiles/hellolib.dir/hello.cpp.o"

# External object files for target hellolib
hellolib_EXTERNAL_OBJECTS =

libhellolib.a: CMakeFiles/hellolib.dir/hello.cpp.o
libhellolib.a: CMakeFiles/hellolib.dir/build.make
libhellolib.a: CMakeFiles/hellolib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cp/learn_slam/ch1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libhellolib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/hellolib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hellolib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hellolib.dir/build: libhellolib.a

.PHONY : CMakeFiles/hellolib.dir/build

CMakeFiles/hellolib.dir/requires: CMakeFiles/hellolib.dir/hello.cpp.o.requires

.PHONY : CMakeFiles/hellolib.dir/requires

CMakeFiles/hellolib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hellolib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hellolib.dir/clean

CMakeFiles/hellolib.dir/depend:
	cd /home/cp/learn_slam/ch1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cp/learn_slam/ch1 /home/cp/learn_slam/ch1 /home/cp/learn_slam/ch1/build /home/cp/learn_slam/ch1/build /home/cp/learn_slam/ch1/build/CMakeFiles/hellolib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hellolib.dir/depend

