# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /snap/clion/73/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/73/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/uc0de/CLionProjects/openeyes_core

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/uc0de/CLionProjects/openeyes_core/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/openeyes_core.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/openeyes_core.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/openeyes_core.dir/flags.make

CMakeFiles/openeyes_core.dir/main.cpp.o: CMakeFiles/openeyes_core.dir/flags.make
CMakeFiles/openeyes_core.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/uc0de/CLionProjects/openeyes_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/openeyes_core.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/openeyes_core.dir/main.cpp.o -c /home/uc0de/CLionProjects/openeyes_core/main.cpp

CMakeFiles/openeyes_core.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/openeyes_core.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/uc0de/CLionProjects/openeyes_core/main.cpp > CMakeFiles/openeyes_core.dir/main.cpp.i

CMakeFiles/openeyes_core.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/openeyes_core.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/uc0de/CLionProjects/openeyes_core/main.cpp -o CMakeFiles/openeyes_core.dir/main.cpp.s

CMakeFiles/openeyes_core.dir/serial.cpp.o: CMakeFiles/openeyes_core.dir/flags.make
CMakeFiles/openeyes_core.dir/serial.cpp.o: ../serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/uc0de/CLionProjects/openeyes_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/openeyes_core.dir/serial.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/openeyes_core.dir/serial.cpp.o -c /home/uc0de/CLionProjects/openeyes_core/serial.cpp

CMakeFiles/openeyes_core.dir/serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/openeyes_core.dir/serial.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/uc0de/CLionProjects/openeyes_core/serial.cpp > CMakeFiles/openeyes_core.dir/serial.cpp.i

CMakeFiles/openeyes_core.dir/serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/openeyes_core.dir/serial.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/uc0de/CLionProjects/openeyes_core/serial.cpp -o CMakeFiles/openeyes_core.dir/serial.cpp.s

# Object files for target openeyes_core
openeyes_core_OBJECTS = \
"CMakeFiles/openeyes_core.dir/main.cpp.o" \
"CMakeFiles/openeyes_core.dir/serial.cpp.o"

# External object files for target openeyes_core
openeyes_core_EXTERNAL_OBJECTS =

openeyes_core: CMakeFiles/openeyes_core.dir/main.cpp.o
openeyes_core: CMakeFiles/openeyes_core.dir/serial.cpp.o
openeyes_core: CMakeFiles/openeyes_core.dir/build.make
openeyes_core: /usr/lib/x86_64-linux-gnu/librealsense2.so.2.22.0
openeyes_core: /usr/local/lib/libopencv_dnn.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_gapi.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_highgui.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_ml.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_objdetect.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_photo.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_stitching.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_video.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_videoio.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_imgcodecs.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_calib3d.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_features2d.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_flann.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_imgproc.so.4.1.0
openeyes_core: /usr/local/lib/libopencv_core.so.4.1.0
openeyes_core: CMakeFiles/openeyes_core.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/uc0de/CLionProjects/openeyes_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable openeyes_core"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/openeyes_core.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/openeyes_core.dir/build: openeyes_core

.PHONY : CMakeFiles/openeyes_core.dir/build

CMakeFiles/openeyes_core.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/openeyes_core.dir/cmake_clean.cmake
.PHONY : CMakeFiles/openeyes_core.dir/clean

CMakeFiles/openeyes_core.dir/depend:
	cd /home/uc0de/CLionProjects/openeyes_core/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/uc0de/CLionProjects/openeyes_core /home/uc0de/CLionProjects/openeyes_core /home/uc0de/CLionProjects/openeyes_core/cmake-build-debug /home/uc0de/CLionProjects/openeyes_core/cmake-build-debug /home/uc0de/CLionProjects/openeyes_core/cmake-build-debug/CMakeFiles/openeyes_core.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/openeyes_core.dir/depend

