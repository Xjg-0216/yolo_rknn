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
CMAKE_SOURCE_DIR = /home/xujg/yolo_rknn/yolov5_bytetrack/cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/build/build_rk3588_linux

# Include any dependencies generated for this target.
include CMakeFiles/yolov5_image_demo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/yolov5_image_demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/yolov5_image_demo.dir/flags.make

CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.o: CMakeFiles/yolov5_image_demo.dir/flags.make
CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.o: ../../yolov5_image_demo.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xujg/yolo_rknn/yolov5_bytetrack/cpp/build/build_rk3588_linux/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.o"
	/usr/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.o -c /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/yolov5_image_demo.cc

CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/yolov5_image_demo.cc > CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.i

CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/yolov5_image_demo.cc -o CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.s

CMakeFiles/yolov5_image_demo.dir/postprocess.cc.o: CMakeFiles/yolov5_image_demo.dir/flags.make
CMakeFiles/yolov5_image_demo.dir/postprocess.cc.o: ../../postprocess.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xujg/yolo_rknn/yolov5_bytetrack/cpp/build/build_rk3588_linux/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/yolov5_image_demo.dir/postprocess.cc.o"
	/usr/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yolov5_image_demo.dir/postprocess.cc.o -c /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/postprocess.cc

CMakeFiles/yolov5_image_demo.dir/postprocess.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yolov5_image_demo.dir/postprocess.cc.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/postprocess.cc > CMakeFiles/yolov5_image_demo.dir/postprocess.cc.i

CMakeFiles/yolov5_image_demo.dir/postprocess.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yolov5_image_demo.dir/postprocess.cc.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/postprocess.cc -o CMakeFiles/yolov5_image_demo.dir/postprocess.cc.s

CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.o: CMakeFiles/yolov5_image_demo.dir/flags.make
CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.o: ../../rknpu2/yolov5.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xujg/yolo_rknn/yolov5_bytetrack/cpp/build/build_rk3588_linux/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.o"
	/usr/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.o -c /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/rknpu2/yolov5.cc

CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/rknpu2/yolov5.cc > CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.i

CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/rknpu2/yolov5.cc -o CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.s

# Object files for target yolov5_image_demo
yolov5_image_demo_OBJECTS = \
"CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.o" \
"CMakeFiles/yolov5_image_demo.dir/postprocess.cc.o" \
"CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.o"

# External object files for target yolov5_image_demo
yolov5_image_demo_EXTERNAL_OBJECTS =

yolov5_image_demo: CMakeFiles/yolov5_image_demo.dir/yolov5_image_demo.cc.o
yolov5_image_demo: CMakeFiles/yolov5_image_demo.dir/postprocess.cc.o
yolov5_image_demo: CMakeFiles/yolov5_image_demo.dir/rknpu2/yolov5.cc.o
yolov5_image_demo: CMakeFiles/yolov5_image_demo.dir/build.make
yolov5_image_demo: utils.out/libimageutils.a
yolov5_image_demo: utils.out/libfileutils.a
yolov5_image_demo: utils.out/libimagedrawing.a
yolov5_image_demo: /home/xujg/yolo_rknn/3rdparty/rknpu2/Linux/aarch64/librknnrt.so
yolov5_image_demo: /home/xujg/yolo_rknn/3rdparty/jpeg_turbo/Linux/aarch64/libturbojpeg.a
yolov5_image_demo: /home/xujg/yolo_rknn/3rdparty/librga/Linux/aarch64/librga.a
yolov5_image_demo: CMakeFiles/yolov5_image_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xujg/yolo_rknn/yolov5_bytetrack/cpp/build/build_rk3588_linux/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable yolov5_image_demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/yolov5_image_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/yolov5_image_demo.dir/build: yolov5_image_demo

.PHONY : CMakeFiles/yolov5_image_demo.dir/build

CMakeFiles/yolov5_image_demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/yolov5_image_demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/yolov5_image_demo.dir/clean

CMakeFiles/yolov5_image_demo.dir/depend:
	cd /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/build/build_rk3588_linux && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xujg/yolo_rknn/yolov5_bytetrack/cpp /home/xujg/yolo_rknn/yolov5_bytetrack/cpp /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/build/build_rk3588_linux /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/build/build_rk3588_linux /home/xujg/yolo_rknn/yolov5_bytetrack/cpp/build/build_rk3588_linux/CMakeFiles/yolov5_image_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/yolov5_image_demo.dir/depend

