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
CMAKE_SOURCE_DIR = /home/b7/cvminecraft/image_difference

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/b7/cvminecraft/image_difference/build

# Include any dependencies generated for this target.
include CMakeFiles/test_perspective.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_perspective.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_perspective.dir/flags.make

CMakeFiles/test_perspective.dir/test_perspective.cpp.o: CMakeFiles/test_perspective.dir/flags.make
CMakeFiles/test_perspective.dir/test_perspective.cpp.o: ../test_perspective.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/b7/cvminecraft/image_difference/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_perspective.dir/test_perspective.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_perspective.dir/test_perspective.cpp.o -c /home/b7/cvminecraft/image_difference/test_perspective.cpp

CMakeFiles/test_perspective.dir/test_perspective.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_perspective.dir/test_perspective.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/b7/cvminecraft/image_difference/test_perspective.cpp > CMakeFiles/test_perspective.dir/test_perspective.cpp.i

CMakeFiles/test_perspective.dir/test_perspective.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_perspective.dir/test_perspective.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/b7/cvminecraft/image_difference/test_perspective.cpp -o CMakeFiles/test_perspective.dir/test_perspective.cpp.s

CMakeFiles/test_perspective.dir/test_perspective.cpp.o.requires:

.PHONY : CMakeFiles/test_perspective.dir/test_perspective.cpp.o.requires

CMakeFiles/test_perspective.dir/test_perspective.cpp.o.provides: CMakeFiles/test_perspective.dir/test_perspective.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_perspective.dir/build.make CMakeFiles/test_perspective.dir/test_perspective.cpp.o.provides.build
.PHONY : CMakeFiles/test_perspective.dir/test_perspective.cpp.o.provides

CMakeFiles/test_perspective.dir/test_perspective.cpp.o.provides.build: CMakeFiles/test_perspective.dir/test_perspective.cpp.o


# Object files for target test_perspective
test_perspective_OBJECTS = \
"CMakeFiles/test_perspective.dir/test_perspective.cpp.o"

# External object files for target test_perspective
test_perspective_EXTERNAL_OBJECTS =

test_perspective: CMakeFiles/test_perspective.dir/test_perspective.cpp.o
test_perspective: CMakeFiles/test_perspective.dir/build.make
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
test_perspective: /usr/lib/x86_64-linux-gnu/libcholmod.so
test_perspective: /usr/lib/x86_64-linux-gnu/libamd.so
test_perspective: /usr/lib/x86_64-linux-gnu/libcolamd.so
test_perspective: /usr/lib/x86_64-linux-gnu/libcamd.so
test_perspective: /usr/lib/x86_64-linux-gnu/libccolamd.so
test_perspective: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
test_perspective: /usr/local/lib/libceres.a
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
test_perspective: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
test_perspective: /usr/lib/x86_64-linux-gnu/libglog.so
test_perspective: /usr/lib/x86_64-linux-gnu/libspqr.so
test_perspective: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
test_perspective: /usr/lib/x86_64-linux-gnu/libtbb.so
test_perspective: /usr/lib/x86_64-linux-gnu/libcholmod.so
test_perspective: /usr/lib/x86_64-linux-gnu/libamd.so
test_perspective: /usr/lib/x86_64-linux-gnu/libcolamd.so
test_perspective: /usr/lib/x86_64-linux-gnu/libcamd.so
test_perspective: /usr/lib/x86_64-linux-gnu/libccolamd.so
test_perspective: /usr/lib/liblapack.so
test_perspective: /usr/lib/libblas.so
test_perspective: /usr/lib/x86_64-linux-gnu/librt.so
test_perspective: /usr/lib/x86_64-linux-gnu/libcxsparse.so
test_perspective: /usr/lib/liblapack.so
test_perspective: /usr/lib/libblas.so
test_perspective: /usr/lib/x86_64-linux-gnu/librt.so
test_perspective: /usr/lib/x86_64-linux-gnu/libcxsparse.so
test_perspective: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
test_perspective: CMakeFiles/test_perspective.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/b7/cvminecraft/image_difference/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_perspective"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_perspective.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_perspective.dir/build: test_perspective

.PHONY : CMakeFiles/test_perspective.dir/build

CMakeFiles/test_perspective.dir/requires: CMakeFiles/test_perspective.dir/test_perspective.cpp.o.requires

.PHONY : CMakeFiles/test_perspective.dir/requires

CMakeFiles/test_perspective.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_perspective.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_perspective.dir/clean

CMakeFiles/test_perspective.dir/depend:
	cd /home/b7/cvminecraft/image_difference/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/b7/cvminecraft/image_difference /home/b7/cvminecraft/image_difference /home/b7/cvminecraft/image_difference/build /home/b7/cvminecraft/image_difference/build /home/b7/cvminecraft/image_difference/build/CMakeFiles/test_perspective.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_perspective.dir/depend

