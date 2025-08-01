# CMakeList.txt : CMake project for gtsam_test, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("gtsam_test")

# include_directories("/usr/include/gtsam/")
# include_directories("/home/antond2/gtsam")
include_directories("/usr/local/include/gtsam")
include_directories("/usr/include/eigen3/")
include_directories("/usr/local/include/matplot")
include_directories("/home/antond2/json/single_include")

# Note: Matplot++ is different from,.and more up to date than Matplotlib-cpp.

find_package(GTSAMCMakeTools)
include(GtsamBuildTypes) # Load build type flags and default to Debug mode
include(GtsamTesting)    # Easy functions for creating unit tests and scripts

# Ok apparently most of my .so files are in /lib/x86_64
# set(GTSAM_LIB_PATH "/lib/x86_64-linux-gnu/libgtsam.so")
# set(GTSAM_LIB_PATH "/home/antond2/gtsam/libtsam.so") # I have another one of these in /usr/local/lib
set(GTSAM_LIB_PATH "/usr/local/lib/libgtsam.so")
set(TBB_LIB_PATH "/lib/x86_64-linux-gnu/libtbb.so")
set(MATPLOTPLUSPLUS_LIB_PATH "/usr/local/lib/libmatplot.a")

# Add source to this project's executable.
add_executable (gtsam_test "gtsam_test.cpp" "gtsam_test.h" "data_tools.h" "data_tools.cpp" 
"utils.h" "utils.cpp" "tracker.h" "tracker.cpp" "1euro.h")

# Need to manually link with gtsam, apparently find_package doesn't work'
target_link_libraries(gtsam_test 
"${GTSAM_LIB_PATH}" "${TBB_LIB_PATH}" "${MATPLOTPLUSPLUS_LIB_PATH}" 
"/usr/lib/x86_64-linux-gnu/libjpeg.so" "/usr/lib/x86_64-linux-gnu/libtiff.so" 
"/usr/lib/x86_64-linux-gnu/libz.so" "/usr/lib/x86_64-linux-gnu/libpng.so" 
"/usr/local/lib/Matplot++/libnodesoup.a" "/usr/lib/x86_64-linux-gnu/libboost_iostreams.so")

add_compile_options(-w)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET gtsam_test PROPERTY CXX_STANDARD 20)
endif()

# <trial_name> <synthetic_trial_name or 'none'> <'uwb' or 'no_uwb'> <dump (true|false)>"
enable_testing()

# add_test(NAME synthetic_20_60_nouwb COMMAND gtsam_test stereoi_circle2 synthetic_20_60 no_uwb 0.2 true)
# add_test(NAME synthetic_20_60_uwb COMMAND gtsam_test stereoi_circle2 synthetic_20_60 uwb 0.2 true)

add_test(NAME uwb_calibration_rot/real_nouwb COMMAND gtsam_test uwb_calibration__rot synthetic_1_0 no_uwb 0.0 true)
add_test(NAME uwb_calibration_rot/real_uwb COMMAND gtsam_test uwb_calibration__rot synthetic_1_0 uwb 0.0 true)

add_test(NAME uwb_calibration_trans/real_nouwb COMMAND gtsam_test uwb_calibration_trans synthetic_1_0 no_uwb 0.0 true)
add_test(NAME uwb_calibration_trans/real_uwb COMMAND gtsam_test uwb_calibration_trans synthetic_1_0 uwb 0.0 true)

# Just going to pass synthetic noise model as 0 since I won't be generating synthetic ranges
add_test(NAME pilot4_whiteboard2/real_nouwb COMMAND gtsam_test pilot4_whiteboard2 synthetic_1_0 no_uwb 0.0 true)
add_test(NAME pilot4_whiteboard2/real_uwb COMMAND gtsam_test pilot4_whiteboard2 synthetic_1_0 uwb 0.0 true)

add_test(NAME pilot4_chair2/real_nouwb COMMAND gtsam_test pilot4_chair2 synthetic_1_0 no_uwb 0.0 true)
add_test(NAME pilot4_chair2/real_uwb COMMAND gtsam_test pilot4_chair2 synthetic_1_0 uwb 0.0 true)

add_test(NAME pilot4_loopy/real_nouwb COMMAND gtsam_test pilot4_loopy synthetic_1_0 no_uwb 0.0 true)
add_test(NAME pilot4_loopy/real_uwb COMMAND gtsam_test pilot4_loopy synthetic_1_0 uwb 0.0 true)

add_test(NAME pilot4_room/real_nouwb COMMAND gtsam_test pilot4_room synthetic_1_0 no_uwb 0.0 true)
add_test(NAME pilot4_room/real_uwb COMMAND gtsam_test pilot4_room synthetic_1_0 uwb 0.0 true)

# add_test(NAME loopy_synthetic_1_5_nouwb COMMAND gtsam_test pilot3_stereoi_loopy synthetic_1_5 no_uwb 0.1 true)
# add_test(NAME loopy_synthetic_1_5_uwb COMMAND gtsam_test pilot3_stereoi_loopy synthetic_1_5 uwb 0.1 true)

add_test(NAME stereoi_circle2/synthetic_1_5_nouwb COMMAND gtsam_test stereoi_circle2 synthetic_1_5 no_uwb 0.2 true)
add_test(NAME stereoi_circle2/synthetic_1_5_uwb COMMAND gtsam_test stereoi_circle2 synthetic_1_5 uwb 0.2 true)

add_test(NAME stereoi_circle2/synthetic_1_10_nouwb COMMAND gtsam_test stereoi_circle2 synthetic_1_10 no_uwb 0.2 true)
add_test(NAME stereoi_circle2/synthetic_1_10_uwb COMMAND gtsam_test stereoi_circle2 synthetic_1_10 uwb 0.2 true)
