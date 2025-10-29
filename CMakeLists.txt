cmake_minimum_required (VERSION 3.8)

project ("gtsam_test")


# You should only need to customize these 4 filepaths, probably just the json, single_include
include_directories("/usr/local/include/gtsam")
include_directories("/usr/include/eigen3/")
# include_directories("/home/antond2/json/single_include")
set(GTSAM_LIB_PATH "/usr/local/lib/libgtsam.so")

find_package(GTSAMCMakeTools)
include(GtsamBuildTypes) # Load build type flags and default to Debug mode
include(GtsamTesting)    # Easy functions for creating unit tests and scripts

set(TBB_LIB_PATH "/lib/x86_64-linux-gnu/libtbb.so")

# Add source to this project's executable.
add_executable (gtsam_test "gtsam_test.cpp" "gtsam_test.h" "data_tools.h" "data_tools.cpp" 
"tracker.h" "tracker.cpp" "1euro.h")

# Need to manually link with gtsam, apparently find_package doesn't work'
target_link_libraries(gtsam_test 
"${GTSAM_LIB_PATH}" "${TBB_LIB_PATH}" "${MATPLOTPLUSPLUS_LIB_PATH}" 
"/usr/lib/x86_64-linux-gnu/libjpeg.so" "/usr/lib/x86_64-linux-gnu/libtiff.so" 
"/usr/lib/x86_64-linux-gnu/libz.so" "/usr/lib/x86_64-linux-gnu/libboost_iostreams.so")

add_compile_options(-w)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET gtsam_test PROPERTY CXX_STANDARD 20)
endif()

# <trial_name> <synthetic_trial_name or 'none'> <'uwb' or 'no_uwb'> <dump (true|false)>"
enable_testing()

add_test(NAME irl5_imu_bias_straight3 COMMAND gtsam_test irl5_imu_bias_straight3 none no_uwb 0.0 true)
add_test(NAME irl5_imu_bias_worn COMMAND gtsam_test irl5_imu_bias_worn none no_uwb 0.0 true)

# add_test(NAME synthetic_20_60_nouwb COMMAND gtsam_test stereoi_circle2 synthetic_20_60 no_uwb 0.2 true)
# add_test(NAME synthetic_20_60_uwb COMMAND gtsam_test stereoi_circle2 synthetic_20_60 uwb 0.2 true)

add_test(NAME uwb_calibration_rot/real_nouwb COMMAND gtsam_test uwb_calibration__rot synthetic_1_0 no_uwb 0.0 true)
add_test(NAME uwb_calibration_rot/real_uwb COMMAND gtsam_test uwb_calibration__rot synthetic_1_0 uwb 0.0 true)

add_test(NAME uwb_calibration_trans/real_nouwb COMMAND gtsam_test uwb_calibration_trans synthetic_1_0 no_uwb 0.0 true)
add_test(NAME uwb_calibration_trans/real_uwb COMMAND gtsam_test uwb_calibration_trans synthetic_1_0 uwb 0.0 true)

# Just going to pass synthetic noise model as 0 since I won't be generating synthetic ranges
# add_test(NAME pilot4_whiteboard2/real_nouwb COMMAND gtsam_test pilot4_whiteboard2 synthetic_1_0 no_uwb 0.0 true)
# add_test(NAME pilot4_whiteboard2/real_uwb COMMAND gtsam_test pilot4_whiteboard2 synthetic_1_0 uwb 0.0 true)

# add_test(NAME pilot4_chair2/real_nouwb COMMAND gtsam_test pilot4_chair2 synthetic_1_0 no_uwb 0.0 true)
# add_test(NAME pilot4_chair2/real_uwb COMMAND gtsam_test pilot4_chair2 synthetic_1_0 uwb 0.0 true)

add_test(NAME pilot4_loopy/real_nouwb COMMAND gtsam_test pilot4_loopy synthetic_1_0 no_uwb 0.0 true)
add_test(NAME pilot4_loopy/real_uwb COMMAND gtsam_test pilot4_loopy synthetic_1_0 uwb 0.0 true)

add_test(NAME loopy_test/real_nouwb COMMAND gtsam_test loopy_test synthetic_1_0 no_uwb 0.0 true)
add_test(NAME loopy_test/real_uwb COMMAND gtsam_test loopy_test synthetic_1_0 uwb 0.0 true)

# add_test(NAME pilot4_room/real_nouwb COMMAND gtsam_test pilot4_room synthetic_1_0 no_uwb 0.0 true)
# add_test(NAME pilot4_room/real_uwb COMMAND gtsam_test pilot4_room synthetic_1_0 uwb 0.0 true)

add_test(NAME stereoi_circle2/synthetic_1_5_nouwb COMMAND gtsam_test stereoi_circle2 synthetic_1_5 no_uwb 0.02 true)
add_test(NAME stereoi_circle2/synthetic_1_5_uwb COMMAND gtsam_test stereoi_circle2 synthetic_1_5 uwb 0.02 true)

