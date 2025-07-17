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
"utils.h" "utils.cpp" "tracker.h" "tracker.cpp")

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

add_test(NAME synthetic_1_5_nouwb COMMAND gtsam_test stereoi_circle2 synthetic_1_5 no_uwb 0 true)
add_test(NAME synthetic_1_5_uwb COMMAND gtsam_test stereoi_circle2 synthetic_1_5 uwb 0 true)

add_test(NAME synthetic_1_10_nouwb COMMAND gtsam_test stereoi_circle2 synthetic_1_10 no_uwb 0.2 true)
add_test(NAME synthetic_1_10_uwb COMMAND gtsam_test stereoi_circle2 synthetic_1_10 uwb 0.2 true)

add_test(NAME synthetic_1_20_nouwb COMMAND gtsam_test stereoi_circle2 synthetic_1_20 no_uwb 0.2 true)
add_test(NAME synthetic_1_20_uwb COMMAND gtsam_test stereoi_circle2 synthetic_1_20 uwb 0.2 true)


add_test(NAME synthetic_2_5_nouwb COMMAND gtsam_test stereoi_circle2 synthetic_2_5 no_uwb 0.2 true)
add_test(NAME synthetic_2_5_uwb COMMAND gtsam_test stereoi_circle2 synthetic_2_5 uwb 0.2 true)

add_test(NAME synthetic_2_10_nouwb COMMAND gtsam_test stereoi_circle2 synthetic_2_10 no_uwb 0.2 true)
add_test(NAME synthetic_2_10_uwb COMMAND gtsam_test stereoi_circle2 synthetic_2_10 uwb 0.2 true)

add_test(NAME synthetic_2_20_nouwb COMMAND gtsam_test stereoi_circle2 synthetic_2_20 no_uwb 0.2 true)
add_test(NAME synthetic_2_20_uwb COMMAND gtsam_test stereoi_circle2 synthetic_2_20 uwb 0.2 true)