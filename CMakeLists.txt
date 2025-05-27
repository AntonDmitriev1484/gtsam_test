# CMakeList.txt : CMake project for gtsam_test, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("gtsam_test")

# Matplot++ https://github.com/alandefreitas/matplotplusplus


include_directories("/usr/include/gtsam/")
include_directories("/usr/include/eigen3/")
include_directories("/usr/local/include/matplot")
include_directories("/usr/include/json/single_include") #NHolmann JSON library for reading in data

#find_package(Eigen3)
#find_package(nlohmann_json)
#find_package(GT)

# Note: Matplot++ is different from,.and more up to date than Matplotlib-cpp.

find_package(GTSAMCMakeTools)
include(GtsamBuildTypes) # Load build type flags and default to Debug mode
include(GtsamTesting)    # Easy functions for creating unit tests and scripts

set(GTSAM_LIB_PATH "/lib/x86_64-linux-gnu/libgtsam.so")
set(TBB_LIB_PATH "/lib/x86_64-linux-gnu/libtbb.so")
set(MATPLOTPLUSPLUS_LIB_PATH "/usr/local/lib/libmatplot.a")

# Add source to this project's executable.
add_executable (gtsam_test "gtsam_test.cpp" "gtsam_test.h" "data_tools.h" "data_tools.cpp" "utils.h" "utils.cpp")

# Needed to manually link with gtsam, apparently find_package doesn't work'
target_link_libraries(gtsam_test 
"${GTSAM_LIB_PATH}" "${TBB_LIB_PATH}"
"${MATPLOTPLUSPLUS_LIB_PATH}" "/usr/lib/x86_64-linux-gnu/libjpeg.so" "/usr/lib/x86_64-linux-gnu/libtiff.so" "/usr/lib/x86_64-linux-gnu/libz.so" "/usr/lib/x86_64-linux-gnu/libpng.so" "/usr/local/lib/Matplot++/libnodesoup.a")


if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET gtsam_test PROPERTY CXX_STANDARD 20)
endif()
