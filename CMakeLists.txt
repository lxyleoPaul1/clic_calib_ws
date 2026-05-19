cmake_minimum_required(VERSION 3.0.2)
project(clic_calib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## Phase 0: spline + utils headers only (no SLAM executables)
###################################
catkin_package(
  INCLUDE_DIRS include src
  LIBRARIES clic_calib_spline_lib
  DEPENDS EIGEN3
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(clic_calib_spline_lib
  src/clic_calib/spline/trajectory.cpp
)
target_link_libraries(clic_calib_spline_lib ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

###################################
## Phase 1+ (commented out for phase0-setup)
###################################
# find_package(yaml-cpp REQUIRED)
# find_package(Ceres REQUIRED)
#
# add_library(clic_calib_core
#   src/clic_calib/utils/lever_arm.cpp
#   src/clic_calib/estimator/calibration_estimator.cpp
#   src/clic_calib/estimator/observability_analyzer.cpp
# )
# target_link_libraries(clic_calib_core clic_calib_spline_lib ...)
#
# add_executable(calibrate_offline app/calibrate_offline.cpp)
# add_executable(analyze_observability app/analyze_observability.cpp)
#
# if(CATKIN_ENABLE_TESTING)
#   catkin_add_gtest(test_spline_recovery test/test_spline_recovery.cpp)
# endif()
