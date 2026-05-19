cmake_minimum_required(VERSION 3.0.2)
project(clic_calib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## Libraries
###################################
catkin_package(
  INCLUDE_DIRS include src
  LIBRARIES clic_calib_spline_lib clic_calib_data_lib
  DEPENDS EIGEN3
)

include_directories(
  include
  src
  test
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_library(clic_calib_spline_lib
  src/clic_calib/spline/trajectory.cpp
)
target_link_libraries(clic_calib_spline_lib ${catkin_LIBRARIES})

add_library(clic_calib_data_lib
  src/clic_calib/utils/lever_arm.cpp
  src/clic_calib/io/rtk_reader.cpp
)
target_link_libraries(clic_calib_data_lib
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(TARGETS clic_calib_spline_lib clic_calib_data_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

###################################
## Tests
###################################
if(CATKIN_ENABLE_TESTING)
  set(FACTOR_TEST_DEPS clic_calib_spline_lib ${CERES_LIBRARIES})

  catkin_add_gtest(test_lever_arm test/test_lever_arm.cpp)
  target_link_libraries(test_lever_arm clic_calib_data_lib clic_calib_spline_lib
    ${YAML_CPP_LIBRARIES})

  catkin_add_gtest(test_rtk_reader test/test_rtk_reader.cpp)
  target_link_libraries(test_rtk_reader clic_calib_data_lib ${YAML_CPP_LIBRARIES})

  catkin_add_gtest(test_rtk_factor_jacobian test/test_rtk_factor_jacobian.cpp)
  target_link_libraries(test_rtk_factor_jacobian ${FACTOR_TEST_DEPS})

  catkin_add_gtest(test_sphere_factor_jacobian test/test_sphere_factor_jacobian.cpp)
  target_link_libraries(test_sphere_factor_jacobian ${FACTOR_TEST_DEPS})

  catkin_add_gtest(test_apriltag_factor_jacobian test/test_apriltag_factor_jacobian.cpp)
  target_link_libraries(test_apriltag_factor_jacobian ${FACTOR_TEST_DEPS})

  catkin_add_gtest(test_prior_smoothness_jacobian
    test/test_prior_smoothness_jacobian.cpp)
  target_link_libraries(test_prior_smoothness_jacobian ${FACTOR_TEST_DEPS})
endif()
