cmake_minimum_required(VERSION 3.0.2)
project(clic_calib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  sensor_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation sample_consensus search)
find_package(OpenCV REQUIRED)

find_library(APRILTAG_LIBRARY apriltag)
find_library(APRILTAG_UTILS_LIBRARY apriltag-utils)
find_path(APRILTAG_INCLUDE_DIR apriltag/apriltag.h)
if(NOT APRILTAG_LIBRARY OR NOT APRILTAG_INCLUDE_DIR)
  message(FATAL_ERROR "libapriltag not found (install libapriltag-dev)")
endif()

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## Libraries
###################################
catkin_package(
  INCLUDE_DIRS include src
  LIBRARIES clic_calib_spline_lib clic_calib_data_lib clic_calib_target_lib clic_calib_estimator_lib
  DEPENDS EIGEN3 PCL OpenCV
)

include_directories(
  include
  src
  test
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${APRILTAG_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS})

add_library(clic_calib_spline_lib
  src/clic_calib/spline/trajectory.cpp
)
target_link_libraries(clic_calib_spline_lib ${catkin_LIBRARIES})

add_library(clic_calib_data_lib
  src/clic_calib/utils/lever_arm.cpp
  src/clic_calib/utils/noise_model.cpp
  src/clic_calib/utils/temporal_correlation.cpp
  src/clic_calib/io/rtk_reader.cpp
  src/clic_calib/io/attitude_reader.cpp
  src/clic_calib/io/observation_archive.cpp
  src/clic_calib/io/calibration_result.cpp
  src/clic_calib/io/sensor_rig_config.cpp
  src/clic_calib/config/body_model_config.cpp
  src/clic_calib/config/body_cluster_detection_config.cpp
)
target_link_libraries(clic_calib_data_lib
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_library(clic_calib_target_lib
  src/clic_calib/target/sphere_extractor.cpp
  src/clic_calib/target/apriltag_detector_wrapper.cpp
  src/clic_calib/target/body_cluster_extractor.cpp
)
target_link_libraries(clic_calib_target_lib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${APRILTAG_LIBRARY}
  ${APRILTAG_UTILS_LIBRARY}
)

add_library(clic_calib_estimator_lib
  src/clic_calib/estimator/calibration_estimator.cpp
  src/clic_calib/estimator/observability_analyzer.cpp
  src/clic_calib/estimator/stage1_trajectory_fitter.cpp
  src/clic_calib/estimator/trajectory_stage.cpp
  src/clic_calib/estimator/extrinsic_initializer.cpp
  src/clic_calib/estimator/extrinsic_refiner.cpp
  src/clic_calib/estimator/stage2_extrinsic_fim.cpp
  src/clic_calib/estimator/two_stage_pipeline.cpp
  src/clic_calib/estimator/uq_decomposition.cpp
  src/clic_calib/estimator/attitude_stream_config.cpp
  src/clic_calib/estimator/real_data_session.cpp
  src/clic_calib/estimator/observed_mean_gate.cpp
  src/clic_calib/estimator/body_gated_calibration.cpp
  src/clic_calib/estimator/multi_lidar_body_calibration.cpp
  src/clic_calib/estimator/relative_extrinsic.cpp
  src/clic_calib/target/body_centroid_analysis.cpp
  src/clic_calib/target/drone_model_registration.cpp
)
target_link_libraries(clic_calib_estimator_lib
  clic_calib_spline_lib
  clic_calib_data_lib
  ${CERES_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(analyze_observability app/analyze_observability.cpp)
target_link_libraries(analyze_observability
  clic_calib_estimator_lib
  clic_calib_data_lib
  ${CERES_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(calibrate_offline app/calibrate_offline.cpp)
target_link_libraries(calibrate_offline
  clic_calib_estimator_lib
  clic_calib_data_lib
  ${CERES_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(preprocess_rosbag app/preprocess_rosbag.cpp)
target_link_libraries(preprocess_rosbag
  clic_calib_target_lib
  clic_calib_data_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(TARGETS clic_calib_spline_lib clic_calib_data_lib clic_calib_target_lib
  clic_calib_estimator_lib analyze_observability calibrate_offline preprocess_rosbag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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

  catkin_add_gtest(test_full_pipeline_synthetic
    test/test_full_pipeline_synthetic.cpp)
  target_link_libraries(test_full_pipeline_synthetic
    clic_calib_estimator_lib clic_calib_data_lib clic_calib_spline_lib
    ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES})

  catkin_add_gtest(test_pipeline_noise_sweep
    test/test_pipeline_noise_sweep.cpp)
  target_link_libraries(test_pipeline_noise_sweep
    clic_calib_estimator_lib clic_calib_data_lib clic_calib_spline_lib
    ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES})

  catkin_add_gtest(test_observability_synthetic
    test/test_observability_synthetic.cpp)
  target_link_libraries(test_observability_synthetic
    clic_calib_estimator_lib clic_calib_data_lib clic_calib_spline_lib
    ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES})

  catkin_add_gtest(test_patent_z_accuracy test/test_patent_z_accuracy.cpp)
  target_link_libraries(test_patent_z_accuracy
    clic_calib_estimator_lib clic_calib_data_lib clic_calib_spline_lib
    ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES})

  catkin_add_gtest(test_prior_ablation test/test_prior_ablation.cpp)
  target_link_libraries(test_prior_ablation
    clic_calib_estimator_lib clic_calib_data_lib clic_calib_spline_lib
    ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES})

  catkin_add_gtest(test_uq_decomposition test/test_uq_decomposition.cpp)
  target_link_libraries(test_uq_decomposition
    clic_calib_estimator_lib clic_calib_data_lib clic_calib_spline_lib
    ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES} ${OpenCV_LIBS})

  catkin_add_gtest(test_real_data_interface test/test_real_data_interface.cpp)
  target_link_libraries(test_real_data_interface
    clic_calib_estimator_lib clic_calib_data_lib clic_calib_spline_lib
    ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES})

  catkin_add_gtest(test_sphere_extractor test/test_sphere_extractor.cpp)
  target_link_libraries(test_sphere_extractor clic_calib_target_lib ${PCL_LIBRARIES})

  catkin_add_gtest(test_apriltag_wrapper test/test_apriltag_wrapper.cpp)
  target_link_libraries(test_apriltag_wrapper clic_calib_target_lib ${OpenCV_LIBS})

  catkin_add_gtest(test_clicob_v2_roundtrip test/test_clicob_v2_roundtrip.cpp)
  target_link_libraries(test_clicob_v2_roundtrip clic_calib_data_lib
    ${YAML_CPP_LIBRARIES})

  catkin_add_gtest(test_clicob_v1_backward_compat
    test/test_clicob_v1_backward_compat.cpp)
  target_link_libraries(test_clicob_v1_backward_compat clic_calib_data_lib)

  catkin_add_gtest(test_body_centroid_factor_jacobian
    test/test_body_centroid_factor_jacobian.cpp)
  target_link_libraries(test_body_centroid_factor_jacobian ${FACTOR_TEST_DEPS})

  catkin_add_gtest(test_fixed_traj_body_centroid_factor_jacobian
    test/test_fixed_traj_body_centroid_factor_jacobian.cpp)
  target_link_libraries(test_fixed_traj_body_centroid_factor_jacobian
    clic_calib_spline_lib ${CERES_LIBRARIES})

  catkin_add_gtest(test_body_cluster_extractor test/test_body_cluster_extractor.cpp)
  target_link_libraries(test_body_cluster_extractor clic_calib_target_lib
    clic_calib_data_lib ${YAML_CPP_LIBRARIES})
endif()
