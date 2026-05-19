/*
 * clic_calib — offline calibration entry (Phase 0 skeleton).
 */

#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/lever_arm.h>

#include <ros/ros.h>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibrate_offline");
  ros::NodeHandle nh("~");

  std::string config_dir;
  nh.param<std::string>("config_dir", config_dir, "");

  clic_calib::CalibrationEstimatorOptions opt;
  nh.param("knot_interval_s", opt.knot_interval_s, 0.05);
  nh.param("max_iterations", opt.max_iterations, 50);
  nh.param("verbose", opt.verbose, false);

  clic_calib::CalibrationEstimator estimator(opt);
  auto trajectory =
      std::make_shared<clic_calib::BodyTrajectory>(opt.knot_interval_s, 0.0);
  estimator.SetBodyTrajectory(trajectory);

  if (!config_dir.empty()) {
    try {
      const auto levers = clic_calib::LeverArmConfig::from_yaml(
          config_dir + "/lever_arms.yaml");
      std::cout << "Loaded L_B_to_A: " << levers.L_B_to_A.transpose()
                << "\n";
    } catch (const std::exception& e) {
      std::cerr << "Failed to load lever_arms.yaml: " << e.what() << "\n";
      return 1;
    }
  }

  std::cout << "clic_calib Phase 0: skeleton only — no factors wired yet.\n";
  estimator.Solve();
  return 0;
}
