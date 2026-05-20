/*
 * clic_calib — thin wrapper over libapriltag (Phase 4).
 */

#pragma once

#include <clic_calib/sensor_data/apriltag_observation.h>

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace clic_calib {

/** @brief Detect AprilTags in camera images; one @p AprilTagObservation per tag. */
class AprilTagDetectorWrapper {
 public:
  struct Options {
    std::string family = "tag36h11";
    double quad_decimate = 2.0;
    int nthreads = 2;
    int refine_edges = 1;
  };

  AprilTagDetectorWrapper() : AprilTagDetectorWrapper(Options{}) {}
  explicit AprilTagDetectorWrapper(const Options& options);
  ~AprilTagDetectorWrapper();

  AprilTagDetectorWrapper(const AprilTagDetectorWrapper&) = delete;
  AprilTagDetectorWrapper& operator=(const AprilTagDetectorWrapper&) = delete;

  /** @brief Detect all tags in @p image (BGR or grayscale). */
  bool Detect(const cv::Mat& image, double timestamp_s, int sensor_id,
              std::vector<AprilTagObservation>* observations) const;

  bool DetectFromFile(const std::string& image_path, double timestamp_s,
                      int sensor_id,
                      std::vector<AprilTagObservation>* observations) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace clic_calib
