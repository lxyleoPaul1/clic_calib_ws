#include <clic_calib/target/apriltag_detector_wrapper.h>

#include <apriltag/apriltag.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h11.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cstring>
#include <stdexcept>
#include <string>

namespace clic_calib {
namespace {

apriltag_family_t* CreateFamily(const std::string& name) {
  if (name == "tag36h11") {
    return tag36h11_create();
  }
  if (name == "tag25h9") {
    return tag25h9_create();
  }
  if (name == "tag16h5") {
    return tag16h5_create();
  }
  throw std::runtime_error("Unsupported apriltag family: " + name);
}

void DestroyFamily(const std::string& name, apriltag_family_t* family) {
  if (!family) {
    return;
  }
  if (name == "tag36h11") {
    tag36h11_destroy(family);
  } else if (name == "tag25h9") {
    tag25h9_destroy(family);
  } else if (name == "tag16h5") {
    tag16h5_destroy(family);
  }
}

}  // namespace

struct AprilTagDetectorWrapper::Impl {
  Options options;
  apriltag_family_t* family = nullptr;
  apriltag_detector_t* detector = nullptr;

  explicit Impl(const Options& opts) : options(opts) {
    family = CreateFamily(options.family);
    detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, family);
    detector->quad_decimate = static_cast<float>(options.quad_decimate);
    detector->nthreads = options.nthreads;
    detector->refine_edges = options.refine_edges;
  }

  ~Impl() {
    if (detector) {
      apriltag_detector_destroy(detector);
    }
    DestroyFamily(options.family, family);
  }
};

AprilTagDetectorWrapper::AprilTagDetectorWrapper(const Options& options)
    : impl_(std::make_unique<Impl>(options)) {}

AprilTagDetectorWrapper::~AprilTagDetectorWrapper() = default;

bool AprilTagDetectorWrapper::Detect(const cv::Mat& image, double timestamp_s,
                                     int sensor_id,
                                     std::vector<AprilTagObservation>* observations) const {
  if (!observations || image.empty()) {
    return false;
  }
  observations->clear();

  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  } else {
    return false;
  }

  if (!gray.isContinuous()) {
    gray = gray.clone();
  }

  image_u8_t* im = image_u8_create(static_cast<unsigned>(gray.cols),
                                   static_cast<unsigned>(gray.rows));
  for (int row = 0; row < gray.rows; ++row) {
    std::memcpy(im->buf + row * im->stride, gray.ptr(row),
                static_cast<size_t>(gray.cols));
  }

  zarray_t* detections = apriltag_detector_detect(impl_->detector, im);
  image_u8_destroy(im);

  for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t* det = nullptr;
    zarray_get(detections, i, &det);
    if (!det) {
      continue;
    }

    AprilTagObservation obs;
    obs.t_sensor_ = timestamp_s;
    obs.sensor_id_ = sensor_id;
    obs.tag_id_ = static_cast<int>(det->id);
    obs.detection_confidence_ =
        det->decision_margin > 0.0 ? det->decision_margin : 1.0;

    for (int c = 0; c < 4; ++c) {
      obs.corners_pixel_[c] =
          Eigen::Vector2d(det->p[c][0], det->p[c][1]);
    }
    observations->push_back(obs);
  }

  apriltag_detections_destroy(detections);
  return !observations->empty();
}

bool AprilTagDetectorWrapper::DetectFromFile(const std::string& image_path,
                                             double timestamp_s, int sensor_id,
                                             std::vector<AprilTagObservation>* observations) const {
  const cv::Mat image = cv::imread(image_path, cv::IMREAD_UNCHANGED);
  if (image.empty()) {
    return false;
  }
  return Detect(image, timestamp_s, sensor_id, observations);
}

}  // namespace clic_calib
