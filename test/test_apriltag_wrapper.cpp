#include <clic_calib/target/apriltag_detector_wrapper.h>

#include <gtest/gtest.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

TEST(AprilTagDetectorWrapper, DetectsSyntheticTag36h11) {
  cv::Mat canvas(480, 640, CV_8UC1, cv::Scalar(255));
  cv::Ptr<cv::aruco::Dictionary> dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  cv::Mat marker;
  cv::aruco::drawMarker(dict, 0, 200, marker, 1);
  marker.copyTo(canvas(cv::Rect(220, 140, 200, 200)));

  clic_calib::AprilTagDetectorWrapper detector;
  std::vector<clic_calib::AprilTagObservation> observations;
  ASSERT_TRUE(detector.Detect(canvas, 1.0, 0, &observations));
  ASSERT_FALSE(observations.empty());
  EXPECT_EQ(observations.front().tag_id_, 0);
  for (const auto& c : observations.front().corners_pixel_) {
    EXPECT_GT(c.x(), 0.0);
    EXPECT_GT(c.y(), 0.0);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
