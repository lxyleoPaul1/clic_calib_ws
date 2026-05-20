#include <clic_calib/target/sphere_extractor.h>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <random>

namespace {

pcl::PointCloud<pcl::PointXYZI> MakeSyntheticCloud(const Eigen::Vector3d& center,
                                                   double radius_m) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  std::mt19937 rng(99);
  std::normal_distribution<double> noise(0.0, 0.005);

  for (int i = 0; i < 100; ++i) {
    const double theta = 2.0 * M_PI * i / 100.0;
    const double phi = M_PI * (i % 10) / 10.0;
    Eigen::Vector3d dir(std::sin(phi) * std::cos(theta),
                        std::sin(phi) * std::sin(theta), std::cos(phi));
    const Eigen::Vector3d p = center + radius_m * dir;
    pcl::PointXYZI pt;
    pt.x = static_cast<float>(p.x() + noise(rng));
    pt.y = static_cast<float>(p.y() + noise(rng));
    pt.z = static_cast<float>(p.z() + noise(rng));
    pt.intensity = 220.0f;
    cloud.push_back(pt);
  }

  std::uniform_real_distribution<double> bg(-5.0, 5.0);
  for (int i = 0; i < 1000; ++i) {
    pcl::PointXYZI pt;
    pt.x = static_cast<float>(bg(rng));
    pt.y = static_cast<float>(bg(rng));
    pt.z = static_cast<float>(bg(rng));
    pt.intensity = 50.0f;
    cloud.push_back(pt);
  }
  return cloud;
}

}  // namespace

TEST(SphereExtractor, RecoversSphereCenterWithin2cm) {
  const Eigen::Vector3d true_center(1.5, -0.5, 3.0);
  const double radius = 0.10;
  const auto cloud = MakeSyntheticCloud(true_center, radius);

  clic_calib::SphereExtractor::Options opts;
  opts.intensity_min = 100.0;
  opts.min_cluster_size = 20;
  clic_calib::SphereExtractor extractor(opts);

  clic_calib::LiDARTargetObservation obs;
  Eigen::Vector3d fitted_center;
  ASSERT_TRUE(extractor.Extract(cloud, 1.0, 0, &true_center, &obs, &fitted_center));
  EXPECT_GE(obs.points_L_.size(), 20u);
  EXPECT_LT((fitted_center - true_center).norm(), 0.02);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
