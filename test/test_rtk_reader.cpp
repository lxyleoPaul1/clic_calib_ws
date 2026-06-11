#include <clic_calib/io/rtk_reader.h>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include <filesystem>
#include <string>

namespace {

std::string PackageTestDataPath(const std::string& name) {
  const std::filesystem::path from_source =
      std::filesystem::path(__FILE__).parent_path() / "data" / name;
  if (std::filesystem::exists(from_source)) {
    return from_source.string();
  }
  return (std::filesystem::path("test/data") / name).string();
}

TEST(NMEAReader, Parses100LineSample) {
  const std::string path = PackageTestDataPath("sample_nmea_100.txt");
  ASSERT_TRUE(std::filesystem::exists(path)) << path;

  const clic_calib::NMEAReader reader;
  const auto measurements = reader.read(path);

  EXPECT_EQ(measurements.size(), 100u);
  for (size_t i = 1; i < measurements.size(); ++i) {
    EXPECT_LE(measurements[i - 1].t_world_, measurements[i].t_world_);
  }

  // Default NMEAReader uses first fix as local ENU origin → first position is (0,0,0).
  EXPECT_NEAR(measurements.front().p_A_W_observed_.norm(), 0.0, 1e-6);
  EXPECT_GT(measurements.back().p_A_W_observed_.norm(), 1.0);
  EXPECT_EQ(measurements.front().fix_status_,
            clic_calib::RTKMeasurement::FixStatus::FIXED);
}

TEST(CSVReader, ParsesSample) {
  const std::string path = PackageTestDataPath("sample_rtk.csv");
  ASSERT_TRUE(std::filesystem::exists(path)) << path;

  const clic_calib::CSVReader reader;
  const auto measurements = reader.read(path);
  ASSERT_GE(measurements.size(), 3u);
  EXPECT_EQ(measurements[0].fix_status_,
            clic_calib::RTKMeasurement::FixStatus::FIXED);
  EXPECT_GT(measurements[0].covariance_(0, 0), 0.0);
}

TEST(DJIDatLogReader, ParsesSample) {
  const std::string path = PackageTestDataPath("sample_m3e.dat");
  ASSERT_TRUE(std::filesystem::exists(path)) << path;

  const clic_calib::DJIDatLogReader reader;
  const auto measurements = reader.read(path);
  ASSERT_EQ(measurements.size(), 5u);
  EXPECT_GT(measurements[0].cn0_dbHz_, 40.0);
}

}  // namespace

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
