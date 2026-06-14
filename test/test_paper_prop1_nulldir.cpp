#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

namespace {

using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;

constexpr int kNumFrames = 41;
constexpr int kNumDirections = 12;

Mat3 RotX(double a) {
  const double c = std::cos(a);
  const double s = std::sin(a);
  Mat3 R;
  R << 1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c;
  return R;
}

Mat3 RotY(double a) {
  const double c = std::cos(a);
  const double s = std::sin(a);
  Mat3 R;
  R << c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c;
  return R;
}

Mat3 RotZ(double a) {
  const double c = std::cos(a);
  const double s = std::sin(a);
  Mat3 R;
  R << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  return R;
}

double DegToRad(double deg) { return deg * M_PI / 180.0; }

std::vector<Mat3> MakeAttitudeProfile(double span_deg) {
  std::vector<Mat3> out;
  out.reserve(kNumFrames);
  const double span = DegToRad(span_deg);
  for (int i = 0; i < kNumFrames; ++i) {
    const double u = -1.0 + 2.0 * static_cast<double>(i) /
                                static_cast<double>(kNumFrames - 1);
    const double yaw = span * u;
    const double pitch = 0.55 * span * std::sin(M_PI * u);
    const double roll = 0.30 * span * std::sin(2.0 * M_PI * u);
    out.push_back(RotZ(yaw) * RotY(pitch) * RotX(roll));
  }
  return out;
}

Mat6 BuildFisher(const std::vector<Mat3>& R_wb, const Mat3& R_lw) {
  Mat6 F = Mat6::Zero();
  for (const Mat3& R : R_wb) {
    Eigen::Matrix<double, 3, 6> J;
    J.block<3, 3>(0, 0) = R_lw * R;
    J.block<3, 3>(0, 3) = Mat3::Identity();
    F += J.transpose() * J;
  }
  return F / static_cast<double>(R_wb.size());
}

Vec6 NullDirection(const Vec3& eta, const Mat3& R_lw, const Mat3& R_bar) {
  Vec6 d;
  d.head<3>() = eta;
  d.tail<3>() = -R_lw * R_bar * eta;
  return d.normalized();
}

double QuadForm(const Mat6& F, const Vec6& d) {
  return d.transpose() * F * d;
}

double MinEigenvalue(const Mat6& F) {
  Eigen::SelfAdjointEigenSolver<Mat6> es(F);
  return es.eigenvalues()(0);
}

double RotationDispersionDeg(const std::vector<Mat3>& R_wb,
                             const Mat3& R_bar) {
  double sq = 0.0;
  for (const Mat3& R : R_wb) {
    const Eigen::AngleAxisd aa(R_bar.transpose() * R);
    sq += aa.angle() * aa.angle();
  }
  return std::sqrt(sq / static_cast<double>(R_wb.size())) * 180.0 / M_PI;
}

std::vector<Vec3> TestDirections() {
  std::vector<Vec3> dirs;
  dirs.push_back(Vec3::UnitX());
  dirs.push_back(Vec3::UnitY());
  dirs.push_back(Vec3::UnitZ());
  std::mt19937 rng(7);
  std::normal_distribution<double> n(0.0, 1.0);
  while (static_cast<int>(dirs.size()) < kNumDirections) {
    Vec3 v(n(rng), n(rng), n(rng));
    if (v.norm() > 1e-9) {
      dirs.push_back(v.normalized());
    }
  }
  return dirs;
}

struct Row {
  double span_deg = 0.0;
  double rot_disp_deg = 0.0;
  double q_mean = 0.0;
  double q_max = 0.0;
  double min_eig = 0.0;
};

std::filesystem::path DataPath() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "paper/figures/data/prop1_nulldir.csv";
}

}  // namespace

TEST(PaperProp1, NullDirectionFisherQuadratic) {
  const Mat3 R_lw = RotZ(DegToRad(17.0)) * RotY(DegToRad(-8.0)) *
                    RotX(DegToRad(5.0));
  const Mat3 R_bar = Mat3::Identity();
  const std::vector<Vec3> dirs = TestDirections();
  const std::vector<double> spans = {0.0, 2.0, 5.0, 10.0, 20.0, 35.0};

  std::vector<Row> rows;
  for (double span : spans) {
    const std::vector<Mat3> R_wb = MakeAttitudeProfile(span);
    const Mat6 F = BuildFisher(R_wb, R_lw);
    std::vector<double> q_vals;
    double q_max = 0.0;
    for (const Vec3& eta : dirs) {
      const Vec6 d = NullDirection(eta, R_lw, R_bar);
      const double q = QuadForm(F, d);
      q_vals.push_back(q);
      q_max = std::max(q_max, q);
    }
    const double q_mean =
        std::accumulate(q_vals.begin(), q_vals.end(), 0.0) /
        static_cast<double>(q_vals.size());
    rows.push_back(
        Row{span, RotationDispersionDeg(R_wb, R_bar), q_mean, q_max,
            MinEigenvalue(F)});
  }

  const auto path = DataPath();
  std::filesystem::create_directories(path.parent_path());
  std::ofstream csv(path);
  csv << "span_deg,rot_dispersion_deg,quad_mean,quad_max,min_eigenvalue\n";
  csv << std::scientific << std::setprecision(9);
  for (const Row& r : rows) {
    csv << r.span_deg << "," << r.rot_disp_deg << "," << r.q_mean << ","
        << r.q_max << "," << r.min_eig << "\n";
    std::cout << std::scientific << std::setprecision(6)
              << "span=" << r.span_deg << " rot_disp=" << r.rot_disp_deg
              << " q_mean=" << r.q_mean << " q_max=" << r.q_max
              << " min_eig=" << r.min_eig << "\n";
  }
  std::cout << "csv: " << path << "\n";

  ASSERT_EQ(rows.size(), spans.size());
  EXPECT_LT(std::abs(rows.front().q_mean), 1e-14);
  EXPECT_LT(std::abs(rows.front().q_max), 1e-14);
  EXPECT_LT(std::abs(rows.front().min_eig), 1e-12);
  for (size_t i = 1; i < rows.size(); ++i) {
    EXPECT_GT(rows[i].q_mean, rows[i - 1].q_mean);
    EXPECT_GT(rows[i].min_eig, rows[i - 1].min_eig);
  }
  EXPECT_GT(rows.back().q_mean, 1e-2);
  EXPECT_GT(rows.back().min_eig, 1e-2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
