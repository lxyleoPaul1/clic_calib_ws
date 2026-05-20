#include <clic_calib/estimator/observability_analyzer.h>

#include <clic_calib/estimator/calibration_estimator.h>

#include <ceres/crs_matrix.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace clic_calib {
namespace {

Eigen::MatrixXd CRSToDense(const ceres::CRSMatrix& crs) {
  Eigen::MatrixXd J =
      Eigen::MatrixXd::Zero(crs.num_rows, std::max(0, crs.num_cols));
  for (int r = 0; r < crs.num_rows; ++r) {
    for (int k = crs.rows[r]; k < crs.rows[r + 1]; ++k) {
      J(r, crs.cols[k]) = crs.values[k];
    }
  }
  return J;
}

Eigen::MatrixXd InformationFromCRSJacobian(const ceres::CRSMatrix& crs) {
  const int n = std::max(0, crs.num_cols);
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(n, n);
  for (int r = 0; r < crs.num_rows; ++r) {
    const int begin = crs.rows[r];
    const int end = crs.rows[r + 1];
    for (int k = begin; k < end; ++k) {
      const int col_k = crs.cols[k];
      const double val_k = crs.values[k];
      for (int l = k; l < end; ++l) {
        const int col_l = crs.cols[l];
        const double val_l = crs.values[l];
        F(col_k, col_l) += val_k * val_l;
        if (col_k != col_l) {
          F(col_l, col_k) += val_k * val_l;
        }
      }
    }
  }
  return F;
}

Eigen::MatrixXd ExtractSubmatrix(const Eigen::MatrixXd& F,
                                 const std::vector<int>& rows,
                                 const std::vector<int>& cols) {
  Eigen::MatrixXd out(rows.size(), cols.size());
  for (size_t i = 0; i < rows.size(); ++i) {
    for (size_t j = 0; j < cols.size(); ++j) {
      out(i, j) = F(rows[i], cols[j]);
    }
  }
  return out;
}

Eigen::MatrixXd SchurComplementExtrinsic(const Eigen::MatrixXd& F,
                                           const AnalysisParameterLayout& layout) {
  const auto& ext = layout.extrinsic_local_indices;
  const auto& rest = layout.rest_local_indices;
  if (ext.empty()) {
    throw std::runtime_error("SchurComplementExtrinsic: no extrinsic parameters");
  }
  if (rest.empty()) {
    return ExtractSubmatrix(F, ext, ext);
  }

  const Eigen::MatrixXd F_ee = ExtractSubmatrix(F, ext, ext);
  const Eigen::MatrixXd F_er = ExtractSubmatrix(F, ext, rest);
  const Eigen::MatrixXd F_re = ExtractSubmatrix(F, rest, ext);
  const Eigen::MatrixXd F_rr = ExtractSubmatrix(F, rest, rest);

  const Eigen::LDLT<Eigen::MatrixXd> ldlt(F_rr);
  Eigen::MatrixXd F_rr_inv_F_re;
  if (ldlt.info() == Eigen::Success) {
    F_rr_inv_F_re = ldlt.solve(F_re);
  } else {
    F_rr_inv_F_re = F_rr.completeOrthogonalDecomposition().solve(F_re);
  }
  return F_ee - F_er * F_rr_inv_F_re;
}

Eigen::MatrixXd CorrelationFromCovariance(const Eigen::MatrixXd& cov) {
  const int n = static_cast<int>(cov.rows());
  Eigen::MatrixXd corr = Eigen::MatrixXd::Identity(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const double denom = std::sqrt(std::max(cov(i, i), 0.0) * std::max(cov(j, j), 0.0));
      corr(i, j) = denom > 0.0 ? cov(i, j) / denom : 0.0;
      corr(j, i) = corr(i, j);
    }
  }
  return corr;
}

void WriteJsonArray(std::ostream& os, const Eigen::VectorXd& v) {
  os << "[";
  for (int i = 0; i < v.size(); ++i) {
    if (i > 0) {
      os << ", ";
    }
    os << std::setprecision(16) << v(i);
  }
  os << "]";
}

void WriteJsonMatrix(std::ostream& os, const Eigen::MatrixXd& M) {
  os << "[";
  for (int r = 0; r < M.rows(); ++r) {
    if (r > 0) {
      os << ", ";
    }
    os << "[";
    for (int c = 0; c < M.cols(); ++c) {
      if (c > 0) {
        os << ", ";
      }
      os << std::setprecision(16) << M(r, c);
    }
    os << "]";
  }
  os << "]";
}

}  // namespace

ObservabilityReport ObservabilityAnalyzer::analyze(
    const CalibrationEstimator& estimator) const {
  const ceres::Problem& problem = estimator.problem();
  const AnalysisParameterLayout layout = estimator.analysis_parameter_layout();

  ceres::Problem::EvaluateOptions eval_opts;
  eval_opts.apply_loss_function = false;
  eval_opts.num_threads = 1;

  double cost = 0.0;
  ceres::CRSMatrix jacobian_crs;
  std::vector<double> residuals;
  ceres::Problem& mutable_problem =
      const_cast<ceres::Problem&>(problem);
  if (!mutable_problem.Evaluate(eval_opts, &cost, &residuals, nullptr,
                                &jacobian_crs)) {
    throw std::runtime_error("ObservabilityAnalyzer: Ceres Evaluate failed");
  }

  const Eigen::MatrixXd F = InformationFromCRSJacobian(jacobian_crs);
  if (jacobian_crs.num_cols != layout.num_local_parameters) {
    throw std::runtime_error(
        "ObservabilityAnalyzer: Jacobian column count mismatch with layout");
  }

  ObservabilityReport report;
  report.information_matrix = SchurComplementExtrinsic(F, layout);

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(report.information_matrix);
  if (es.info() != Eigen::Success) {
    throw std::runtime_error("ObservabilityAnalyzer: eigen decomposition failed");
  }

  report.eigenvalues = es.eigenvalues();
  report.lambda_min = report.eigenvalues.minCoeff();
  report.lambda_max = report.eigenvalues.maxCoeff();
  report.condition_number = report.lambda_min > 1e-9
                                ? report.lambda_max / report.lambda_min
                                : 1e300;

  double pdop_trace = 0.0;
  if (report.lambda_min > 1e-9) {
    const Eigen::MatrixXd cov_ext =
        report.information_matrix.ldlt().solve(
            Eigen::MatrixXd::Identity(report.information_matrix.rows(),
                                      report.information_matrix.cols()));
    const int dim = static_cast<int>(report.information_matrix.rows());
    for (int sensor = 0; sensor < dim / 6; ++sensor) {
      const int t0 = sensor * 6 + 3;
      for (int k = 0; k < 3; ++k) {
        if (t0 + k < dim) {
          pdop_trace += std::max(cov_ext(t0 + k, t0 + k), 0.0);
        }
      }
    }
    report.pdop_ext = std::sqrt(pdop_trace);
    report.correlation_matrix = CorrelationFromCovariance(cov_ext);
  } else {
    report.pdop_ext = 1e9;
    report.correlation_matrix =
        Eigen::MatrixXd::Identity(report.information_matrix.rows(),
                                  report.information_matrix.cols());
  }

  report.worst_eigenvector = es.eigenvectors().col(0);

  if (layout.lidar_trans_fext_indices.size() == 3) {
    report.worst_direction = Eigen::Vector3d(
        report.worst_eigenvector(layout.lidar_trans_fext_indices[0]),
        report.worst_eigenvector(layout.lidar_trans_fext_indices[1]),
        report.worst_eigenvector(layout.lidar_trans_fext_indices[2]));
    const double n = report.worst_direction.norm();
    if (n > 0.0) {
      report.worst_direction /= n;
    }
  }

  return report;
}

bool ObservabilityAnalyzer::WriteReportJson(const ObservabilityReport& report,
                                            const std::string& path) {
  std::ofstream os(path);
  if (!os) {
    return false;
  }

  os << std::setprecision(16);
  os << "{\n";
  os << "  \"lambda_min\": " << report.lambda_min << ",\n";
  os << "  \"lambda_max\": " << report.lambda_max << ",\n";
  os << "  \"condition_number\": " << report.condition_number << ",\n";
  os << "  \"pdop_ext\": " << report.pdop_ext << ",\n";
  os << "  \"eigenvalues\": ";
  WriteJsonArray(os, report.eigenvalues);
  os << ",\n";
  os << "  \"worst_eigenvector\": ";
  WriteJsonArray(os, report.worst_eigenvector);
  os << ",\n";
  os << "  \"worst_direction\": ["
     << report.worst_direction.x() << ", " << report.worst_direction.y()
     << ", " << report.worst_direction.z() << "],\n";
  os << "  \"information_matrix\": ";
  WriteJsonMatrix(os, report.information_matrix);
  os << ",\n";
  os << "  \"correlation_matrix\": ";
  WriteJsonMatrix(os, report.correlation_matrix);
  os << "\n}\n";
  return static_cast<bool>(os);
}

}  // namespace clic_calib
