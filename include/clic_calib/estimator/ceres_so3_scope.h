#pragma once

#include <clic_calib/factor/ceres_local_param.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>

#include <memory>
#include <vector>

namespace clic_calib {

/** Ceres problem scope: shared SO3 local param, DO_NOT_TAKE_OWNERSHIP (SIGSEGV fix). */
struct CeresSo3ProblemScope {
  ceres::Problem::Options opts;
  std::unique_ptr<ceres::Problem> problem;
  std::vector<std::unique_ptr<ceres::CostFunction>> owned_costs;
  std::unique_ptr<LieLocalParameterization<SO3d>> so3_local;

  CeresSo3ProblemScope() {
    opts.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    opts.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    opts.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem = std::make_unique<ceres::Problem>(opts);
    so3_local = std::make_unique<LieLocalParameterization<SO3d>>();
  }

  void SetLocalParamSO3(double* q) {
    if (problem->HasParameterBlock(q)) {
      problem->SetParameterization(q, so3_local.get());
    }
  }
};

}  // namespace clic_calib
