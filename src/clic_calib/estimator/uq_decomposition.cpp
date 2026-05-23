#include <clic_calib/estimator/uq_decomposition.h>

#include <cmath>
#include <iomanip>
#include <limits>

namespace clic_calib {
namespace {

struct ExtrinsicSixDofErrors {
  Eigen::Vector3d rot_err_rad = Eigen::Vector3d::Zero();
  Eigen::Vector3d trans_err_m = Eigen::Vector3d::Zero();
};

ExtrinsicSixDofErrors ExtrinsicError(const SE3d& T_est, const SE3d& T_gt) {
  ExtrinsicSixDofErrors e;
  const SO3d R_err = T_gt.so3().inverse() * T_est.so3();
  e.rot_err_rad = R_err.log();
  e.trans_err_m = T_est.translation() - T_gt.translation();
  return e;
}

double PresentationScale(int dof) {
  if (dof % 6 < 3) {
    return 180e3 / M_PI;
  }
  if (dof >= 12) {
    return 1e5;
  }
  return 1e3;
}

const char* PresentationUnit(int dof) {
  if (dof % 6 < 3) {
    return "mrad";
  }
  if (dof >= 12) {
    return "0.01ms";
  }
  return "mm";
}

bool IsZeroMeanQualified(int mc_dof, const McRunningStats& s) {
  if (mc_dof == 9 || mc_dof == 10) {
    const double st = s.Std();
    return st <= 1e-18 || std::abs(s.mean) <= st;
  }
  return true;
}

}  // namespace

void McRunningStats::Push(double x) {
  ++n;
  const double d = x - mean;
  mean += d / static_cast<double>(n);
  const double d2 = x - mean;
  m2 += d * d2;
}

double McRunningStats::Std() const {
  if (n < 2) {
    return 0.0;
  }
  return std::sqrt(m2 / static_cast<double>(n - 1));
}

const char* ExtrinsicMcDofName(int dof) {
  static const char* kNames[14] = {
      "LW_roll", "LW_pitch", "LW_yaw", "LW_tx", "LW_ty", "LW_tz",
      "CW_roll", "CW_pitch", "CW_yaw", "CW_tx", "CW_ty", "CW_tz",
      "t_d_L",   "t_d_C",
  };
  if (dof < 0 || dof >= 14) {
    return "?";
  }
  return kNames[dof];
}

const char* Stage2FimDofName(int fim_dof) {
  static const char* kLabels[14] = {
      "t_d_L",   "LW_roll",  "LW_pitch", "LW_yaw",  "LW_tx",  "LW_ty",
      "LW_tz",   "t_d_C",    "CW_roll",  "CW_pitch", "CW_yaw", "CW_tx",
      "CW_ty",   "CW_tz",
  };
  if (fim_dof < 0 || fim_dof >= 14) {
    return "?";
  }
  return kLabels[fim_dof];
}

int McDofFromFimDof(int fim_dof) {
  static const int kMap[14] = {12, 0, 1, 2, 3, 4, 5,
                               13, 6, 7, 8, 9, 10, 11};
  if (fim_dof < 0 || fim_dof >= 14) {
    return -1;
  }
  return kMap[fim_dof];
}

const McRunningStats* McArmDofConst(const ExtrinsicMcArmStats& arm, int dof) {
  switch (dof) {
    case 0: return &arm.lw_roll;
    case 1: return &arm.lw_pitch;
    case 2: return &arm.lw_yaw;
    case 3: return &arm.lw_tx;
    case 4: return &arm.lw_ty;
    case 5: return &arm.lw_tz;
    case 6: return &arm.cw_roll;
    case 7: return &arm.cw_pitch;
    case 8: return &arm.cw_yaw;
    case 9: return &arm.cw_tx;
    case 10: return &arm.cw_ty;
    case 11: return &arm.cw_tz;
    case 12: return &arm.t_d_L;
    case 13: return &arm.t_d_C;
    default: return nullptr;
  }
}

McRunningStats* McArmDof(ExtrinsicMcArmStats* arm, int dof) {
  return const_cast<McRunningStats*>(McArmDofConst(*arm, dof));
}

double SampleVar(const McRunningStats& s) {
  const double st = s.Std();
  return st * st;
}

void PushExtrinsicMcSample(ExtrinsicMcArmStats* out, const SE3d& T_LW_est,
                           const SE3d& T_CW_est, double t_d_L_est,
                           double t_d_C_est, const SE3d& T_LW_gt,
                           const SE3d& T_CW_gt, double t_d_L_gt,
                           double t_d_C_gt) {
  if (!out) {
    return;
  }
  const ExtrinsicSixDofErrors lw = ExtrinsicError(T_LW_est, T_LW_gt);
  const ExtrinsicSixDofErrors cw = ExtrinsicError(T_CW_est, T_CW_gt);
  out->lw_roll.Push(lw.rot_err_rad.x());
  out->lw_pitch.Push(lw.rot_err_rad.y());
  out->lw_yaw.Push(lw.rot_err_rad.z());
  out->lw_tx.Push(lw.trans_err_m.x());
  out->lw_ty.Push(lw.trans_err_m.y());
  out->lw_tz.Push(lw.trans_err_m.z());
  out->cw_roll.Push(cw.rot_err_rad.x());
  out->cw_pitch.Push(cw.rot_err_rad.y());
  out->cw_yaw.Push(cw.rot_err_rad.z());
  out->cw_tx.Push(cw.trans_err_m.x());
  out->cw_ty.Push(cw.trans_err_m.y());
  out->cw_tz.Push(cw.trans_err_m.z());
  out->t_d_L.Push(t_d_L_est - t_d_L_gt);
  out->t_d_C.Push(t_d_C_est - t_d_C_gt);
  ++out->converged;
}

Eigen::VectorXd TheoStdFromFixedFim(const Eigen::MatrixXd& F_fixed) {
  Eigen::VectorXd theo_std = Eigen::VectorXd::Zero(14);
  const SymmetricInformationReport fobs = AnalyzeInformationMatrix(F_fixed);
  if (!fobs.is_pd) {
    return theo_std;
  }
  const Eigen::MatrixXd cov =
      F_fixed.ldlt().solve(Eigen::MatrixXd::Identity(14, 14));
  for (int d = 0; d < 14; ++d) {
    if (cov(d, d) > 0.0) {
      theo_std(d) = std::sqrt(cov(d, d));
    }
  }
  return theo_std;
}

Stage2FimDiagnostic ComputeStage2FimDiagnostic(
    const BodyTrajectory& fixed_traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<AprilTagObservation>& tags,
    const ExtrinsicOptimizeState& lidar_state,
    const ExtrinsicOptimizeState& camera_state, const LeverArmConfig& levers,
    const NoiseModel& noise, double sphere_radius_m,
    const PinholeIntrinsics& K, const RadtanDistortion& dist, int marker_id) {
  Stage2FimDiagnostic out;
  out.information = BuildStage2ExtrinsicInformation(
      fixed_traj, lidar, tags, lidar_state, camera_state, levers, noise,
      sphere_radius_m, K, dist, marker_id);
  out.fim_report = AnalyzeInformationMatrix(out.information);
  out.theo_std = TheoStdFromFixedFim(out.information);
  return out;
}

DecompositionGateReport EvaluateDecompositionGate(
    const UqDecompositionMcResult& result, double lo, double hi) {
  DecompositionGateReport rep;
  for (int d = 0; d < 14; ++d) {
    const double vf = SampleVar(*McArmDofConst(result.fixed_traj, d));
    const double vp = SampleVar(*McArmDofConst(result.traj_prop, d));
    const double vs = vf + vp;
    const double vt = SampleVar(*McArmDofConst(result.total, d));
    const double ratio =
        vt > 1e-24 ? vs / vt : std::numeric_limits<double>::quiet_NaN();
    rep.decouple_traj_over_fixed[d] =
        vf > 1e-24 ? vp / vf : std::numeric_limits<double>::quiet_NaN();
    const bool pass = std::isfinite(ratio) && ratio >= lo && ratio <= hi;
    if (pass) {
      ++rep.in_band;
      if (d == 3 || d == 4 || d == 5 || d == 9 || d == 10) {
        ++rep.trans_in_band;
      }
      if (IsZeroMeanQualified(d, *McArmDofConst(result.total, d))) {
        ++rep.headline_in_band;
      }
    } else {
      rep.failed_dofs.push_back(ExtrinsicMcDofName(d));
    }
  }
  return rep;
}

TrajPropDominanceReport ComputeTrajPropDominance(
    const ExtrinsicMcArmStats& traj_prop) {
  TrajPropDominanceReport r;
  const int trans_idx[] = {3, 4, 5, 9, 10, 11};
  const int rot_idx[] = {0, 1, 2, 6, 7, 8};
  for (int idx : trans_idx) {
    const double v = SampleVar(*McArmDofConst(traj_prop, idx));
    r.trans_var_sum += v;
    if (idx <= 5) {
      r.lw_trans_var += v;
    }
  }
  for (int idx : rot_idx) {
    const double v = SampleVar(*McArmDofConst(traj_prop, idx));
    r.rot_var_sum += v;
    if (idx <= 5) {
      r.lw_rot_var += v;
    }
  }
  return r;
}

McBiasCaveatReport EvaluateMcBiasCaveats(const ExtrinsicMcArmStats& total) {
  McBiasCaveatReport r;
  r.cw_tx_mean_m = total.cw_tx.mean;
  r.cw_tx_std_m = total.cw_tx.Std();
  r.cw_ty_mean_m = total.cw_ty.mean;
  r.cw_ty_std_m = total.cw_ty.Std();
  r.cw_tx_zero_mean_violated =
      r.cw_tx_std_m > 1e-18 && std::abs(r.cw_tx_mean_m) > r.cw_tx_std_m;
  r.cw_ty_zero_mean_violated =
      r.cw_ty_std_m > 1e-18 && std::abs(r.cw_ty_mean_m) > r.cw_ty_std_m;
  return r;
}

int CountFimMcRatiosInBand(const ExtrinsicMcArmStats& mc,
                           const Eigen::VectorXd& theo_std_fim_order,
                           double lo, double hi) {
  struct Row {
    const McRunningStats* s;
    int fim_idx;
  };
  const Row rows[] = {
      {&mc.t_d_L, 0},       {&mc.lw_roll, 1},   {&mc.lw_pitch, 2},
      {&mc.lw_yaw, 3},      {&mc.lw_tx, 4},     {&mc.lw_ty, 5},
      {&mc.lw_tz, 6},       {&mc.t_d_C, 7},     {&mc.cw_roll, 8},
      {&mc.cw_pitch, 9},    {&mc.cw_yaw, 10},   {&mc.cw_tx, 11},
      {&mc.cw_ty, 12},      {&mc.cw_tz, 13},
  };
  int in_band = 0;
  for (const Row& row : rows) {
    const double th =
        row.fim_idx < theo_std_fim_order.size()
            ? theo_std_fim_order(row.fim_idx)
            : 0.0;
    if (th <= 1e-18) {
      continue;
    }
    const double ratio = row.s->Std() / th;
    if (std::isfinite(ratio) && ratio >= lo && ratio <= hi) {
      ++in_band;
    }
  }
  return in_band;
}

void PrintDecompositionValidationTable(std::ostream& os,
                                       const UqDecompositionMcResult& result,
                                       double lo, double hi,
                                       DecompositionGateReport* report_out) {
  const DecompositionGateReport rep =
      EvaluateDecompositionGate(result, lo, hi);
  if (report_out) {
    *report_out = rep;
  }

  os << std::scientific << std::setprecision(4);
  os << "\n  14-DoF variance decomposition (diag Σ):\n";
  os << "  DoF          Var_fixed    Var_traj     Var_sum"
     << "      Var_total    ratio(sum/tot)  traj/fixed\n";
  for (int d = 0; d < 14; ++d) {
    const double vf = SampleVar(*McArmDofConst(result.fixed_traj, d));
    const double vp = SampleVar(*McArmDofConst(result.traj_prop, d));
    const double vs = vf + vp;
    const double vt = SampleVar(*McArmDofConst(result.total, d));
    const double ratio =
        vt > 1e-24 ? vs / vt : std::numeric_limits<double>::quiet_NaN();
    const double decouple = rep.decouple_traj_over_fixed[d];
    const bool pass = std::isfinite(ratio) && ratio >= lo && ratio <= hi;
    os << "  " << std::setw(10) << std::left << ExtrinsicMcDofName(d)
       << std::right << std::setw(12) << vf << std::setw(12) << vp
       << std::setw(12) << vs << std::setw(12) << vt << std::setw(16) << ratio
       << std::setw(12) << decouple << (pass ? "  OK" : "") << "\n";
  }
  os << std::fixed;
  os << "\n  Per-DoF std (mrad / mm / 0.01 ms):\n";
  os << "  DoF          std_fixed    std_traj     std_sum      std_total\n";
  for (int d = 0; d < 14; ++d) {
    const double sf = McArmDofConst(result.fixed_traj, d)->Std();
    const double sp = McArmDofConst(result.traj_prop, d)->Std();
    const double st = McArmDofConst(result.total, d)->Std();
    const double ss = std::sqrt(SampleVar(*McArmDofConst(result.fixed_traj, d)) +
                                SampleVar(*McArmDofConst(result.traj_prop, d)));
    const double scale = PresentationScale(d);
    os << "  " << std::setw(10) << std::left << ExtrinsicMcDofName(d)
       << std::right << std::setprecision(3) << std::setw(12) << sf * scale
       << std::setw(12) << sp * scale << std::setw(12) << ss * scale
       << std::setw(12) << st * scale << "\n";
  }
  os << "\n  MC sampling note: independent-arm variance SE ~ sqrt(2/N)"
     << " ≈ 14% at N=100; band [" << lo << ", " << hi << "].\n";
}

void PrintMcBiasTable(std::ostream& os, const ExtrinsicMcArmStats& mc) {
  struct Row {
    const char* name;
    const McRunningStats& s;
    const char* unit;
  };
  const Row rows[] = {
      {"t_d_L", mc.t_d_L, "s"},       {"LW_roll", mc.lw_roll, "rad"},
      {"LW_pitch", mc.lw_pitch, "rad"}, {"LW_yaw", mc.lw_yaw, "rad"},
      {"LW_tx", mc.lw_tx, "m"},       {"LW_ty", mc.lw_ty, "m"},
      {"LW_tz", mc.lw_tz, "m"},       {"t_d_C", mc.t_d_C, "s"},
      {"CW_roll", mc.cw_roll, "rad"}, {"CW_pitch", mc.cw_pitch, "rad"},
      {"CW_yaw", mc.cw_yaw, "rad"},   {"CW_tx", mc.cw_tx, "m"},
      {"CW_ty", mc.cw_ty, "m"},       {"CW_tz", mc.cw_tz, "m"},
  };
  os << "\n  MC bias (mean estimate−GT; zero-mean assumption for std/ratio):\n";
  os << std::setw(12) << "DoF" << std::setw(20) << "mean_bias"
     << std::setw(16) << "emp_std" << std::setw(8) << "unit" << "\n";
  for (const Row& row : rows) {
    os << std::scientific << std::setprecision(16);
    os << std::setw(12) << row.name << std::setw(20) << row.s.mean
       << std::setw(16) << row.s.Std() << std::setw(8) << row.unit
       << std::defaultfloat << "\n";
  }
}

void PrintMcBiasTableWithCaveat(std::ostream& os,
                                const ExtrinsicMcArmStats& total) {
  PrintMcBiasTable(os, total);
  const McBiasCaveatReport caveat = EvaluateMcBiasCaveats(total);
  os << std::fixed << std::setprecision(2);
  os << "\n  Camera lateral translation (reported separately; not headline UQ):\n";
  os << "    CW_tx: " << caveat.cw_tx_mean_m * 1e3 << " ± "
     << caveat.cw_tx_std_m * 1e3 << " mm";
  if (caveat.cw_tx_zero_mean_violated) {
    os << "  |mean| > σ — zero-mean assumption violated";
  }
  os << "\n";
  os << "    CW_ty: " << caveat.cw_ty_mean_m * 1e3 << " ± "
     << caveat.cw_ty_std_m * 1e3 << " mm";
  if (caveat.cw_ty_zero_mean_violated) {
    os << "  |mean| > σ — zero-mean assumption violated";
  }
  os << "\n";
}

void PrintDecouplingCostTable(std::ostream& os,
                              const UqDecompositionMcResult& result) {
  os << "\n  Decoupling cost (two-stage price of separating trajectory):\n";
  os << "  DoF          Var_traj/Var_total   Var_traj/Var_fixed\n";
  for (int d = 0; d < 14; ++d) {
    const double vf = SampleVar(*McArmDofConst(result.fixed_traj, d));
    const double vp = SampleVar(*McArmDofConst(result.traj_prop, d));
    const double vt = SampleVar(*McArmDofConst(result.total, d));
    const double frac_total = vt > 1e-24 ? vp / vt : 0.0;
    const double frac_fixed = vf > 1e-24 ? vp / vf : 0.0;
    os << std::scientific << std::setprecision(4);
    os << "  " << std::setw(10) << std::left << ExtrinsicMcDofName(d)
       << std::right << std::setw(20) << frac_total << std::setw(20)
       << frac_fixed << std::defaultfloat << "\n";
  }
}

void PrintFimMcRatioTable(std::ostream& os, const ExtrinsicMcArmStats& mc,
                          const Eigen::VectorXd& theo_std_fim_order, double lo,
                          double hi, int* in_band_out) {
  struct Row {
    const char* name;
    const McRunningStats& s;
    int fim_idx;
  };
  const Row rows[] = {
      {"t_d_L", mc.t_d_L, 0},       {"LW_roll", mc.lw_roll, 1},
      {"LW_pitch", mc.lw_pitch, 2}, {"LW_yaw", mc.lw_yaw, 3},
      {"LW_tx", mc.lw_tx, 4},       {"LW_ty", mc.lw_ty, 5},
      {"LW_tz", mc.lw_tz, 6},       {"t_d_C", mc.t_d_C, 7},
      {"CW_roll", mc.cw_roll, 8},   {"CW_pitch", mc.cw_pitch, 9},
      {"CW_yaw", mc.cw_yaw, 10},    {"CW_tx", mc.cw_tx, 11},
      {"CW_ty", mc.cw_ty, 12},      {"CW_tz", mc.cw_tz, 13},
  };
  os << std::setw(12) << "DoF" << std::setw(16) << "emp_std"
     << std::setw(16) << "theo_std" << std::setw(12) << "ratio"
     << std::setw(8) << "in_band" << "\n";
  int in_band = 0;
  for (const Row& row : rows) {
    const double th = row.fim_idx < theo_std_fim_order.size()
                          ? theo_std_fim_order(row.fim_idx)
                          : 0.0;
    const double emp = row.s.Std();
    const double ratio =
        th > 1e-18 ? emp / th : std::numeric_limits<double>::quiet_NaN();
    const bool ok = std::isfinite(ratio) && ratio >= lo && ratio <= hi;
    if (ok) {
      ++in_band;
    }
    os << std::scientific << std::setprecision(16);
    os << std::setw(12) << row.name << std::setw(16) << emp << std::setw(16)
       << th << std::setw(12) << ratio << std::setw(8) << (ok ? "yes" : "NO")
       << std::defaultfloat << "\n";
  }
  os << "  summary: " << in_band << "/14 in band\n";
  if (in_band_out) {
    *in_band_out = in_band;
  }
}

void PrintStage2FimDiagnostic(std::ostream& os,
                              const Stage2FimDiagnostic& diag,
                              uint32_t rep_seed) {
  os << "\n=== Stage-2 fixed-trajectory FIM @ rep seed " << rep_seed
     << " ===\n";
  os << std::scientific << std::setprecision(4);
  os << "  rank=" << diag.fim_report.rank << "/14  cond=" << diag.fim_report.cond
     << "  λ_min=" << diag.fim_report.lambda_min
     << "  λ_max=" << diag.fim_report.lambda_max << std::defaultfloat << "\n";
  os << "  theo_std (FIM column order):\n";
  for (int d = 0; d < 14; ++d) {
    os << "    " << std::setw(10) << std::left << Stage2FimDofName(d)
       << std::scientific << std::setprecision(16) << diag.theo_std(d)
       << std::defaultfloat << "\n";
  }
}

}  // namespace clic_calib
