#pragma once

#include <clic_calib/estimator/stage2_extrinsic_fim.h>
#include <clic_calib/estimator/two_stage_types.h>
#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/lidar_target_observation.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>

#include <Eigen/Core>

#include <array>
#include <iosfwd>
#include <string>
#include <vector>

namespace clic_calib {

/** Welford online mean / sample std (N−1 denominator). */
struct McRunningStats {
  int n = 0;
  double mean = 0.0;
  double m2 = 0.0;

  void Push(double x);
  double Std() const;
};

/** 14-DoF extrinsic MC arm in presentation order (LW rot/trans, CW rot/trans, t_d). */
struct ExtrinsicMcArmStats {
  McRunningStats lw_roll;
  McRunningStats lw_pitch;
  McRunningStats lw_yaw;
  McRunningStats lw_tx;
  McRunningStats lw_ty;
  McRunningStats lw_tz;
  McRunningStats cw_roll;
  McRunningStats cw_pitch;
  McRunningStats cw_yaw;
  McRunningStats cw_tx;
  McRunningStats cw_ty;
  McRunningStats cw_tz;
  McRunningStats t_d_L;
  McRunningStats t_d_C;
  int converged = 0;
};

struct UqDecompositionBands {
  double additive_lo = 0.65;
  double additive_hi = 1.35;
  double decomp_lo = 0.8;
  double decomp_hi = 1.25;
  double fim_ratio_lo = 0.7;
  double fim_ratio_hi = 1.4;
};

struct UqDecompositionMcResult {
  ExtrinsicMcArmStats total;
  ExtrinsicMcArmStats fixed_traj;
  ExtrinsicMcArmStats traj_prop;
  bool rep_traj_ok = false;
};

struct DecompositionGateReport {
  int in_band = 0;
  int trans_in_band = 0;
  int headline_in_band = 0;
  std::vector<std::string> failed_dofs;
  std::array<double, 14> decouple_traj_over_fixed{};
};

struct TrajPropDominanceReport {
  double trans_var_sum = 0.0;
  double rot_var_sum = 0.0;
  double lw_trans_var = 0.0;
  double lw_rot_var = 0.0;
};

/** Per-run fixed-trajectory Stage-2 FIM diagnostic (FIM column order). */
struct Stage2FimDiagnostic {
  SymmetricInformationReport fim_report;
  Eigen::MatrixXd information = Eigen::MatrixXd::Zero(14, 14);
  /** Std dev per FIM column: t_d_L, LW×6, t_d_C, CW×6. */
  Eigen::VectorXd theo_std = Eigen::VectorXd::Zero(14);
};

struct McBiasCaveatReport {
  double cw_tx_mean_m = 0.0;
  double cw_tx_std_m = 0.0;
  double cw_ty_mean_m = 0.0;
  double cw_ty_std_m = 0.0;
  bool cw_tx_zero_mean_violated = false;
  bool cw_ty_zero_mean_violated = false;
};

constexpr int kNumExtrinsicMcDofs = 14;

const char* ExtrinsicMcDofName(int dof);
const McRunningStats* McArmDofConst(const ExtrinsicMcArmStats& arm, int dof);
McRunningStats* McArmDof(ExtrinsicMcArmStats* arm, int dof);

/** Map FIM column index → ExtrinsicMcArmStats DoF index. */
int McDofFromFimDof(int fim_dof);
const char* Stage2FimDofName(int fim_dof);

double SampleVar(const McRunningStats& s);

void PushExtrinsicMcSample(ExtrinsicMcArmStats* out, const SE3d& T_LW_est,
                           const SE3d& T_CW_est, double t_d_L_est,
                           double t_d_C_est, const SE3d& T_LW_gt,
                           const SE3d& T_CW_gt, double t_d_L_gt,
                           double t_d_C_gt);

Eigen::VectorXd TheoStdFromFixedFim(const Eigen::MatrixXd& F_fixed);

Stage2FimDiagnostic ComputeStage2FimDiagnostic(
    const BodyTrajectory& fixed_traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<AprilTagObservation>& tags,
    const ExtrinsicOptimizeState& lidar_state,
    const ExtrinsicOptimizeState& camera_state, const LeverArmConfig& levers,
    const NoiseModel& noise, double sphere_radius_m,
    const PinholeIntrinsics& K, const RadtanDistortion& dist,
    int marker_id = 0);

DecompositionGateReport EvaluateDecompositionGate(
    const UqDecompositionMcResult& result, double lo, double hi);

TrajPropDominanceReport ComputeTrajPropDominance(
    const ExtrinsicMcArmStats& traj_prop);

McBiasCaveatReport EvaluateMcBiasCaveats(const ExtrinsicMcArmStats& total);

int CountFimMcRatiosInBand(const ExtrinsicMcArmStats& mc,
                           const Eigen::VectorXd& theo_std_fim_order,
                           double lo, double hi);

void PrintDecompositionValidationTable(std::ostream& os,
                                       const UqDecompositionMcResult& result,
                                       double lo, double hi,
                                       DecompositionGateReport* report_out);

void PrintMcBiasTable(std::ostream& os, const ExtrinsicMcArmStats& mc);

void PrintMcBiasTableWithCaveat(std::ostream& os,
                                const ExtrinsicMcArmStats& total);

void PrintDecouplingCostTable(std::ostream& os,
                              const UqDecompositionMcResult& result);

void PrintFimMcRatioTable(std::ostream& os, const ExtrinsicMcArmStats& mc,
                          const Eigen::VectorXd& theo_std_fim_order, double lo,
                          double hi, int* in_band_out);

void PrintStage2FimDiagnostic(std::ostream& os,
                            const Stage2FimDiagnostic& diag,
                            uint32_t rep_seed);

}  // namespace clic_calib
