#pragma once

#include <gflags/gflags.h>

namespace clic_calib {

/** Re-parse gflags from progname-only argv (no ShutDown). */
inline void SoftResetGflagsForCeres() {
  int argc = 1;
  char prog[] = "clic_test";
  char* argv[] = {prog, nullptr};
  char** argv_ptr = argv;
  google::AllowCommandLineReparsing();
  google::ParseCommandLineFlags(&argc, &argv_ptr, false);
}

/**
 * Called before Ceres Solve. After gtest/ClicGTestRunAll has sanitized argv and
 * done the one-time hard reset, repeated ShutDown here makes Ceres re-parse from
 * stale state and corrupts later solves — keep this a soft re-parse only.
 */
inline void ResetGflagsForCeresSolve() { SoftResetGflagsForCeres(); }

}  // namespace clic_calib
