#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

/** Re-parse gflags from a minimal argv (progname only). */
inline void ClicResetGflagsForCeres(const char* progname, bool init_logging) {
  if (init_logging) {
    google::InitGoogleLogging(progname ? progname : "clic_test");
    FLAGS_logtostderr = true;
  }
  int argc = 1;
  char default_name[] = "clic_test";
  char* argv[] = {progname ? const_cast<char*>(progname) : default_name,
                  nullptr};
  char** argv_ptr = argv;
  google::AllowCommandLineReparsing();
  google::ParseCommandLineFlags(&argc, &argv_ptr, false);
}

/**
 * Ceres' first Solve calls google::ParseCommandLineFlags on process argv.
 * gtest also touches gflags; colon-separated --gtest_filter values are
 * mis-read as gflags and corrupt solver/FIM metrics unless we reset after
 * InitGoogleTest.
 */
inline int ClicGTestRunAll(int argc, char** argv) {
  const char* prog = argc > 0 ? argv[0] : nullptr;
  ClicResetGflagsForCeres(prog, true);
  testing::InitGoogleTest(&argc, argv);
  ClicResetGflagsForCeres(prog, false);
  return RUN_ALL_TESTS();
}
