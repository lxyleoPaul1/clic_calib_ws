#pragma once

#include <clic_calib/utils/ceres_gflags_guard.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <cstring>
#include <string>

namespace clic_test_internal {

inline void SoftResetGflags(const char* progname) {
  int argc = 1;
  char default_name[] = "clic_test";
  char* argv[] = {progname ? const_cast<char*>(progname) : default_name,
                  nullptr};
  char** argv_ptr = argv;
  google::AllowCommandLineReparsing();
  google::ParseCommandLineFlags(&argc, &argv_ptr, false);
}

inline std::string ExtractGTestFilter(int argc, char** argv) {
  std::string filter;
  for (int i = 1; i < argc; ++i) {
    const char* arg = argv[i];
    if (std::strncmp(arg, "--gtest_filter=", 15) == 0) {
      filter = arg + 15;
    } else if (std::strcmp(arg, "--gtest_filter") == 0 && i + 1 < argc) {
      filter = argv[++i];
    }
  }
  return filter;
}

}  // namespace clic_test_internal

/** Hard reset after InitGoogleTest (gtest may have poisoned gflags). */
inline void ClicResetGflagsForCeres(const char* progname, bool init_logging) {
  if (init_logging) {
    google::InitGoogleLogging(progname ? progname : "clic_test");
    FLAGS_logtostderr = true;
    clic_test_internal::SoftResetGflags(progname);
  } else {
    google::ShutDownCommandLineFlags();
    clic_test_internal::SoftResetGflags(progname);
  }
}

/**
 * Ceres' first Solve calls google::ParseCommandLineFlags on process argv.
 * --gtest_filter must never reach InitGoogleTest / gflags or FIM metrics corrupt.
 */
inline int ClicGTestRunAll(int argc, char** argv) {
  const std::string gtest_filter =
      clic_test_internal::ExtractGTestFilter(argc, argv);

  // Ceres re-parses the process command line on later Solve calls. Replace argv
  // for the remaining process lifetime so it never sees spaced paths or gtest
  // flags (both corrupt gflags / FIM linearization).
  static char prog_storage[] = "clic_test";
  if (argc > 0) {
    argv[0] = prog_storage;
    for (int i = 1; i < argc; ++i) {
      argv[i] = nullptr;
    }
  }
  int clean_argc = 1;

  constexpr const char* kProg = "clic_test";
  ClicResetGflagsForCeres(kProg, true);
  testing::InitGoogleTest(&clean_argc, argv);
  if (!gtest_filter.empty()) {
    ::testing::GTEST_FLAG(filter) = gtest_filter;
  }
  ClicResetGflagsForCeres(kProg, false);
  return RUN_ALL_TESTS();
}
