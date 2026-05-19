#include <clic_calib/estimator/observability_analyzer.h>

#include <iostream>

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  clic_calib::ObservabilityAnalyzer analyzer;
  const auto report = analyzer.AnalyzeCurrentProblem();
  std::cout << "Observability analyzer stub (Phase 0). PDOP_ext="
            << report.pdop_ext << " lambda_min=" << report.lambda_min << "\n";
  return 0;
}
