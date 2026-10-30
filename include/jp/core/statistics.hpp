#pragma once

#include <cstddef>
#include <vector>

namespace jp::core {

/// Value at `fraction` (0..1) of the sorted values (nearest rank below); 0 for no values.
[[nodiscard]] double percentile(std::vector<double> values, double fraction);

struct Summary {
  std::size_t count = 0;
  double mean = 0.0;
  double median = 0.0;
  double p95 = 0.0;
  double max = 0.0;
};

[[nodiscard]] Summary summarize(const std::vector<double>& values);

}  // namespace jp::core
