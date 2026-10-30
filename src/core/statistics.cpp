#include "jp/core/statistics.hpp"

#include <algorithm>
#include <numeric>

namespace jp::core {

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  const auto k = static_cast<std::size_t>(fraction * static_cast<double>(values.size() - 1));
  std::ranges::nth_element(values, values.begin() + static_cast<std::ptrdiff_t>(k));
  return values[k];
}

Summary summarize(const std::vector<double>& values) {
  Summary s;
  s.count = values.size();
  if (values.empty()) {
    return s;
  }
  s.mean = std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
  s.median = percentile(values, 0.5);
  s.p95 = percentile(values, 0.95);
  s.max = *std::ranges::max_element(values);
  return s;
}

}  // namespace jp::core
