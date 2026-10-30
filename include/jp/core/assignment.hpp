#pragma once

#include <Eigen/Core>
#include <vector>

namespace jp::core {

/// Minimum-cost assignment of rows to columns (Hungarian method) for a rectangular cost matrix.
/// Returns the assigned column for every row, or -1 for rows left unassigned (possible only when
/// there are more rows than columns). Gating of expensive pairs is up to the caller.
[[nodiscard]] std::vector<Eigen::Index> solve_assignment(const Eigen::MatrixXd& cost);

}  // namespace jp::core
