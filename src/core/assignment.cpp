#include "jp/core/assignment.hpp"

#include <limits>

namespace jp::core {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Shortest augmenting path variant of the Hungarian method for rows <= cols. Indices are
// 1-based with a virtual column 0, as in the classic formulation.
class Hungarian {
 public:
  explicit Hungarian(const Eigen::MatrixXd& cost)
      : cost_(&cost),
        rows_(cost.rows()),
        cols_(cost.cols()),
        u_(Eigen::VectorXd::Zero(rows_ + 1)),
        v_(Eigen::VectorXd::Zero(cols_ + 1)),
        row_of_col_(static_cast<std::size_t>(cols_ + 1), 0),
        previous_col_(static_cast<std::size_t>(cols_ + 1), 0),
        min_slack_(cols_ + 1),
        used_(static_cast<std::size_t>(cols_ + 1), 0) {}

  std::vector<Eigen::Index> solve() {
    for (Eigen::Index row = 1; row <= rows_; ++row) {
      add_row(row);
    }
    std::vector<Eigen::Index> result(static_cast<std::size_t>(rows_), -1);
    for (Eigen::Index col = 1; col <= cols_; ++col) {
      const Eigen::Index row = row_of(col);
      if (row != 0) {
        result[static_cast<std::size_t>(row - 1)] = col - 1;
      }
    }
    return result;
  }

 private:
  Eigen::Index& row_of(Eigen::Index col) { return row_of_col_[static_cast<std::size_t>(col)]; }
  Eigen::Index& previous_of(Eigen::Index col) {
    return previous_col_[static_cast<std::size_t>(col)];
  }
  char& used(Eigen::Index col) { return used_[static_cast<std::size_t>(col)]; }

  // Grows the alternating tree from `row` until a free column is reached, then augments.
  void add_row(Eigen::Index row) {
    row_of(0) = row;
    min_slack_.setConstant(kInf);
    std::fill(used_.begin(), used_.end(), char{0});
    Eigen::Index col = 0;
    while (true) {
      col = expand(col);
      if (row_of(col) == 0) {
        break;
      }
    }
    while (col != 0) {
      const Eigen::Index previous = previous_of(col);
      row_of(col) = row_of(previous);
      col = previous;
    }
  }

  // Marks `col0` as used, updates slacks and potentials, returns the next column to visit.
  Eigen::Index expand(Eigen::Index col0) {
    used(col0) = 1;
    const Eigen::Index row0 = row_of(col0);
    double delta = kInf;
    Eigen::Index next = 0;
    for (Eigen::Index col = 1; col <= cols_; ++col) {
      if (used(col) != 0) {
        continue;
      }
      const double slack = (*cost_)(row0 - 1, col - 1) - u_(row0) - v_(col);
      if (slack < min_slack_(col)) {
        min_slack_(col) = slack;
        previous_of(col) = col0;
      }
      if (min_slack_(col) < delta) {
        delta = min_slack_(col);
        next = col;
      }
    }
    for (Eigen::Index col = 0; col <= cols_; ++col) {
      if (used(col) != 0) {
        u_(row_of(col)) += delta;
        v_(col) -= delta;
      } else {
        min_slack_(col) -= delta;
      }
    }
    return next;
  }

  const Eigen::MatrixXd* cost_;
  Eigen::Index rows_;
  Eigen::Index cols_;
  Eigen::VectorXd u_;
  Eigen::VectorXd v_;
  std::vector<Eigen::Index> row_of_col_;
  std::vector<Eigen::Index> previous_col_;
  Eigen::VectorXd min_slack_;
  std::vector<char> used_;
};

}  // namespace

std::vector<Eigen::Index> solve_assignment(const Eigen::MatrixXd& cost) {
  if (cost.rows() == 0 || cost.cols() == 0) {
    return std::vector<Eigen::Index>(static_cast<std::size_t>(cost.rows()), -1);
  }
  if (cost.rows() <= cost.cols()) {
    return Hungarian(cost).solve();
  }
  const Eigen::MatrixXd transposed = cost.transpose();
  const std::vector<Eigen::Index> row_of_col = Hungarian(transposed).solve();
  std::vector<Eigen::Index> result(static_cast<std::size_t>(cost.rows()), -1);
  for (std::size_t col = 0; col < row_of_col.size(); ++col) {
    if (row_of_col[col] >= 0) {
      result[static_cast<std::size_t>(row_of_col[col])] = static_cast<Eigen::Index>(col);
    }
  }
  return result;
}

}  // namespace jp::core
