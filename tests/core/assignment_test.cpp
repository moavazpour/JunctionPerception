#include "jp/core/assignment.hpp"

#include <gtest/gtest.h>

namespace {

TEST(Assignment, SquareMatrixFindsOptimum) {
  Eigen::MatrixXd cost(3, 3);
  cost << 4, 1, 3,  //
      2, 0, 5,      //
      3, 2, 2;
  // Optimum 1 + 2 + 2 = 5: row 0 -> col 1, row 1 -> col 0, row 2 -> col 2.
  EXPECT_EQ(jp::core::solve_assignment(cost), (std::vector<Eigen::Index>{1, 0, 2}));
}

TEST(Assignment, MoreColumnsThanRows) {
  Eigen::MatrixXd cost(2, 3);
  cost << 9, 1, 9,  //
      1, 9, 9;
  EXPECT_EQ(jp::core::solve_assignment(cost), (std::vector<Eigen::Index>{1, 0}));
}

TEST(Assignment, MoreRowsThanColumnsLeavesRowsUnassigned) {
  Eigen::MatrixXd cost(3, 1);
  cost << 5, 1, 3;
  EXPECT_EQ(jp::core::solve_assignment(cost), (std::vector<Eigen::Index>{-1, 0, -1}));
}

TEST(Assignment, EmptyMatrix) {
  EXPECT_EQ(jp::core::solve_assignment(Eigen::MatrixXd(2, 0)), (std::vector<Eigen::Index>{-1, -1}));
}

}  // namespace
