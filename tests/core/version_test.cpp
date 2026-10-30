#include "jp/core/version.hpp"

#include <gtest/gtest.h>

TEST(Version, MatchesProjectVersion) { EXPECT_EQ(jp::core::version(), JP_EXPECTED_VERSION); }
