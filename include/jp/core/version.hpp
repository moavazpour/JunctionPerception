#pragma once

#include <string_view>

namespace jp::core {

/// Semantic version of the project (major.minor.patch), taken from the CMake project version.
[[nodiscard]] std::string_view version() noexcept;

}  // namespace jp::core
