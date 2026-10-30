#pragma once

#include <opencv2/core/types.hpp>
#include <string_view>

namespace jp::twin {

/// Debug drawing colour (BGR) of a twin class; white for unknown classes.
[[nodiscard]] cv::Scalar class_color(std::string_view twin_class);

}  // namespace jp::twin
