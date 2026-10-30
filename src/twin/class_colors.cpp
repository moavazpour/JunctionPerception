#include "jp/twin/class_colors.hpp"

#include <map>
#include <string>

namespace jp::twin {

cv::Scalar class_color(std::string_view twin_class) {
  static const std::map<std::string, cv::Scalar, std::less<>> colors = {
      {"car", {80, 200, 80}},
      {"van", {80, 220, 200}},
      {"truck", {60, 140, 255}},
      {"trailer", {40, 90, 200}},
      {"bus", {200, 120, 40}},
      {"pedestrian", {60, 60, 240}},
      {"bicycle", {240, 80, 220}},
      {"motorcycle", {180, 60, 180}},
      {"emergency_vehicle", {0, 0, 160}},
      {"other", {160, 160, 160}}};
  const auto it = colors.find(twin_class);
  return it == colors.end() ? cv::Scalar(255, 255, 255) : it->second;
}

}  // namespace jp::twin
