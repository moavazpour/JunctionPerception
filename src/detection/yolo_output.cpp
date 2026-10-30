#include "jp/detection/yolo_output.hpp"

#include <charconv>
#include <cmath>
#include <map>
#include <stdexcept>

namespace jp::detection {

std::vector<RawDetection> parse_end2end_output(std::span<const float> output, float min_score) {
  if (output.size() % kEnd2EndRowSize != 0) {
    throw std::invalid_argument("end-to-end output size is not a multiple of 6");
  }
  std::vector<RawDetection> detections;
  for (std::size_t offset = 0; offset < output.size(); offset += kEnd2EndRowSize) {
    const auto row = output.subspan(offset, kEnd2EndRowSize);
    const float score = row[4];
    if (score < min_score) {
      continue;
    }
    RawDetection d;
    d.box = cv::Rect2f(row[0], row[1], row[2] - row[0], row[3] - row[1]);
    d.score = score;
    d.class_id = static_cast<int>(std::lround(row[5]));
    detections.push_back(d);
  }
  return detections;
}

std::vector<std::string> parse_class_names(std::string_view metadata) {
  std::map<int, std::string> by_id;
  std::size_t pos = 0;
  const auto fail = [&]() {
    throw std::invalid_argument("malformed class names: " + std::string(metadata));
  };
  while (true) {
    const std::size_t digit = metadata.find_first_of("0123456789", pos);
    if (digit == std::string_view::npos) {
      break;
    }
    int id = 0;
    const auto [after, error] =
        std::from_chars(metadata.data() + digit, metadata.data() + metadata.size(), id);
    if (error != std::errc{}) {
      fail();
    }
    const std::size_t colon = metadata.find(':', static_cast<std::size_t>(after - metadata.data()));
    const std::size_t open = metadata.find_first_of("'\"", colon);
    if (colon == std::string_view::npos || open == std::string_view::npos) {
      fail();
    }
    const std::size_t close = metadata.find(metadata[open], open + 1);
    if (close == std::string_view::npos) {
      fail();
    }
    by_id.emplace(id, std::string(metadata.substr(open + 1, close - open - 1)));
    pos = close + 1;
  }
  std::vector<std::string> names;
  names.reserve(by_id.size());
  for (const auto& [id, name] : by_id) {
    if (id != static_cast<int>(names.size())) {
      fail();
    }
    names.push_back(name);
  }
  return names;
}

}  // namespace jp::detection
