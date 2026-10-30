#include "jp/benchmark/tumtraf_sequence.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace jp::benchmark {

namespace {

std::vector<std::filesystem::path> sorted_files(const std::filesystem::path& dir,
                                                std::string_view extension) {
  if (!std::filesystem::is_directory(dir)) {
    throw std::runtime_error("missing folder: " + dir.string());
  }
  std::vector<std::filesystem::path> files;
  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    if (entry.is_regular_file() && entry.path().extension() == extension) {
      files.push_back(entry.path());
    }
  }
  std::ranges::sort(files);
  return files;
}

std::string read_text(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("cannot read " + path.string());
  }
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

}  // namespace

TumtrafSequence TumtrafSequence::open(const std::filesystem::path& sequence_dir) {
  const auto labels_dir = sequence_dir / "labels_point_clouds";
  const auto south_files = sorted_files(labels_dir / kSouthLidar, ".json");
  const auto north_files = sorted_files(labels_dir / kNorthLidar, ".json");
  if (south_files.empty() || south_files.size() != north_files.size()) {
    throw std::runtime_error("south and north label sets differ in length in " +
                             sequence_dir.string());
  }

  TumtrafSequence sequence;
  sequence.name_ = sequence_dir.filename().string();
  sequence.frames_.reserve(south_files.size());
  for (std::size_t i = 0; i < south_files.size(); ++i) {
    LabelFrame south = parse_openlabel_frame(read_text(south_files[i]), kSouthLidar);
    LabelFrame north = parse_openlabel_frame(read_text(north_files[i]), kNorthLidar);
    if (south.image_file_names != north.image_file_names) {
      throw std::runtime_error("label files " + south_files[i].filename().string() + " and " +
                               north_files[i].filename().string() + " refer to different images");
    }
    if (i == 0) {
      sequence.cameras_ = south.cameras;
    }

    SequenceFrame frame;
    frame.index = i;
    for (const auto& image_name : south.image_file_names) {
      const std::string camera = sensor_from_file_name(image_name);
      frame.images.emplace(camera, CameraImage{sequence_dir / "images" / camera / image_name,
                                               timestamp_from_file_name(image_name)});
    }
    frame.south_objects = std::move(south.objects);
    frame.north_objects = std::move(north.objects);
    sequence.frames_.push_back(std::move(frame));
  }
  return sequence;
}

const CameraCalibration& TumtrafSequence::camera(std::string_view camera_name) const {
  const auto it = cameras_.find(camera_name);
  if (it == cameras_.end()) {
    throw std::out_of_range("unknown camera: " + std::string(camera_name));
  }
  return it->second;
}

std::vector<std::string> TumtrafSequence::camera_names() const {
  std::vector<std::string> names;
  names.reserve(cameras_.size());
  for (const auto& [name, camera] : cameras_) {
    names.push_back(name);
  }
  return names;
}

}  // namespace jp::benchmark
