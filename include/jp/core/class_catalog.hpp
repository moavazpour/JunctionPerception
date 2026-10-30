#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace jp::core {

/// A twin object class with its fixed footprint prior.
struct ClassInfo {
  std::string name;
  double length_m = 0.0;
  double width_m = 0.0;
  double height_m = 0.0;
  bool vru = false;  ///< Vulnerable road user.
};

/// Twin classes and the mapping of source labels (detector classes, dataset types) onto them,
/// as configured in configs/classes.yaml.
class ClassCatalog {
 public:
  /// Parses the YAML text; throws std::runtime_error if it is malformed or maps a source label
  /// onto an unknown class.
  [[nodiscard]] static ClassCatalog from_yaml(std::string_view yaml_text);
  [[nodiscard]] static ClassCatalog load(const std::filesystem::path& path);

  [[nodiscard]] const std::vector<ClassInfo>& classes() const { return classes_; }
  /// Class by name; throws std::out_of_range for unknown names.
  [[nodiscard]] const ClassInfo& info(std::string_view class_name) const;
  /// Twin class of a source label, e.g. ("coco", "person") -> "pedestrian"; empty if unmapped.
  [[nodiscard]] std::optional<std::string> map(std::string_view source,
                                               std::string_view label) const;

 private:
  std::vector<ClassInfo> classes_;
  std::map<std::string, std::map<std::string, std::string, std::less<>>, std::less<>> sources_;
};

}  // namespace jp::core
