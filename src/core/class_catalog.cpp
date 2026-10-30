#include "jp/core/class_catalog.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <format>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace jp::core {

ClassCatalog ClassCatalog::from_yaml(std::string_view yaml_text) {
  ClassCatalog catalog;
  try {
    const YAML::Node root = YAML::Load(std::string(yaml_text));
    for (const auto& entry : root["classes"]) {
      const YAML::Node& value = entry.second;
      catalog.classes_.push_back({entry.first.as<std::string>(), value["length_m"].as<double>(),
                                  value["width_m"].as<double>(), value["vru"].as<bool>()});
    }
    for (const auto& source : root["sources"]) {
      auto& mapping = catalog.sources_[source.first.as<std::string>()];
      for (const auto& label : source.second) {
        mapping.emplace(label.first.as<std::string>(), label.second.as<std::string>());
      }
    }
  } catch (const YAML::Exception& error) {
    throw std::runtime_error(std::string("invalid class catalog: ") + error.what());
  }
  if (catalog.classes_.empty()) {
    throw std::runtime_error("class catalog defines no classes");
  }
  for (const auto& [source, mapping] : catalog.sources_) {
    for (const auto& [label, target] : mapping) {
      if (std::ranges::none_of(catalog.classes_,
                               [&](const ClassInfo& c) { return c.name == target; })) {
        throw std::runtime_error(
            std::format("source {} maps {} to unknown class {}", source, label, target));
      }
    }
  }
  return catalog;
}

ClassCatalog ClassCatalog::load(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot read " + path.string());
  }
  std::ostringstream text;
  text << in.rdbuf();
  return from_yaml(text.str());
}

const ClassInfo& ClassCatalog::info(std::string_view class_name) const {
  const auto it =
      std::ranges::find_if(classes_, [&](const ClassInfo& c) { return c.name == class_name; });
  if (it == classes_.end()) {
    throw std::out_of_range("unknown class: " + std::string(class_name));
  }
  return *it;
}

std::optional<std::string> ClassCatalog::map(std::string_view source,
                                             std::string_view label) const {
  const auto mapping = sources_.find(source);
  if (mapping == sources_.end()) {
    return std::nullopt;
  }
  const auto target = mapping->second.find(label);
  if (target == mapping->second.end()) {
    return std::nullopt;
  }
  return target->second;
}

}  // namespace jp::core
