#pragma once

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>

#include "jp/core/world_state.hpp"

namespace jp::twin {

/// World state as one line of JSON (schema in docs/architecture.md).
[[nodiscard]] std::string to_json(const core::WorldState& state);

/// Parses a world state; throws std::runtime_error on malformed JSON or an unsupported schema.
[[nodiscard]] core::WorldState world_state_from_json(std::string_view text);

/// Appends world states as JSON lines.
class JsonlWriter {
 public:
  /// Throws std::runtime_error if the file cannot be created.
  explicit JsonlWriter(const std::filesystem::path& path);
  void write(const core::WorldState& state);

 private:
  std::ofstream out_;
};

/// Reads world states from a JSON lines file.
class JsonlReader {
 public:
  /// Throws std::runtime_error if the file cannot be opened.
  explicit JsonlReader(const std::filesystem::path& path);
  /// Next state, or empty at the end of the file.
  [[nodiscard]] std::optional<core::WorldState> next();

 private:
  std::ifstream in_;
};

}  // namespace jp::twin
