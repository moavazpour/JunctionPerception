#include <CLI/CLI.hpp>
#include <string>

#include "jp/core/version.hpp"

int main(int argc, char** argv) {
  CLI::App app{"Fits and validates the image-to-ground homography of a scene", "jp_calibrate"};
  app.set_version_flag("--version", std::string{jp::core::version()});
  CLI11_PARSE(app, argc, argv);
  return 0;
}
