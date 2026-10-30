#include <CLI/CLI.hpp>
#include <string>

#include "jp/core/version.hpp"

int main(int argc, char** argv) {
  CLI::App app{"Runs the perception pipeline on a video source and writes the world state",
               "jp_run"};
  app.set_version_flag("--version", std::string{jp::core::version()});
  CLI11_PARSE(app, argc, argv);
  return 0;
}
