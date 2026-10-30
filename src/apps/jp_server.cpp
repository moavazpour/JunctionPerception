#include <CLI/CLI.hpp>
#include <string>

#include "jp/core/version.hpp"

int main(int argc, char** argv) {
  CLI::App app{"Serves the digital twin web UI, the REST API and the world-state stream",
               "jp_server"};
  app.set_version_flag("--version", std::string{jp::core::version()});
  CLI11_PARSE(app, argc, argv);
  return 0;
}
