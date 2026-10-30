#include <CLI/CLI.hpp>
#include <string>

#include "jp/core/version.hpp"

int main(int argc, char** argv) {
  CLI::App app{"Evaluates recorded world states against benchmark ground truth", "jp_eval"};
  app.set_version_flag("--version", std::string{jp::core::version()});
  CLI11_PARSE(app, argc, argv);
  return 0;
}
