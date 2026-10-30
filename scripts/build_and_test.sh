#!/usr/bin/env bash
# Builds, lints and tests the C++ code and the frontend. Runs inside the development image.
# Usage: scripts/build_and_test.sh [cpp|web|all] [debug|release]
set -euo pipefail

target="${1:-all}"
preset="${2:-release}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

build_cpp() {
    find include src tests -name '*.hpp' -o -name '*.cpp' | xargs clang-format-18 --dry-run --Werror
    cmake --preset "${preset}"
    cmake --build --preset "${preset}"
    ctest --preset "${preset}"
    run-clang-tidy-18 -quiet -p "build/${preset}" "${repo_root}/src/"
}

build_web() {
    cd "${repo_root}/web"
    npm ci --no-audit --no-fund
    npm run lint
    npm run build
    npm test
}

case "${target}" in
    cpp) build_cpp ;;
    web) build_web ;;
    # Separate commands: `set -e` does not apply inside functions called from an `&&` list.
    all) build_cpp; build_web ;;
    *) echo "unknown target '${target}' (expected cpp, web or all)" >&2; exit 2 ;;
esac
