#!/usr/bin/env bash
# Pushes main to GitHub and to the server's working copy, then builds and tests on the server.
# Configuration comes from deploy.env in the repository root (see deploy.env.example).
# Usage: scripts/deploy.sh [cpp|web|all]
set -euo pipefail

target="${1:-all}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

if [[ ! -f deploy.env ]]; then
    echo "deploy.env not found; copy deploy.env.example and fill it in" >&2
    exit 1
fi
# shellcheck source=/dev/null
source deploy.env
: "${DEPLOY_HOST:?}" "${DEPLOY_USER:?}" "${DEPLOY_PATH:?}"
ssh_bin="${DEPLOY_SSH:-ssh}"
remote="${DEPLOY_USER}@${DEPLOY_HOST}"

if [[ "$(git rev-parse --abbrev-ref HEAD)" != "main" ]]; then
    echo "deploy runs from main only" >&2
    exit 1
fi
if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "working tree has uncommitted changes" >&2
    exit 1
fi

export GIT_SSH_COMMAND="${ssh_bin}"
git push origin main
git push "${remote}:${DEPLOY_PATH}" main

"${ssh_bin}" "${remote}" "cd '${DEPLOY_PATH}' \
    && scripts/setup_server.sh \
    && scripts/in_docker.sh scripts/build_and_test.sh '${target}' release"
