# JunctionPerception

Digital twin of a signalized intersection from the video of a single fixed camera: road users are
detected, tracked and placed on the ground plane in metric map coordinates, with a focus on
vulnerable road users.

## Build

Everything builds inside a Docker development image (GCC 14, CMake, Ninja, vcpkg, Node.js), so the
host only needs Docker.

```bash
scripts/setup_server.sh                                   # build the development image
scripts/in_docker.sh scripts/build_and_test.sh            # C++ and frontend: build, lint, test
scripts/in_docker.sh scripts/build_and_test.sh cpp debug  # C++ only, debug preset
```

C++ dependencies come from vcpkg in manifest mode (`vcpkg.json`); the CMake presets are `debug` and
`release`. Dependency caches are kept in `.cache/`.

## Repository layout

| Path | Content |
|---|---|
| `include/jp/`, `src/` | C++ modules (one folder per module) and the command-line apps in `src/apps/` |
| `tests/` | C++ unit tests (GoogleTest) |
| `web/` | Frontend (TypeScript, Vite, Three.js) |
| `docker/` | Development image |
| `scripts/` | Image setup, build-and-test and deployment scripts |
