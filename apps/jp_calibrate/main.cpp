// Offline manual calibration: solves the inputs the web panel stored on the device and prints the report.
//
//   jp_calibrate --inputs state/<site>/calibration_inputs.json --size 1920x1080 --at 48.17599,11.55559
//                [--lens phone_wide] [--height 7] [--height-sigma 1.5] [--anchor-sigma 30]
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include "jp/calib/manual_calibration.hpp"

int main(int argc, char** argv) {
  jp::ManualCalibrationInput in;
  std::string path;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i], v = argv[i + 1];
    if (k == "--inputs") path = v;
    else if (k == "--size") std::sscanf(v.c_str(), "%dx%d", &in.width, &in.height);
    else if (k == "--at") std::sscanf(v.c_str(), "%lf,%lf", &in.anchor.lat, &in.anchor.lon), in.anchor.valid = true;
    else if (k == "--lens") in.lens = v;
    else if (k == "--height") in.height_m = std::stod(v);
    else if (k == "--height-sigma") in.height_sigma = std::stod(v);
    else if (k == "--anchor-sigma") in.anchor_sigma_m = std::stod(v);
    else if (k == "--map-sigma") in.map_sigma_m = std::stod(v);
  }
  if (path.empty() || !in.width || !in.anchor.valid) {
    std::cerr << "usage: jp_calibrate --inputs F --size WxH --at LAT,LON [--lens L] [--height H] [--anchor-sigma M]\n";
    return 2;
  }
  std::ifstream f(path);
  const auto j = nlohmann::json::parse(f);
  bool lens_set = false, h_set = false;
  for (int i = 1; i + 1 < argc; i += 2) lens_set |= !std::strcmp(argv[i], "--lens"), h_set |= !std::strcmp(argv[i], "--height");
  if (!lens_set) in.lens = j.value("lens", in.lens);
  if (!h_set && j.contains("height_m") && j["height_m"].is_number()) in.height_m = j["height_m"].get<double>();
  for (const auto& p : j.at("points")) in.points.push_back({{p["u"].get<double>(), p["v"].get<double>()}, p["lat"].get<double>(), p["lon"].get<double>()});
  for (const auto& l : j.value("lines", nlohmann::json::array())) {
    jp::LineCorrespondence lc;
    for (const auto& q : l["px"]) lc.px.emplace_back(q[0].get<double>(), q[1].get<double>());
    lc.geo = {Eigen::Vector2d(l["geo"][0][0].get<double>(), l["geo"][0][1].get<double>()),
              Eigen::Vector2d(l["geo"][1][0].get<double>(), l["geo"][1][1].get<double>())};
    in.lines.push_back(lc);
  }
  const auto r = jp::calibrateManual(in);
  std::printf("%s [%s]\n", r.message.c_str(), r.quality.c_str());
  for (const auto& w : r.warnings) std::printf("  warning: %s\n", w.c_str());
  for (const auto& e : r.residuals) std::printf("  %s %d: %.2f m, %.1f px\n", e.kind.c_str(), e.index + 1, e.m, e.px);
  for (const auto& a : r.accuracy) std::printf("  accuracy at %.0f m: +-%.2f m\n", a.range_m, a.sigma_m);
  return r.ok ? 0 : 1;
}
