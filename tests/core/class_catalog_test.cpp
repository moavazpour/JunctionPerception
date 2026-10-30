#include "jp/core/class_catalog.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace {

constexpr std::string_view kCatalog = R"yaml(
classes:
  car:        {length_m: 4.5, width_m: 1.8, vru: false}
  pedestrian: {length_m: 0.5, width_m: 0.5, vru: true}
sources:
  coco: {car: car, person: pedestrian}
  tumtraf: {CAR: car, PEDESTRIAN: pedestrian}
)yaml";

TEST(ClassCatalog, MapsSourceLabelsToClasses) {
  const auto catalog = jp::core::ClassCatalog::from_yaml(kCatalog);
  ASSERT_EQ(catalog.classes().size(), 2U);
  EXPECT_EQ(catalog.map("coco", "person"), "pedestrian");
  EXPECT_EQ(catalog.map("tumtraf", "CAR"), "car");
  EXPECT_FALSE(catalog.map("coco", "dog").has_value());
  EXPECT_FALSE(catalog.map("kitti", "car").has_value());
  EXPECT_TRUE(catalog.info("pedestrian").vru);
  EXPECT_DOUBLE_EQ(catalog.info("car").length_m, 4.5);
  EXPECT_THROW(static_cast<void>(catalog.info("tram")), std::out_of_range);
}

TEST(ClassCatalog, RejectsMappingToUnknownClass) {
  constexpr std::string_view bad = R"yaml(
classes:
  car: {length_m: 4.5, width_m: 1.8, vru: false}
sources:
  coco: {bus: bus}
)yaml";
  EXPECT_THROW(static_cast<void>(jp::core::ClassCatalog::from_yaml(bad)), std::runtime_error);
}

TEST(ClassCatalog, RepositoryConfigIsValid) {
  const auto catalog = jp::core::ClassCatalog::load(JP_SOURCE_DIR "/configs/classes.yaml");
  for (const char* label : {"CAR", "VAN", "TRUCK", "TRAILER", "BUS", "EMERGENCY_VEHICLE",
                            "MOTORCYCLE", "BICYCLE", "PEDESTRIAN", "OTHER"}) {
    EXPECT_TRUE(catalog.map("tumtraf", label).has_value()) << label;
  }
  for (const char* label : {"car", "truck", "bus", "motorcycle", "bicycle", "person"}) {
    EXPECT_TRUE(catalog.map("coco", label).has_value()) << label;
  }
}

}  // namespace
