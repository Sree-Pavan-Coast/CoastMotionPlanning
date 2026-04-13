#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include "coastmotionplanning/costs/dual_model_non_holonomic_heuristic.hpp"
#include "coastmotionplanning/costs/non_holonomic_heuristic.hpp"

namespace {

using coastmotionplanning::costs::DualModelNonHolonomicHeuristic;
using coastmotionplanning::costs::HeuristicModel;
using coastmotionplanning::costs::NHLutHeader;

TEST(DualModelNonHolonomicHeuristicTest, FileRoundtripPreservesBothModels) {
    DualModelNonHolonomicHeuristic lut;
    lut.initGrid(8.0f, 72, 20, 0.5f);

    const int half_grid = 10;
    lut.at(HeuristicModel::DUBINS, half_grid + 1, 2, 10) = 3.14f;
    lut.at(HeuristicModel::REEDS_SHEPP, half_grid + 1, 2, 10) = 2.71f;

    const std::string test_file = "test_dual_nh_lut_roundtrip.bin";
    ASSERT_TRUE(lut.saveToFile(test_file));

    DualModelNonHolonomicHeuristic loaded;
    ASSERT_TRUE(loaded.loadFromFile(test_file));

    EXPECT_FLOAT_EQ(loaded.getMinTurningRadius(), 8.0f);
    EXPECT_EQ(loaded.getNumAngleBins(), 72u);
    EXPECT_EQ(loaded.getGridSize(), 20u);
    EXPECT_FLOAT_EQ(loaded.getCellSize(), 0.5f);
    EXPECT_FLOAT_EQ(
        loaded.at(HeuristicModel::DUBINS, half_grid + 1, 2, 10), 3.14f);
    EXPECT_FLOAT_EQ(
        loaded.at(HeuristicModel::REEDS_SHEPP, half_grid + 1, 2, 10), 2.71f);

    std::remove(test_file.c_str());
}

TEST(DualModelNonHolonomicHeuristicTest, RejectsLegacySingleModelFiles) {
    const std::string legacy_file = "test_legacy_nh_lut.bin";

    NHLutHeader legacy_header;
    legacy_header.min_turning_radius = 8.0f;
    legacy_header.num_angle_bins = 72;
    legacy_header.grid_size = 20;
    legacy_header.cell_size = 0.5f;
    legacy_header.stored_y_size = 11;

    std::ofstream stream(legacy_file, std::ios::binary);
    ASSERT_TRUE(stream.is_open());
    stream.write(reinterpret_cast<const char*>(&legacy_header), sizeof(NHLutHeader));
    const float dummy_value = 0.0f;
    stream.write(reinterpret_cast<const char*>(&dummy_value), sizeof(float));
    stream.close();

    DualModelNonHolonomicHeuristic loaded;
    EXPECT_FALSE(loaded.loadFromFile(legacy_file));

    std::remove(legacy_file.c_str());
}

TEST(DualModelNonHolonomicHeuristicTest, OmplFallbackReturnsFiniteValues) {
    DualModelNonHolonomicHeuristic heuristic;
    heuristic.configureOmpl(8.0f);

    const float dubins = heuristic.lookup(HeuristicModel::DUBINS, 5.0f, 1.0f, 0.2f);
    const float reeds_shepp = heuristic.lookup(HeuristicModel::REEDS_SHEPP, 5.0f, 1.0f, 0.2f);

    EXPECT_TRUE(std::isfinite(dubins));
    EXPECT_TRUE(std::isfinite(reeds_shepp));
    EXPECT_GT(dubins, 0.0f);
    EXPECT_GT(reeds_shepp, 0.0f);
}

TEST(DualModelNonHolonomicHeuristicTest, ReedsSheppCostIsNotGreaterThanDubins) {
    DualModelNonHolonomicHeuristic heuristic;
    heuristic.configureOmpl(8.0f);

    const float dubins = heuristic.lookup(HeuristicModel::DUBINS, 6.0f, 2.0f, 0.5f);
    const float reeds_shepp = heuristic.lookup(HeuristicModel::REEDS_SHEPP, 6.0f, 2.0f, 0.5f);

    EXPECT_LE(reeds_shepp, dubins + 1e-4f);
}

} // namespace
