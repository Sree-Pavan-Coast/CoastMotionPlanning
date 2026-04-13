#include <gtest/gtest.h>
#include <cmath>
#include <fstream>

#include "coastmotionplanning/costs/non_holonomic_heuristic.hpp"

using namespace coastmotionplanning::costs;

class NonHolonomicHeuristicTest : public ::testing::Test {
protected:
    NonHolonomicHeuristic lut;

    void SetUp() override {
        // Create a small test LUT
        lut.initGrid(8.0f, 72, 20, 0.5f);

        // Set the goal cell (center, theta=0) to 0
        int half = 10;
        lut.at(half, 0, 0) = 0.0f;

        // Set some known values for symmetry testing
        lut.at(half + 2, 3, 5) = 1.5f;   // (dx=1.0, dy=1.5, theta=5)
    }
};

TEST_F(NonHolonomicHeuristicTest, GoalLookupIsZero) {
    float cost = lut.lookup(0.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(cost, 0.0f);
}

TEST_F(NonHolonomicHeuristicTest, NonGoalIsPositive) {
    float cost = lut.lookup(5.0f, 0.0f, 0.0f);
    // Default init is LARGE_VALUE
    EXPECT_GT(cost, 0.0f);
}

TEST_F(NonHolonomicHeuristicTest, YSymmetryCorrectness) {
    // Set a value at y=1.5 (y_idx=3), theta=5
    int half = 10;
    lut.at(half + 2, 3, 5) = 42.0f;

    // Lookup at y=1.5 should return 42
    float cost_pos_y = lut.lookup(1.0f, 1.5f, 5 * static_cast<float>(2.0 * M_PI / 72));
    EXPECT_FLOAT_EQ(cost_pos_y, 42.0f);

    // Lookup at y=-1.5 should apply symmetry: L(x,-y,-θ)
    // So it looks up at y=1.5, theta = 72-5 = 67
    // We need to set that value
    lut.at(half + 2, 3, 67) = 99.0f;
    float cost_neg_y = lut.lookup(1.0f, -1.5f, 5 * static_cast<float>(2.0 * M_PI / 72));
    EXPECT_FLOAT_EQ(cost_neg_y, 99.0f);
}

TEST_F(NonHolonomicHeuristicTest, OutOfRangeReturnsLargeValue) {
    float cost = lut.lookup(1000.0f, 0.0f, 0.0f);
    EXPECT_GT(cost, 1e5f);
}

TEST_F(NonHolonomicHeuristicTest, FileRoundtrip) {
    std::string test_file = "test_nh_lut_roundtrip.bin";

    // Set some values
    int half = 10;
    lut.at(half, 0, 0) = 0.0f;
    lut.at(half + 1, 2, 10) = 3.14f;

    ASSERT_TRUE(lut.saveToFile(test_file));

    NonHolonomicHeuristic loaded;
    ASSERT_TRUE(loaded.loadFromFile(test_file));

    EXPECT_EQ(loaded.getMinTurningRadius(), lut.getMinTurningRadius());
    EXPECT_EQ(loaded.getNumAngleBins(), lut.getNumAngleBins());
    EXPECT_EQ(loaded.getGridSize(), lut.getGridSize());
    EXPECT_FLOAT_EQ(loaded.getCellSize(), lut.getCellSize());

    EXPECT_FLOAT_EQ(loaded.at(half, 0, 0), 0.0f);
    EXPECT_FLOAT_EQ(loaded.at(half + 1, 2, 10), 3.14f);

    // Cleanup
    std::remove(test_file.c_str());
}

TEST_F(NonHolonomicHeuristicTest, InvalidFileDetected) {
    NonHolonomicHeuristic loaded;
    EXPECT_FALSE(loaded.loadFromFile("nonexistent_file.bin"));
}

TEST_F(NonHolonomicHeuristicTest, CorruptedMagicDetected) {
    std::string test_file = "test_nh_corrupt.bin";

    // Write garbage
    std::ofstream f(test_file, std::ios::binary);
    uint32_t garbage = 0xDEADBEEF;
    f.write(reinterpret_cast<const char*>(&garbage), sizeof(garbage));
    f.close();

    NonHolonomicHeuristic loaded;
    EXPECT_FALSE(loaded.loadFromFile(test_file));

    std::remove(test_file.c_str());
}

TEST_F(NonHolonomicHeuristicTest, HitchPenaltyForTruckTrailer) {
    // Set a Reeds-Shepp cost
    int half = 10;
    lut.at(half + 2, 0, 0) = 5.0f;

    // With no hitch angle difference, should return RS cost
    float cost_no_hitch = lut.lookupWithHitchPenalty(
        1.0f, 0.0f, 0.0f,  // dx, dy, dtheta
        0.0f, 0.0f,          // hitch, goal_hitch
        2.0f);               // penalty factor
    EXPECT_FLOAT_EQ(cost_no_hitch, 5.0f);  // max(5.0, 0.0)

    // With large hitch angle difference, hitch penalty should dominate
    float cost_big_hitch = lut.lookupWithHitchPenalty(
        1.0f, 0.0f, 0.0f,
        1.5f, 0.0f,          // hitch=1.5 rad, goal=0
        10.0f);              // penalty = 10 * 1.5 = 15
    EXPECT_FLOAT_EQ(cost_big_hitch, 15.0f);  // max(5.0, 15.0)
}
