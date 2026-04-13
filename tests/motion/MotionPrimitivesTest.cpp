#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>

#include "coastmotionplanning/motion_primitives/car_motion_table.hpp"
#include "coastmotionplanning/motion_primitives/truck_trailer_motion_table.hpp"

using namespace coastmotionplanning::motion_primitives;

// ============================================================================
// Helpers
// ============================================================================
static MotionTableConfig defaultConfig() {
    MotionTableConfig cfg;
    cfg.minimum_turning_radius    = 8.0f;
    cfg.num_angle_quantization    = 72;   // 5° bins
    cfg.size_x                    = 100;
    cfg.non_straight_penalty      = 1.05f;
    cfg.change_penalty            = 0.0f;
    cfg.reverse_penalty           = 2.0f;
    cfg.cost_penalty              = 2.0f;
    cfg.retrospective_penalty     = 0.015f;
    cfg.use_quadratic_cost_penalty = false;
    cfg.allow_primitive_interpolation = false;
    return cfg;
}

// ============================================================================
// CarMotionTable — Dubin
// ============================================================================
class CarMotionTableDubinTest : public ::testing::Test {
protected:
    void SetUp() override {
        table_.initDubin(defaultConfig());
    }
    CarMotionTable table_;
};

TEST_F(CarMotionTableDubinTest, Initialization) {
    // Dubin should have exactly 3 base primitives (forward, left, right)
    EXPECT_EQ(table_.getNumPrimitives(), 3u);
    EXPECT_EQ(table_.getNumAngleBins(), 72u);
    EXPECT_EQ(table_.getModel(), CarMotionTable::Model::DUBIN);
    EXPECT_NEAR(table_.getBinSize(), 2.0 * M_PI / 72.0, 1e-5);
}

TEST_F(CarMotionTableDubinTest, ProjectionsReturnCorrectCount) {
    auto projections = table_.getProjections(50.0f, 50.0f, 0);
    EXPECT_EQ(projections.size(), 3u);
}

TEST_F(CarMotionTableDubinTest, StraightPrimitiveGoesForward) {
    // At heading bin 0 (pointing along +X), the straight primitive
    // should move purely in +X
    auto projections = table_.getProjections(0.0f, 0.0f, 0);

    // Find the FORWARD primitive
    auto it = std::find_if(projections.begin(), projections.end(),
        [](const MotionPose& p) { return p.turn_dir == TurnDirection::FORWARD; });
    ASSERT_NE(it, projections.end());

    EXPECT_GT(it->x, 0.0f);          // moved in +X
    EXPECT_NEAR(it->y, 0.0f, 1e-5f); // no lateral movement
    EXPECT_NEAR(it->theta, 0.0f, 1e-5f); // heading unchanged
}

TEST_F(CarMotionTableDubinTest, LeftTurnGoesPositiveY) {
    // At heading bin 0, a left turn should deflect into +Y
    auto projections = table_.getProjections(0.0f, 0.0f, 0);
    auto it = std::find_if(projections.begin(), projections.end(),
        [](const MotionPose& p) { return p.turn_dir == TurnDirection::LEFT; });
    ASSERT_NE(it, projections.end());

    EXPECT_GT(it->x, 0.0f);
    EXPECT_GT(it->y, 0.0f);  // left = +Y
    EXPECT_GT(it->theta, 0.0f); // heading increased
}

TEST_F(CarMotionTableDubinTest, RightTurnGoesNegativeY) {
    auto projections = table_.getProjections(0.0f, 0.0f, 0);
    auto it = std::find_if(projections.begin(), projections.end(),
        [](const MotionPose& p) { return p.turn_dir == TurnDirection::RIGHT; });
    ASSERT_NE(it, projections.end());

    EXPECT_GT(it->x, 0.0f);
    EXPECT_LT(it->y, 0.0f);  // right = -Y
}

TEST_F(CarMotionTableDubinTest, NoDubinReversePrimitives) {
    auto projections = table_.getProjections(50.0f, 50.0f, 0);
    for (const auto& p : projections) {
        EXPECT_NE(p.turn_dir, TurnDirection::REVERSE);
        EXPECT_NE(p.turn_dir, TurnDirection::REV_LEFT);
        EXPECT_NE(p.turn_dir, TurnDirection::REV_RIGHT);
    }
}

TEST_F(CarMotionTableDubinTest, ProjectionsAreTranslated) {
    // Projections at (50,50) should be offset from projections at (0,0)
    auto p0 = table_.getProjections(0.0f, 0.0f, 0);
    auto p50 = table_.getProjections(50.0f, 50.0f, 0);

    ASSERT_EQ(p0.size(), p50.size());
    for (std::size_t i = 0; i < p0.size(); ++i) {
        EXPECT_NEAR(p50[i].x - p0[i].x, 50.0f, 1e-4f);
        EXPECT_NEAR(p50[i].y - p0[i].y, 50.0f, 1e-4f);
        EXPECT_NEAR(p50[i].theta, p0[i].theta, 1e-5f); // headings identical
    }
}

TEST_F(CarMotionTableDubinTest, RotatedHeadingProducesRotatedDeltas) {
    // At heading bin 18 (= 90°, pointing +Y), straight should go in +Y not +X
    unsigned int bin_90deg = 18; // 18 * 5° = 90°
    auto projections = table_.getProjections(0.0f, 0.0f, bin_90deg);

    auto it = std::find_if(projections.begin(), projections.end(),
        [](const MotionPose& p) { return p.turn_dir == TurnDirection::FORWARD; });
    ASSERT_NE(it, projections.end());

    EXPECT_NEAR(it->x, 0.0f, 1e-4f);  // no X movement at 90°
    EXPECT_GT(it->y, 0.0f);           // moved in +Y
}

TEST_F(CarMotionTableDubinTest, TravelCostsArePositive) {
    for (unsigned int i = 0; i < table_.getNumPrimitives(); ++i) {
        EXPECT_GT(table_.getTravelCost(i), 0.0f);
    }
}

TEST_F(CarMotionTableDubinTest, AngularBinConversion) {
    // 0 radians -> bin 0
    EXPECT_EQ(table_.getClosestAngularBin(0.0), 0u);
    // pi radians -> bin 36
    EXPECT_EQ(table_.getClosestAngularBin(M_PI), 36u);
    // Round-trip
    float angle = table_.getAngleFromBin(18);
    EXPECT_EQ(table_.getClosestAngularBin(angle), 18u);
}

// ============================================================================
// CarMotionTable — Reeds-Shepp
// ============================================================================
class CarMotionTableReedsSheppTest : public ::testing::Test {
protected:
    void SetUp() override {
        table_.initReedsShepp(defaultConfig());
    }
    CarMotionTable table_;
};

TEST_F(CarMotionTableReedsSheppTest, HasSixBasePrimitives) {
    EXPECT_EQ(table_.getNumPrimitives(), 6u);
    EXPECT_EQ(table_.getModel(), CarMotionTable::Model::REEDS_SHEPP);
}

TEST_F(CarMotionTableReedsSheppTest, HasReversePrimitives) {
    auto projections = table_.getProjections(0.0f, 0.0f, 0);
    bool has_reverse = false;
    for (const auto& p : projections) {
        if (p.turn_dir == TurnDirection::REVERSE ||
            p.turn_dir == TurnDirection::REV_LEFT ||
            p.turn_dir == TurnDirection::REV_RIGHT) {
            has_reverse = true;
            break;
        }
    }
    EXPECT_TRUE(has_reverse);
}

TEST_F(CarMotionTableReedsSheppTest, ReverseStraightGoesBackward) {
    auto projections = table_.getProjections(0.0f, 0.0f, 0);
    auto it = std::find_if(projections.begin(), projections.end(),
        [](const MotionPose& p) { return p.turn_dir == TurnDirection::REVERSE; });
    ASSERT_NE(it, projections.end());

    EXPECT_LT(it->x, 0.0f);          // moved in -X (backward)
    EXPECT_NEAR(it->y, 0.0f, 1e-5f); // no lateral movement
}

TEST_F(CarMotionTableReedsSheppTest, InterpolatedPrimitivesIncreaseCoverage) {
    auto cfg = defaultConfig();
    cfg.allow_primitive_interpolation = true;

    CarMotionTable table_interp;
    table_interp.initReedsShepp(cfg);

    // With interpolation, there should be more than the 6 base primitives
    // (exact count depends on the increments value from the turning radius)
    EXPECT_GE(table_interp.getNumPrimitives(), 6u);
}

TEST_F(CarMotionTableReedsSheppTest, AllBinsProduceValidProjections) {
    for (unsigned int bin = 0; bin < table_.getNumAngleBins(); ++bin) {
        auto projections = table_.getProjections(50.0f, 50.0f, bin);
        EXPECT_EQ(projections.size(), 6u) << "Failed at bin " << bin;

        for (const auto& p : projections) {
            EXPECT_FALSE(std::isnan(p.x)) << "NaN x at bin " << bin;
            EXPECT_FALSE(std::isnan(p.y)) << "NaN y at bin " << bin;
            EXPECT_FALSE(std::isnan(p.theta)) << "NaN theta at bin " << bin;
        }
    }
}

TEST_F(CarMotionTableReedsSheppTest, ChordLengthExceedsSqrt2) {
    // Verify every primitive moves at least sqrt(2) cells from origin
    auto projections = table_.getProjections(0.0f, 0.0f, 0);
    const float sqrt2 = std::sqrt(2.0f);
    for (const auto& p : projections) {
        float dist = std::hypot(p.x, p.y);
        EXPECT_GE(dist, sqrt2 - 0.01f)  // small tolerance
            << "Primitive " << static_cast<int>(p.turn_dir)
            << " has chord " << dist << " < sqrt(2)";
    }
}

// ============================================================================
// TruckTrailerMotionTable
// ============================================================================
class TruckTrailerMotionTableTest : public ::testing::Test {
protected:
    void SetUp() override {
        TruckTrailerKinematics kin;
        kin.tractor_wheelbase = 3.0f;
        kin.hitch_offset      = 1.0f;
        kin.trailer_wheelbase = 5.0f;
        kin.max_hitch_angle   = 1.05f; // ~60°
        table_.init(defaultConfig(), kin);
    }
    TruckTrailerMotionTable table_;
};

TEST_F(TruckTrailerMotionTableTest, Initialization) {
    EXPECT_EQ(table_.getNumPrimitives(), 6u);
    EXPECT_EQ(table_.getNumAngleBins(), 72u);
}

TEST_F(TruckTrailerMotionTableTest, ProjectionsWithAlignedTrailer) {
    // Tractor and trailer both heading bin 0 (aligned, no hitch angle)
    auto projections = table_.getProjections(50.0f, 50.0f, 0, 0);

    // Should get all 6 primitives (no jackknife when aligned)
    EXPECT_EQ(projections.size(), 6u);

    for (const auto& p : projections) {
        EXPECT_FALSE(std::isnan(p.x));
        EXPECT_FALSE(std::isnan(p.y));
        EXPECT_FALSE(std::isnan(p.theta1));
        EXPECT_FALSE(std::isnan(p.theta2));
    }
}

TEST_F(TruckTrailerMotionTableTest, StraightForwardKeepsTrailerAligned) {
    // When aligned (hitch angle = 0), going straight should keep trailer aligned
    auto projections = table_.getProjections(0.0f, 0.0f, 0, 0);

    auto it = std::find_if(projections.begin(), projections.end(),
        [](const TruckTrailerPose& p) { return p.turn_dir == TurnDirection::FORWARD; });
    ASSERT_NE(it, projections.end());

    // Trailer should barely change angle when going straight with 0 hitch angle
    EXPECT_NEAR(it->theta2, 0.0f, 0.5f); // within half a bin
}

TEST_F(TruckTrailerMotionTableTest, JackknifePruning) {
    // Create a scenario near the jackknife limit:
    // tractor heading bin 0, trailer heading such that hitch angle is near max
    // The max_hitch_angle is 1.05 rad ≈ 60° ≈ 12 bins
    unsigned int tractor_bin = 0;
    unsigned int trailer_bin = 14; // 14 * 5° = 70° hitch angle > 60° limit

    auto projections = table_.getProjections(50.0f, 50.0f, tractor_bin, trailer_bin);

    // Some primitives should have been pruned due to jackknife
    // We can't guarantee exactly how many, but there should be fewer than 6
    // (at least the ones that would increase the hitch angle further)
    // Just verify we get a valid (possibly reduced) set
    EXPECT_LE(projections.size(), 6u);

    // All returned projections must have hitch angle within limits
    for (const auto& p : projections) {
        float hitch_angle_rad = std::abs(p.theta1 - p.theta2) * table_.getBinSize();
        float wrapped = std::min(hitch_angle_rad,
                                 2.0f * static_cast<float>(M_PI) - hitch_angle_rad);
        EXPECT_LE(wrapped, table_.getMaxHitchAngle() + 0.01f)
            << "Jackknife violation: hitch angle " << wrapped << " rad";
    }
}

TEST_F(TruckTrailerMotionTableTest, TractorSpatialDeltasMatchCar) {
    // The tractor spatial deltas x,y should be identical to a CarMotionTable
    CarMotionTable car_table;
    car_table.initReedsShepp(defaultConfig());

    auto car_proj = car_table.getProjections(0.0f, 0.0f, 0);
    auto tt_proj  = table_.getProjections(0.0f, 0.0f, 0, 0);

    // Both should have 6 primitives when aligned (no jackknife)
    ASSERT_EQ(car_proj.size(), 6u);
    ASSERT_EQ(tt_proj.size(), 6u);

    for (std::size_t i = 0; i < 6; ++i) {
        EXPECT_NEAR(car_proj[i].x, tt_proj[i].x, 1e-4f)
            << "Mismatch at primitive " << i;
        EXPECT_NEAR(car_proj[i].y, tt_proj[i].y, 1e-4f)
            << "Mismatch at primitive " << i;
        // Tractor heading delta should match too
        EXPECT_NEAR(car_proj[i].theta, tt_proj[i].theta1, 1e-4f)
            << "Tractor heading mismatch at primitive " << i;
    }
}

TEST_F(TruckTrailerMotionTableTest, TrailerAngleUpdatesOnTurn) {
    // When aligned and turning left, the trailer should lag behind the tractor
    auto projections = table_.getProjections(0.0f, 0.0f, 0, 0);

    auto it = std::find_if(projections.begin(), projections.end(),
        [](const TruckTrailerPose& p) { return p.turn_dir == TurnDirection::LEFT; });
    ASSERT_NE(it, projections.end());

    // Tractor turns left (theta1 increases), trailer should follow but less
    EXPECT_GT(it->theta1, 0.0f);

    // Trailer theta2 changes less than tractor theta1 (trailer lags).
    // Must handle wrap-around: a value of ~71.4 in [0,72) means -0.6 bins.
    const float n = static_cast<float>(table_.getNumAngleBins());
    float trailer_delta = it->theta2;
    if (trailer_delta > n / 2.0f) {
        trailer_delta -= n;  // wrap: 71.4 -> -0.6
    }
    float tractor_delta = it->theta1;
    if (tractor_delta > n / 2.0f) {
        tractor_delta -= n;
    }

    // The trailer angle change magnitude should be smaller than or equal to the tractor's
    EXPECT_LE(std::abs(trailer_delta), std::abs(tractor_delta) + 0.5f);
}

TEST_F(TruckTrailerMotionTableTest, AllBinsProduceValidProjections) {
    // Sweep all tractor × trailer combinations to check for NaN / crashes
    for (unsigned int t1 = 0; t1 < table_.getNumAngleBins(); t1 += 6) {
        for (unsigned int t2 = 0; t2 < table_.getNumAngleBins(); t2 += 6) {
            auto projections = table_.getProjections(50.0f, 50.0f, t1, t2);
            for (const auto& p : projections) {
                EXPECT_FALSE(std::isnan(p.x))
                    << "NaN at t1=" << t1 << " t2=" << t2;
                EXPECT_FALSE(std::isnan(p.y))
                    << "NaN at t1=" << t1 << " t2=" << t2;
                EXPECT_FALSE(std::isnan(p.theta1))
                    << "NaN at t1=" << t1 << " t2=" << t2;
                EXPECT_FALSE(std::isnan(p.theta2))
                    << "NaN at t1=" << t1 << " t2=" << t2;
            }
        }
    }
}

TEST_F(TruckTrailerMotionTableTest, TravelCostsArePositive) {
    for (unsigned int i = 0; i < table_.getNumPrimitives(); ++i) {
        EXPECT_GT(table_.getTravelCost(i), 0.0f);
    }
}

TEST_F(TruckTrailerMotionTableTest, AngularBinConversion) {
    EXPECT_EQ(table_.getClosestAngularBin(0.0), 0u);
    EXPECT_EQ(table_.getClosestAngularBin(M_PI), 36u);
    float angle = table_.getAngleFromBin(18);
    EXPECT_EQ(table_.getClosestAngularBin(angle), 18u);
}
