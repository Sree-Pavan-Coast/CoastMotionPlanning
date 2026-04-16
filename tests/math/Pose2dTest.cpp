#include "coastmotionplanning/math/pose2d.hpp"

#include <gtest/gtest.h>

namespace coastmotionplanning {
namespace math {
namespace {

using common::MotionDirection;

Pose2d makePose(double x, double y, double heading_deg) {
    return Pose2d{x, y, Angle::from_degrees(heading_deg)};
}

TEST(Pose2dTest, InferMotionDirectionToReturnsForwardForAlignedTravel) {
    const Pose2d start = makePose(0.0, 0.0, 0.0);
    const Pose2d goal = makePose(2.0, 0.0, 0.0);

    EXPECT_EQ(start.inferMotionDirectionTo(goal), MotionDirection::Forward);
}

TEST(Pose2dTest, InferMotionDirectionToReturnsReverseForOpposedTravel) {
    const Pose2d start = makePose(0.0, 0.0, 0.0);
    const Pose2d goal = makePose(-2.0, 0.0, 0.0);

    EXPECT_EQ(start.inferMotionDirectionTo(goal), MotionDirection::Reverse);
}

TEST(Pose2dTest, InferMotionDirectionToUsesFallbackForNearZeroTravel) {
    const Pose2d start = makePose(1.0, 1.0, 0.0);
    const Pose2d goal = makePose(1.0 + 1e-9, 1.0, 180.0);

    EXPECT_EQ(
        start.inferMotionDirectionTo(goal, MotionDirection::Reverse),
        MotionDirection::Reverse);
}

TEST(Pose2dTest, InferMotionDirectionToUsesFallbackForAmbiguousTravel) {
    const Pose2d start = makePose(0.0, 0.0, 0.0);
    const Pose2d goal = makePose(0.0, 2.0, 0.0);

    EXPECT_EQ(
        start.inferMotionDirectionTo(goal, MotionDirection::Reverse),
        MotionDirection::Reverse);
}

} // namespace
} // namespace math
} // namespace coastmotionplanning
