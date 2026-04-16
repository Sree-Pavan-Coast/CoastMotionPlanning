#include "coastmotionplanning/geometry/shape_types.hpp"

#include <gtest/gtest.h>

namespace coastmotionplanning {
namespace geometry {
namespace {

TEST(ShapeTypesTest, ArePointsCloseReturnsTrueForSamePoint) {
    const Point2d a(1.0, 2.0);
    const Point2d b(1.0, 2.0);

    EXPECT_TRUE(arePointsClose(a, b));
}

TEST(ShapeTypesTest, ArePointsCloseUsesDefaultTolerance) {
    const Point2d a(1.0, 2.0);
    const Point2d b(1.0 + (0.5 * common::EPSILON), 2.0);

    EXPECT_TRUE(arePointsClose(a, b));
}

TEST(ShapeTypesTest, ArePointsCloseRejectsPointsOutsideTolerance) {
    const Point2d a(1.0, 2.0);
    const Point2d b(1.0 + (2.0 * common::EPSILON), 2.0);

    EXPECT_FALSE(arePointsClose(a, b));
}

TEST(ShapeTypesTest, ArePointsCloseHonorsCustomTolerance) {
    const Point2d a(0.0, 0.0);
    const Point2d b(0.0, 0.002);

    EXPECT_TRUE(arePointsClose(a, b, 0.01));
    EXPECT_FALSE(arePointsClose(a, b, 0.001));
}

} // namespace
} // namespace geometry
} // namespace coastmotionplanning
