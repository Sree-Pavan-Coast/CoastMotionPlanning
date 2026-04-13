#include <gtest/gtest.h>
#include <grid_map_core/grid_map_core.hpp>

class GridMapTest : public ::testing::Test {
protected:
    grid_map::GridMap map;

    void SetUp() override {
        // Create an empty map and set geometry. Length is in meters, resolution is m/cell.
        map.setGeometry(grid_map::Length(10.0, 10.0), 0.1);
        
        // Add a layer for testing
        map.add("obstacle_cost", 0.0);
    }
};

TEST_F(GridMapTest, Initialization) {
    // Check if the size of the map matches the geometry and resolution
    // For a 10x10 map with 0.1 resolution, it should have 100x100 cells
    EXPECT_EQ(map.getSize()(0), 100);
    EXPECT_EQ(map.getSize()(1), 100);
    
    // Check if the resolution was correctly stored
    EXPECT_DOUBLE_EQ(map.getResolution(), 0.1);
    
    // Check that the layer was successfully added
    EXPECT_TRUE(map.exists("obstacle_cost"));
}

TEST_F(GridMapTest, SetAndGetCellValues) {
    // Get the index for the center of the map [0.0, 0.0]
    grid_map::Position position(0.0, 0.0);
    grid_map::Index index;
    map.getIndex(position, index);
    
    // Set a cost value
    map.at("obstacle_cost", index) = 255.0; // Assume 255 is lethal obstacle
    
    // Verify the value was stored
    EXPECT_DOUBLE_EQ(map.atPosition("obstacle_cost", position), 255.0);
}

TEST_F(GridMapTest, IteratorUsage) {
    // Iterate over a sub-region to set a blockade (1m x 1m box in the middle)
    grid_map::Polygon polygon;
    polygon.setFrameId(map.getFrameId());
    polygon.addVertex(grid_map::Position(-0.5, -0.5));
    polygon.addVertex(grid_map::Position(-0.5, 0.5));
    polygon.addVertex(grid_map::Position(0.5, 0.5));
    polygon.addVertex(grid_map::Position(0.5, -0.5));

    for (grid_map::PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator) {
        map.at("obstacle_cost", *iterator) = 128.0;
    }
    
    // Verify a random point within this box is now 128.0
    EXPECT_DOUBLE_EQ(map.atPosition("obstacle_cost", grid_map::Position(0.2, 0.2)), 128.0);
    
    // Verify a point outside this box is still 0.0 (the default we started with)
    EXPECT_DOUBLE_EQ(map.atPosition("obstacle_cost", grid_map::Position(2.0, 2.0)), 0.0);
}
