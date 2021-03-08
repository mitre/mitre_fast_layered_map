#include <gtest/gtest.h>
#include <ros/ros.h>
#include <filters/filter_chain.h>
#include "filters.hpp"

TEST(OutlierRemoval, active)
{
    EXPECT_EQ(0, 0);
}

TEST(OutlierRemoval, layer_doesnt_exist)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"random_layer"});
    gridMap.setFrameId("random");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    grid_map::GridMap outMap;
    EXPECT_FALSE(filterChain.update(gridMap, outMap));
}

/**
 * Remove an outlier in a 3x3 grid
 * [ ], [ ], [ ],
 * [ ], [x], [ ],
 * [ ], [ ], [ ]
 */
TEST(OutlierRemoval, remove_middle_one)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"map"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);
    
    // Prefilled point
    gridMap.at("map", grid_map::Index(1, 1)) = 100;

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }
    
    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    EXPECT_EQ(0, gridMap.at("map", grid_map::Index(1, 1)));
}

/**
 * Remove perimeter outliers in a 3x3 grid
 * [ ], [ ], [x],
 * [x], [ ], [ ],
 * [ ], [ ], [ ]
 */
TEST(OutlierRemoval, remove_perimeter_one)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"map"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);

    // Prefilled points
    gridMap.at("map", grid_map::Index(1, 0)) = 100;
    gridMap.at("map", grid_map::Index(0, 2)) = 100;

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }
    
    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    EXPECT_EQ(0, gridMap.at("map", grid_map::Index(1, 0)));
    EXPECT_EQ(0, gridMap.at("map", grid_map::Index(0, 2)));
}

/**
 * Don't remove a point that has a neighbor in a 3x3 grid
 * Point could be in any cell
 * [n], [n], [n],
 * [n], [x], [n],
 * [n], [n], [n]
 */
TEST(OutlierRemoval, preserve_points)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"map"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);
    gridMap.at("map", grid_map::Index(1, 1)) = 100; // Prefilled center point

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        // Skip over prefilled center point
        if(index(0) == 1 && index(1) == 1)
        {
            continue;
        }

        // Set cell with index to 100
        gridMap.at("map", index) = 100;

        if(!filterChain.update(gridMap, gridMap))
        {
            GTEST_FATAL_FAILURE_("Unable to update grid map.");
        }

        // Expect both center point and earlier set point to still be 100
        EXPECT_EQ(100, gridMap.at("map", grid_map::Index(1, 1)));
        EXPECT_EQ(100, gridMap.at("map", index));

        // Set index to 0 so it doesn't affect next test
        gridMap.at("map", index) = 0;
    }

}


/**
 * Larger example to that should preserve some and remove others
 * p = point that should be preserved
 * r = point that should be removed
 * [p], [ ], [ ], [ ], [ ],
 * [ ], [p], [ ], [r], [ ],
 * [ ], [p], [ ], [ ], [ ],
 * [ ], [p], [ ], [ ], [ ],
 * [ ], [ ], [ ], [ ], [r]
 */
TEST(OutlierRemoval, large_case)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"map"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(5, 5), 1);

    // Prefilled points
    gridMap.at("map", grid_map::Index(0, 0)) = 100;
    gridMap.at("map", grid_map::Index(1, 1)) = 100;
    gridMap.at("map", grid_map::Index(2, 1)) = 100;
    gridMap.at("map", grid_map::Index(3, 1)) = 100;
    gridMap.at("map", grid_map::Index(1, 3)) = 100;
    gridMap.at("map", grid_map::Index(4, 4)) = 100;

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }
    
    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    EXPECT_EQ(100, gridMap.at("map", grid_map::Index(0, 0)));
    EXPECT_EQ(100, gridMap.at("map", grid_map::Index(1, 1)));
    EXPECT_EQ(100, gridMap.at("map", grid_map::Index(2, 1)));
    EXPECT_EQ(100, gridMap.at("map", grid_map::Index(3, 1)));
    EXPECT_EQ(0, gridMap.at("map", grid_map::Index(1, 3)));
    EXPECT_EQ(0, gridMap.at("map", grid_map::Index(4, 4)));   
}
