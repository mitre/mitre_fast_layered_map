#include <gtest/gtest.h>
#include "filters.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include <filters/filter_chain.h>

#define GTEST_COUT std::cerr << "[   INFO   ] "

TEST(RayTrace, active)
{
    EXPECT_EQ(0, 0);
}

/**
 * Test that error checking works for cases in 
 * which the ground and nonground layers don't exist.
 */
TEST(RayTrace, layers_dont_exist)
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

    EXPECT_FALSE(filterChain.update(gridMap, gridMap));

    // Add the non ground layer, still missing ground layer
    gridMap.add("nonground", 0);
    EXPECT_FALSE(filterChain.update(gridMap, gridMap));

    // Add ground layer, should now work
    gridMap.add("ground", 0);
    EXPECT_TRUE(filterChain.update(gridMap, gridMap));
}

/**
 * Say all squares are open
 * u = unknown
 * 0 = clear
 * 100 = occupied
 * Nonground
 * [c], [c], [c],
 * [c], [c], [c],
 * [c], [c], [c],
 * 
 * Ground
 * [u], [u], [u],
 * [u], [u], [u],
 * [u], [u], [u]
 * 
 */

TEST(RayTrace, fully_open)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"nonground"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);

    // Initialize ground as every cell having received one point
    gridMap.add("ground", 1);

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    // Check each cell to see if was ray traced to 0
    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);
        
        EXPECT_EQ(1, gridMap.at("ground", index));
        EXPECT_EQ(0, gridMap.at("nonground", index));
    }
}


/**
 * Say all squares are open
 * u = unknown
 * c = clear
 * o = occupied
 * 
 * Nonground
 * [c], [c], [c], [c], [c],
 * [c], [o], [o], [o], [c],
 * [c], [c], [c], [c], [c],
 * [c], [c], [c], [c], [c],
 * [c], [c], [c], [c], [c]
 * 
 * Ground
 * [u], [u], [u], [u], [u],
 * [u], [u], [u], [u], [u],
 * [u], [u], [u], [u], [u],
 * [u], [u], [u], [u], [u],
 * [u], [u], [u], [u], [u]
 * 
 * Expected Occupancy Output
 * [u], [u], [u], [u], [u],
 * [u], [o], [o], [o], [u],
 * [c], [c], [c], [c], [c],
 * [c], [c], [c], [c], [c],
 * [c], [c], [c], [c], [c]
 * 
 */

TEST(RayTrace, front_blocked)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"occupancy"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(5, 5), 1);

    // Initialize nonground
    gridMap.add("ground", 0);
    gridMap.add("nonground", 0);
    gridMap.at("nonground", grid_map::Index(1,1)) = 100;
    gridMap.at("nonground", grid_map::Index(1,2)) = 100;
    gridMap.at("nonground", grid_map::Index(1,3)) = 100;

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    /*
    Uncomment to see that the line iterator will hit the occupied cells (1,1) and (1, 3)
    before getting to (1, 0) and (1, 4), thus getting in the way of our "ray"
    */
    // for(grid_map::LineIterator it(gridMap, grid_map::Index(2, 2), grid_map::Index(1,0)); !it.isPastEnd(); ++it)
    // {
    //     grid_map::Index index(*it);
    //     GTEST_COUT << index(0) << ", " << index(1) << std::endl;
    // }

    gridMap["occupancy"] = gridMap["ground"] + gridMap["nonground"];

    Eigen::MatrixXi answerMat(5, 5);
    answerMat <<
    20,   20,  20, 20,  20,
    20, 100, 100, 100, 20,
    0,    0,   0,  0,   0,
    0,    0,   0,  0,   0,
    0,    0,   0,  0,   0;

    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        // Check that occupancy matches the expected answer
        EXPECT_EQ(answerMat(index(0), index(1)), gridMap.at("occupancy", index));
    }
}

/*************
 * Performs ray tracing operation on various size grids in order to ensure
 * the math is performed correctly. In the past we have had small errors
 * that caused cell calculations to be out of bounds.
 ************/
TEST(RayTrace, stress_test)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"occupancy"});
    gridMap.setFrameId("map");
    // Initialize nonground
    gridMap.add("ground", 0);
    gridMap.add("nonground", 0);

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");
    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    for (int i = 5; i < 100; i+=3)
    {
        for (int j = 1; j <= 10; j++)
        {
            gridMap.setGeometry(grid_map::Length(i, i), 0.1 * j);

            gridMap["ground"].setConstant(0);
            gridMap["nonground"].setConstant(0);

            if(!filterChain.update(gridMap, gridMap))
            {
                GTEST_FATAL_FAILURE_("Unable to update grid map.");
            }
        }
    }


}