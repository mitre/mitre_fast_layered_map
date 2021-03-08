#include <gtest/gtest.h>
#include "filters.hpp"
#include <filters/filter_chain.h>
#include <math.h>
#include <Eigen/Dense>

#define GTEST_COUT std::cerr << "[   INFO   ] "

TEST(ThresholdFilter, active)
{
    EXPECT_EQ(0, 0);
}

/**
 * Test that error checking works for cases in 
 * which the layers don't exist.
 */
TEST(ThresholdFilter, layers_dont_exist)
{
    ros::NodeHandle nh("~");

    grid_map::GridMap gridMap({"random_layer"});
    gridMap.setFrameId("random");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if (!filterChain.configure("mitre_fast_layered_map_filters_threshold", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    EXPECT_FALSE(filterChain.update(gridMap, gridMap));

    // Bayes filter needs current_probability and obstacle_detected layers
    gridMap.add("current_probability", 0);
    EXPECT_FALSE(filterChain.update(gridMap, gridMap));

    gridMap.add("nonground", 0);
    EXPECT_TRUE(filterChain.update(gridMap, gridMap));

}

// /***
//  * All of the tests below this point are just checking that the math
//  * comes out as expected. You can use this to refer to the expected performance
//  * given the settings.
//  */

// /**
//  * Ensure its outputting expected values when a reading is given as 1 or 0
//  */
TEST(BayesUpdate, binary)
{
    ros::NodeHandle nh("~");

    // NOTE: Current probability will be filled with nans, but the filter should take care of it
    grid_map::GridMap gridMap({"current_probability", "nonground"});
    gridMap.setFrameId("random");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if (!filterChain.configure("mitre_fast_layered_map_filters_threshold", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    // Initialize obstacle detected, only two cells have detection in first frame
    gridMap["current_probability"] <<
    0, 0.1, 0.2,
    0.3, 0.4, 0.5,
    0.6, 0.7, 0.8;

    
    // Create matrix of expected output to compare to
    Eigen::MatrixXf answerMat(3, 3);
    answerMat << 0, 0, 0,
        0, 100, 100,
        100, 100, 100;

    if (!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_FLOAT_EQ(answerMat(index(0), index(1)), gridMap.at("nonground", index)) << "Index: " << index(0) << " " << index(1);
    }

}