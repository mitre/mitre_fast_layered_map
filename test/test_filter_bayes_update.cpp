#include <gtest/gtest.h>
#include "filters.hpp"
#include <filters/filter_chain.h>
#include <math.h>
#include <Eigen/Dense>

#define GTEST_COUT std::cerr << "[   INFO   ] "

TEST(BayesUpdate, active)
{
    EXPECT_EQ(0, 0);
}

/**
 * Test that error checking works for cases in 
 * which the 
 */
TEST(BayesUpdate, layers_dont_exist)
{
    ros::NodeHandle nh("~");

    grid_map::GridMap gridMap({"random_layer"});
    gridMap.setFrameId("random");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if (!filterChain.configure("mitre_fast_layered_map_filters_bayes_update", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    EXPECT_FALSE(filterChain.update(gridMap, gridMap));

    // Bayes filter needs current_probability and history layers
    gridMap.add("current_probability", 0);
    EXPECT_FALSE(filterChain.update(gridMap, gridMap));

    // Expects 4 history layers, only add 2 and ensure it throws error
    gridMap.add("history_0", 0);
    gridMap.add("history_1", 0);
    EXPECT_FALSE(filterChain.update(gridMap, gridMap));

    // Add other 2 history layers
    gridMap.add("history_2", 0);
    gridMap.add("history_3", 0);
    EXPECT_TRUE(filterChain.update(gridMap, gridMap));

}

// TO ADD: TEST RUNNING FILTER WHEN FRAMES_IN_SCOPE IS JUST ALL 0's

/***
 * All of the tests below this point are just checking that the math
 * comes out as expected. You can use this to refer to the expected performance
 * given the settings.
 */

/**
 * Ensure its outputting expected values when a reading is given as 1 or 0
 */
TEST(BayesUpdate, binary)
{
    ros::NodeHandle nh("~");

    // NOTE: Current probability will be filled with nans, but the filter should take care of it
    grid_map::GridMap gridMap({"current_probability"});
    gridMap.setFrameId("random");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if (!filterChain.configure("mitre_fast_layered_map_filters_bayes_update", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    // Add all of our history layers, -1 for no data
    gridMap.add("history_0", -1);
    gridMap.add("history_1", -1);
    gridMap.add("history_2", -1);
    gridMap.add("history_3", -1);

    //
    // FRAME ONE
    //

    // Initialize obstacle detected, only two cells have detection in first frame
    gridMap["history_0"] <<
    1, 1, 0,
    0, 0, 0,
    0, 0, 0;

    // Create matrix of expected output to compare to
    Eigen::MatrixXf answerMat(3, 3);
    answerMat << (2.0 / 3), (2.0 / 3), (1.0 / 3),
        (1.0 / 3), (1.0 / 3), (1.0 / 3),
        (1.0 / 3), (1.0 / 3), (1.0 / 3);

    if (!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_FLOAT_EQ(answerMat(index(0), index(1)), gridMap.at("current_probability", index)) << "Index: " << index(0) << " " << index(1);
    }

    //
    // FRAME TWO
    //

    // Set points for second history frame
    gridMap["history_1"] <<
    1, 0, 0,
    1, 0, 0,
    0, 0, 0;

    // Create matrix of expected output to compare to
    answerMat << 0.8, 0.5, 0.2,
        0.5, 0.2, 0.2,
        0.2, 0.2, 0.2;

    if (!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_FLOAT_EQ(answerMat(index(0), index(1)), gridMap.at("current_probability", index)) << "Index: " << index(0) << " " << index(1);
    }
}

// /**
//  * Ensure its outputting expected values for non-binary readings
//  */
TEST(BayesUpdate, readings)
{
    ros::NodeHandle nh("~");

    // NOTE: Current probability will be filled with nans, but the filter should take care of it
    grid_map::GridMap gridMap({"current_probability"});
    gridMap.setFrameId("random");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if (!filterChain.configure("mitre_fast_layered_map_filters_bayes_update", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    //
    // FRAME ONE
    //

    // Add all of our history layers, -1 for no data
    gridMap.add("history_0", -1);
    gridMap.add("history_1", -1);
    gridMap.add("history_2", -1);
    gridMap.add("history_3", -1);

    gridMap["history_0"] <<
    0, 1, 2,
    3, 4, 5,
    6, 7, 8;

    // Create matrix of expected output to compare to
    // ALL PROBABILITY CALCULATED USING src/utils/bayes_filter_profiler.py
    Eigen::MatrixXf answerMat(3, 3);
    answerMat << (1.0 / 3), (2.0 / 3), 0.8333333,
        0.9836, 0.9859, 0.9876,
        0.9890, 0.99, 0.99;

    if (!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_NEAR(answerMat(index(0), index(1)), gridMap.at("current_probability", index), 0.001) << "Index: " << index(0) << " " << index(1);
    }

    //
    // FRAME TWO
    //

    gridMap["history_1"] <<
    4, 3, 0,
    5, 2, 1,
    8, 9, 3;

    // Create matrix of expected output to compare to
    answerMat << 0.9722, 0.9917, 0.714285,
        0.9997, 0.9971, 0.9937,
        0.9998, 0.9998, 0.9998;

    if (!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_NEAR(answerMat(index(0), index(1)), gridMap.at("current_probability", index), 0.001) << "Index: " << index(0) << " " << index(1);
    }
}

// TEST(BayesUpdate, maxReadings)
// {
//     // CREATE TEST THAT USES MAX NUMBER OF HISTORY READINGS
// }