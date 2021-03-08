#include <gtest/gtest.h>
#include "filters.hpp"
#include <filters/filter_chain.h>
#include <math.h>
#include <Eigen/Dense>

#define GTEST_COUT std::cerr << "[   INFO   ] "

TEST(Inflation, active)
{
    EXPECT_EQ(0, 0);
}

/**
 * Test that error checking works for cases in 
 * which the ground and nonground layers don't exist.
 */
TEST(Inflation, layers_dont_exist)
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

    // Inflation filter needs nonground layer
    gridMap.add("nonground", 0);
    EXPECT_TRUE(filterChain.update(gridMap, gridMap));
}

/**
 * Don't inflate any spots when there are no obstacles
 * c = clear
 * u = unknown
 * o = occupied
 * 
 * Input
 * [c], [c], [c],
 * [c], [c], [c],
 * [c], [c], [c]
 * 
 * Expected Output
 * [c], [c], [c],
 * [c], [c], [c],
 * [c], [c], [c]
 */
TEST(Inflation, all_clear)
{
    ros::NodeHandle nh("~");

    grid_map::GridMap gridMap;
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);

    // Initialize nonground plane to all clear
    gridMap.add("nonground", 0);

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_EQ(0, gridMap.at("nonground", index));
    }
}

/**
 * Inflate object to provide buffer in all surrounding cells
 * c = clear = 0
 * u = unknown = nan
 * o = occupied = 100
 * b = buffer = 30
 * 
 * Input
 * [c], [c], [c],
 * [c], [o], [c],
 * [c], [c], [c]
 * 
 * Expected Output
 * [b], [b], [b],
 * [b], [o], [b],
 * [b], [b], [b]
 */
TEST(Inflation, center_filled)
{
    ros::NodeHandle nh("~");

    grid_map::GridMap gridMap;
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(3, 3), 1);

    // Initialize nonground plane to all clear
    gridMap.add("nonground", 0);
    gridMap.at("nonground", grid_map::Index(1, 1)) = 100;

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        if (index(0) == 1 && index(1) == 1)
        {
            EXPECT_EQ(100, gridMap.at("nonground", index));
        }
        else
        {
            EXPECT_EQ(30, gridMap.at("nonground", index)) <<
        "Index: " << index(0) << " " << index(1);
        }
    }
}

/**
 * When using too small of an inflation radius, we have
 * to be careful about clipping corners
 * 
 * c = clear = 0
 * u = unknown = nan
 * o = occupied = 100
 * b = buffer = 30
 * 
 * Input
 * [c], [c], [c],
 * [c], [o], [c],
 * [c], [c], [c]
 * 
 * Expected Output
 * [c], [b], [c],
 * [b], [o], [b],
 * [c], [b], [c]
 */
TEST(Inflation, center_filled_small_radius)
{
    ros::NodeHandle nh("~");

    grid_map::GridMap gridMap;
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(5, 5), 1);

    // Initialize nonground plane to all clear
    gridMap.add("nonground", 0);
    gridMap.at("nonground", grid_map::Index(2, 2)) = 100;

    // Create answer matrix
    Eigen::MatrixXi answerMat(5,5);
    answerMat <<
    0,  0,   0,  0, 0,
    0, 30,  30, 30, 0,
    0, 30, 100, 30, 0,
    0, 30,  30, 30, 0,
    0,  0,   0,  0, 0;

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters_low_inflation", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_EQ(answerMat(index(0), index(1)), gridMap.at("nonground", index)) <<
        "Index: " << index(0) << " " << index(1);
    }
}

/**
 * Say all squares are open
 * u = unknown = nan
 * c = clear = 0
 * o = occupied = 100
 * b = buffer = 30
 * 
 * Input
 * [c], [c], [o], [c], [c],
 * [c], [c], [o], [c], [c],
 * [o], [c], [c], [c], [o],
 * [c], [c], [o], [c], [c],
 * [c], [c], [o], [c], [c]
 * 
 * Expected Output
 * [c], [b], [o], [b], [c],
 * [b], [b], [o], [b], [b],
 * [o], [b], [b], [b], [o],
 * [b], [b], [o], [b], [b],
 * [c], [b], [o], [b], [c]
 * 
 */

TEST(Inflation, complex)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"ground"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(5, 5), 1);

    // Initialize nonground
    gridMap.add("nonground", 0);
    gridMap.at("nonground", grid_map::Index(0,2)) = 100;
    gridMap.at("nonground", grid_map::Index(1,2)) = 100;
    gridMap.at("nonground", grid_map::Index(2,0)) = 100;
    gridMap.at("nonground", grid_map::Index(2,4)) = 100;
    gridMap.at("nonground", grid_map::Index(3,2)) = 100;
    gridMap.at("nonground", grid_map::Index(4,2)) = 100;

    // Create matrix of expected output to compare to
    Eigen::MatrixXi answerMat(5, 5);
    answerMat <<
    0,   30, 100, 30, 0,
    30,  30, 100, 30, 30,
    100, 30,  30, 30, 100,
    30,  30, 100, 30, 30,
    0,   30, 100, 30, 0;


    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_EQ(answerMat(index(0), index(1)), gridMap.at("nonground", index)) <<
        "Index: " << index(0) << " " << index(1);
    }

}

TEST(Inflation, move_forward)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"ground"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(5, 5), 1);

    // Initialize nonground
    gridMap.add("nonground", 0);
    gridMap.at("nonground", grid_map::Index(0,2)) = 100;

    gridMap.move(grid_map::Position(1, 0));
    gridMap["nonground"] = (gridMap["nonground"].array().isNaN()).select(0, gridMap["nonground"]);

    // Create matrix of expected output to compare to
    Eigen::MatrixXi answerMat(5, 5);
    answerMat <<
    0, 30, 100, 30, 0,
    0, 30,  30, 30, 0,
    0,  0,   0,  0, 0,
    0,  0,   0,  0, 0,
    0, 30,  30, 30, 0;

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters_low_inflation", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_EQ(answerMat(index(0), index(1)), gridMap.at("nonground", index)) <<
        "Index: " << index(0) << " " << index(1);
    }
}

TEST(Inflation, move_backward)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"ground"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(5, 5), 1);

    // Initialize nonground
    gridMap.add("nonground", 0);
    gridMap.at("nonground", grid_map::Index(4,2)) = 100;

    gridMap.move(grid_map::Position(-1, 0));
    gridMap["nonground"] = (gridMap["nonground"].array().isNaN()).select(0, gridMap["nonground"]);

    // Create matrix of expected output to compare to
    Eigen::MatrixXi answerMat(5, 5);
    answerMat <<
    0, 30,  30, 30, 0,
    0,  0,   0,  0, 0,
    0,  0,   0,  0, 0,
    0, 30,  30, 30, 0,
    0, 30, 100, 30, 0;

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters_low_inflation", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_EQ(answerMat(index(0), index(1)), gridMap.at("nonground", index)) <<
        "Index: " << index(0) << " " << index(1);
    }
}

TEST(Inflation, move_left)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"ground"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(5, 5), 1);

    // Initialize nonground
    gridMap.add("nonground", 0);
    gridMap.at("nonground", grid_map::Index(2,0)) = 100;

    gridMap.move(grid_map::Position(0, 1));
    gridMap["nonground"] = (gridMap["nonground"].array().isNaN()).select(0, gridMap["nonground"]);

    // Create matrix of expected output to compare to
    Eigen::MatrixXi answerMat(5, 5);
    answerMat <<
      0,  0, 0, 0, 0,
     30, 30, 0, 0, 30,
    100, 30, 0, 0, 30,
     30, 30, 0, 0, 30,
      0,  0, 0, 0, 0;

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters_low_inflation", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_EQ(answerMat(index(0), index(1)), gridMap.at("nonground", index)) <<
        "Index: " << index(0) << " " << index(1);
    }
}

TEST(Inflation, move_right)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"ground"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(5, 5), 1);

    // Initialize nonground
    gridMap.add("nonground", 0);
    gridMap.at("nonground", grid_map::Index(2,4)) = 100;

    gridMap.move(grid_map::Position(0, -1));
    gridMap["nonground"] = (gridMap["nonground"].array().isNaN()).select(0, gridMap["nonground"]);

    // Create matrix of expected output to compare to
    Eigen::MatrixXi answerMat(5, 5);
    answerMat <<
      0,  0, 0,  0,   0,
     30,  0, 0, 30,  30,
     30,  0, 0, 30, 100,
     30,  0, 0, 30,  30,
      0,  0, 0,  0,  0;

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters_low_inflation", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_EQ(answerMat(index(0), index(1)), gridMap.at("nonground", index)) <<
        "Index: " << index(0) << " " << index(1);
    }
}

TEST(Inflation, move_diagonal)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"ground"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(5, 5), 1);

    // Initialize nonground
    gridMap.add("nonground", 0);
    gridMap.at("nonground", grid_map::Index(0,0)) = 100;

    gridMap.move(grid_map::Position(1, 1));
    gridMap["nonground"] = (gridMap["nonground"].array().isNaN()).select(0, gridMap["nonground"]);

    // Create matrix of expected output to compare to
    Eigen::MatrixXi answerMat(5, 5);
    answerMat <<
    100, 30, 0,  0,  30,
     30, 30, 0,  0,  30,
      0,  0, 0,  0,   0,
      0,  0, 0,  0,   0,
     30, 30, 0,  0,  30;

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters_low_inflation", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    if(!filterChain.update(gridMap, gridMap))
    {
        GTEST_FATAL_FAILURE_("Unable to update grid map.");
    }

    for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        EXPECT_EQ(answerMat(index(0), index(1)), gridMap.at("nonground", index)) <<
        "Index: " << index(0) << " " << index(1);
    }
}

//
// STRESS TESTS
//
// The below tests are just to try and make sure that inflation will never fail or throw and error.
// We could consider using try's around the eigen.block() calls since that is where any errors will pop up

TEST(Inflation, Stress_test_1)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"ground"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(50, 50), 0.3);

    // Initialize nonground
    gridMap.add("nonground", 0);

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    // Fill every cell in the gridmap
    // This will cause inflation to run on every single cell
    for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
    {
        gridMap.at("nonground", grid_map::Index(*it)) = 100;
    }

    EXPECT_NO_FATAL_FAILURE(filterChain.update(gridMap, gridMap));
}

TEST(Inflation, Stress_test_2)
{
    ros::NodeHandle nh("~");
    grid_map::GridMap gridMap({"ground"});
    gridMap.setFrameId("map");
    gridMap.setGeometry(grid_map::Length(50, 50), 0.3);

    // Initialize nonground
    gridMap.add("nonground", 0);

    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");

    if(!filterChain.configure("mitre_fast_layered_map_filters", nh))
    {
        GTEST_FATAL_FAILURE_("Unable to configure filter chain."); // We should never get here
    }

    for (int i = 0; i < 100; i++)
    {

        int t = i > 50 ? i: -i; // Switch direction halfway through
        // Create wierd irregular pattern
        grid_map::Position pos(20*t, 10 * sin(t));
        // Move the map to keep up
        gridMap.move(pos);

        // Fill every cell in the gridmap
        // This will cause inflation to run on every single cell
        for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
        {
            gridMap.at("nonground", grid_map::Index(*it)) = 100;
        }

        EXPECT_NO_FATAL_FAILURE(filterChain.update(gridMap, gridMap));

    }
}