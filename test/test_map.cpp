#include <gtest/gtest.h>
#include "sensor_map.hpp"
#include "init_vars.hpp"
#include <ros/ros.h>
#include "grid_map_core/grid_map_core.hpp"

#define GTEST_COUT std::cerr << "[   INFO   ] "

TEST(SensorMap, active)
{
    EXPECT_EQ(0, 0);
}

TEST(SensorMap, friendClass)
{
    // Basic test to make sure friend class functionality is working

    MapTestEnv env;

    // Send and compare configs
    ros::NodeHandle nh("~");
    mitre_fast_layered_map::SensorMap map(&nh, env.defaultConfig_);
    mitre_fast_layered_map::TestMap testMap;
    EXPECT_TRUE(testMap.CheckConfigEqual(map, env.defaultConfig_));
}

// Test bad filter chain namespace
// @TODO: Apparently filter chain doesnt return an error occured
// on nonexistent. Not sure how else to see this will happen
// {
//     MapTestEnv env;
//     env.defaultConfig_.filterNs = "nonexistent";
    
//     ros::NodeHandle nh("~");
//     mitre_fast_layered_map::SensorMap map(&nh, env.defaultConfig_);
//     EXPECT_EQ(-1, map.init());
// }

// Test map has appropriate frame and geometry
TEST(SensorMap, correctMapGeometry)
{
    MapTestEnv env;
    mitre_fast_layered_map::TestMap testMap;

    ros::NodeHandle nh("~");
    mitre_fast_layered_map::SensorMap map(&nh, env.defaultConfig_);
    map.init();
    
    EXPECT_TRUE(testMap.CheckGeometry(map, env.defaultConfig_.len, env.defaultConfig_.resolution));
    EXPECT_TRUE(testMap.CheckFrame(map, env.defaultConfig_.mapFrameId));
}

TEST(SensorMap, irregularGeometry)
{
    MapTestEnv env;
    mitre_fast_layered_map::TestMap testMap;

    // Length is 50 but resolution is 0.3 which doesn't divide evenly
    // we expect length to be set to 50.1
    env.defaultConfig_.resolution = 0.3;

    ros::NodeHandle nh("~");
    mitre_fast_layered_map::SensorMap map(&nh, env.defaultConfig_);
    map.init();
    
    EXPECT_TRUE(testMap.CheckGeometry(map, grid_map::Length(50.1, 50.1), env.defaultConfig_.resolution));
    EXPECT_TRUE(testMap.CheckFrame(map, env.defaultConfig_.mapFrameId));
}

// Test run before using init
TEST(SensorMap, runBeforeinit)
{
    MapTestEnv env;

    ros::NodeHandle nh("~");
    mitre_fast_layered_map::SensorMap map(&nh, env.defaultConfig_);
    EXPECT_EQ(-1, map.once());
}

// Test run after init
TEST(SensorMap, runAfterinit)
{
    MapTestEnv env;

    ros::NodeHandle nh("~");
    mitre_fast_layered_map::SensorMap map(&nh, env.defaultConfig_);
    EXPECT_EQ(0, map.init());
    EXPECT_EQ(0, map.once());
}

// Test odomCb
TEST(SensorMap, odomCb)
{
    MapTestEnv env;
    mitre_fast_layered_map::TestMap testMap;
    ros::NodeHandle nh("~");
    mitre_fast_layered_map::SensorMap map(&nh, env.defaultConfig_);
    
    EXPECT_EQ(0, map.init());
    // Create odom msg
    nav_msgs::Odometry odomMsg;
    odomMsg.pose.pose.position.x = 100;
    odomMsg.pose.pose.position.y = 100;
    nav_msgs::Odometry::ConstPtr odomPtr = boost::make_shared<nav_msgs::Odometry>(odomMsg);
    EXPECT_NO_FATAL_FAILURE(map.odomCb(odomPtr));
    // // Check it moved to the correct area
    EXPECT_TRUE(testMap.CheckPosition(map, odomMsg.pose.pose.position.x,
        odomMsg.pose.pose.position.y));

}

// Test updating map very quickly (faster than the update time)
// TODO: Unable to be implemented until update rate is a configuration variable
// TEST(SensorMap, fastOdomCb)
// {
//     MapTestEnv env;
//     mitre_fast_layered_map::TestMap testMap;
//     ros::NodeHandle nh("~");
//     mitre_fast_layered_map::SensorMap map(&nh, env.defaultConfig_);
    
//     EXPECT_EQ(0, map.init());
//     // Create odom msg
//     nav_msgs::Odometry odomMsg;
//     odomMsg.pose.pose.position.x = 10;
//     odomMsg.pose.pose.position.y = 10;
//     nav_msgs::Odometry::ConstPtr odomPtr = boost::make_shared<nav_msgs::Odometry>(odomMsg);
//     EXPECT_NO_FATAL_FAILURE(map.odomCb(odomPtr));
//     // // Check it moved to the correct area
//     EXPECT_TRUE(testMap.CheckPosition(map, odomMsg.pose.pose.position.x,
//         odomMsg.pose.pose.position.y));

//     // WARNING: Possible race condition. We expect this to be faster
//     // than 1/10th sec, but if not the test will fail
//     odomMsg.pose.pose.position.x = 20;
//     odomMsg.pose.pose.position.y = 20;
//     odomPtr = boost::make_shared<nav_msgs::Odometry>(odomMsg);
//     EXPECT_NO_FATAL_FAILURE(map.odomCb(odomPtr));
//     // // Check it moved to the correct area
//     EXPECT_FALSE(testMap.CheckPosition(map, odomMsg.pose.pose.position.x,
//         odomMsg.pose.pose.position.y));
// }

// Test moving map out of bounds of current map
TEST(SensorMap, odomCbOutOfMapBounds)
{
    MapTestEnv env;
    mitre_fast_layered_map::TestMap testMap;
    ros::NodeHandle nh("~");
    mitre_fast_layered_map::SensorMap map(&nh, env.defaultConfig_);
    
    EXPECT_EQ(0, map.init());
    // Create odom msg
    nav_msgs::Odometry odomMsg;
    odomMsg.pose.pose.position.x = 100;
    odomMsg.pose.pose.position.y = 100;
    nav_msgs::Odometry::ConstPtr odomPtr = boost::make_shared<nav_msgs::Odometry>(odomMsg);
    EXPECT_NO_FATAL_FAILURE(map.odomCb(odomPtr));
    // // Check it moved to the correct area
    EXPECT_TRUE(testMap.CheckPosition(map, odomMsg.pose.pose.position.x,
        odomMsg.pose.pose.position.y));
}

// Test move map doesn't have any nans
TEST(SensorMap, noNans)
{
    MapTestEnv env;
    mitre_fast_layered_map::TestMap testMap;
    ros::NodeHandle nh("~");
    mitre_fast_layered_map::SensorMap map(&nh, env.defaultConfig_);
    
    EXPECT_EQ(0, map.init());

    EXPECT_EQ(0, map.moveMap(10, 10));
    // Check it moved to the correct area
    EXPECT_TRUE(testMap.CheckPosition(map, 10, 10));
    // We don't want to find any nans in the map after moving
    EXPECT_TRUE(testMap.CheckNans(map));
}

// Test move map returns 1 on outside of current map bounds
TEST(SensorMap, moveMapOutOfMapBounds)
{
    MapTestEnv env;
    mitre_fast_layered_map::TestMap testMap;
    ros::NodeHandle nh("~");
    mitre_fast_layered_map::SensorMap map(&nh, env.defaultConfig_);
    
    EXPECT_EQ(0, map.init());

    EXPECT_EQ(1, map.moveMap(100, 100));
    // // Check it moved to the correct area
    EXPECT_TRUE(testMap.CheckPosition(map, 100, 100));
    // We don't want to find any nans in the map after moving
    EXPECT_TRUE(testMap.CheckNans(map));
}

// For now we assume the point cloud callbacks work since all they do is transform
// the point clouds into the correct frame. Below we test the update functions
// along with the corresponding filter chains.

// FILTERS EACH HAVE THEIR OWN SEPERATE TEST FILES
// SEE THOSE FOR FULL FUNCTIONALITY TESTS

// Send ground points and see that the cells are set to 0
TEST(SensorMap, groundPoints)
{
    MapTestEnv env;
    mitre_fast_layered_map::TestMap testMap;
    ros::NodeHandle nh("~");
    mitre_fast_layered_map::SensorMap map(&nh, env.smallMap_);
    EXPECT_EQ(0, map.init());
    EXPECT_TRUE(testMap.CheckPosition(map, 0, 0));

    // Create point cloud for input
    mitre_fast_layered_map::PointCloud cloud;
    cloud.width = 6;
    cloud.height = 1;
    // We have a 5mx5m map, so put points within that map
    // Uses FLU so x is forward and back, y is sideways
    cloud.points.push_back(pcl::PointXYZ(2, -2, 0)); // (0, 4)
    cloud.points.push_back(pcl::PointXYZ(1, -1, 0)); // (1, 3)
    cloud.points.push_back(pcl::PointXYZ(0, 0, 0)); // (2, 2)
    cloud.points.push_back(pcl::PointXYZ(0, 0, 0)); // (2, 2)
    cloud.points.push_back(pcl::PointXYZ(-2, 1, 0)); // (4,1)
    cloud.points.push_back(pcl::PointXYZ(1, 1, 0)); // (1, 1)
    cloud.points.push_back(pcl::PointXYZ(-1, -2, 0)); // (3, 4)

    Eigen::MatrixXi answerMat(5,5);
    answerMat <<
     0, 0, 0, 0, 1,
     0, 1, 0, 1, 0,
     0, 0, 2, 0, 0,
     0, 0, 0, 0, 1,
     0, 1, 0, 0, 0;

    // Update ground points
    map.updateGroundPts(cloud);

    // Check map against our answer mat
    EXPECT_TRUE(testMap.TestMapCells(map, "ground", answerMat));
}

// Send points with elevation make sure cells have lowest elevation
TEST(SensorMap, elevationMin)
{
    MapTestEnv env;
    mitre_fast_layered_map::TestMap testMap;
    ros::NodeHandle nh("~");
    mitre_fast_layered_map::SensorMap map(&nh, env.smallMap_);
    EXPECT_EQ(0, map.init());
    EXPECT_TRUE(testMap.CheckPosition(map, 0, 0));

    // Create point cloud for input
    mitre_fast_layered_map::PointCloud cloud;
    cloud.width = 7;
    cloud.height = 1;
    // We have a 5mx5m map, so put points within that map
    // Uses FLU so x is forward and back, y is sideways
    cloud.points.push_back(pcl::PointXYZ(2, -2, 2)); // (0, 4)
    cloud.points.push_back(pcl::PointXYZ(1, -1, 0)); // (1, 3)
    cloud.points.push_back(pcl::PointXYZ(0, 0, -2)); // (2, 2)
    cloud.points.push_back(pcl::PointXYZ(-2, 1, 1)); // (4,1)
    cloud.points.push_back(pcl::PointXYZ(1, 1, 5)); // (1, 1)
    cloud.points.push_back(pcl::PointXYZ(1, 1, 2)); // Should take lower of two heights
    cloud.points.push_back(pcl::PointXYZ(-1, -2, 0)); // (3, 4)

    // -1 denotes a nan
    Eigen::MatrixXf answerMat(5,5);
    answerMat <<
    FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX,       2,
    FLT_MAX,       2, FLT_MAX,       0, FLT_MAX,
    FLT_MAX, FLT_MAX,      -2, FLT_MAX, FLT_MAX,
    FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX,       0,
    FLT_MAX,       1, FLT_MAX, FLT_MAX, FLT_MAX;

    // Update ground points
    map.updateGroundPts(cloud);

    // Check map against our answer mat
    EXPECT_TRUE(testMap.TestMapCells(map, "elevation_min", answerMat));
}

 // Test that ground points show as obstacles
TEST(SensorMap, nongroundPoints)
{
    MapTestEnv env;
    mitre_fast_layered_map::TestMap testMap;
    ros::NodeHandle nh("~");
    // Small map for this test
    env.smallMap_.len = grid_map::Length(3, 3);
    mitre_fast_layered_map::SensorMap map(&nh, env.smallMap_);
    EXPECT_EQ(0, map.init());
    EXPECT_TRUE(testMap.CheckPosition(map, 0, 0));

    // Create point cloud for input
    mitre_fast_layered_map::PointCloud cloud;
    cloud.width = 33;
    cloud.height = 1;
    // We have a 3mx3m map, so put points within that map
    // Uses FLU so x is forward and back, y is sideways
    // See answer matrix for where all points are expected to land
    Eigen::MatrixXi cloudPointStorage(7, 3);
    // x, y, num times
    cloudPointStorage <<
     1,  1, 1, // (0, 0)
     1,  0, 2, // (0, 1)
     1, -1, 3, // (0, 2)
     0,  0, 1, // (1, 1)
     0, -1, 2, // (1, 2)
    -1,  1, 9, // (2, 0)
    -1, -1, 15; // (2, 2)

    // Build our point cloud
    for (int i = 0; i < cloudPointStorage.rows(); i++)
    {
        auto row = cloudPointStorage.row(i);
        for (int j = 0; j < row(2); j++)
        {
            cloud.points.push_back(pcl::PointXYZ(row(0), row(1), 0));
        }
    }

    // FRAME 1

    // Number of hits we expect to see on the points in cell layer
    Eigen::MatrixXi history0Answers(3,3);
    history0Answers <<
    1,   2,   3,
    0,   1,   2,
    9,   0,   15;

    // Update ground points
    map.updateNongroundPts(cloud);

    // Check map against our answer mat
    EXPECT_TRUE(testMap.TestMapCells(map, "history_0", history0Answers));

    // FRAME 2
    // Move first row out of scope
    EXPECT_EQ(0, map.moveMap(-1, 0));

    // As we move, the map should mark that these cells have no valid readings now
    history0Answers <<
    -1, -1, -1,
     0,  1,  2,
     9,  0,  15;

    EXPECT_TRUE(testMap.TestMapCells(map, "history_0", history0Answers));

    // The answer matrices may not be what you expect.
    // Remember that grid map has a rolling grid, so the movement above
    // has the affect of the top row moving to the bottom of the gridmap
    // but eigen doesn't do that.

    history0Answers <<
    -1, -1, -1,
     0,  1,  2,
     9,  0, 15;

    Eigen::MatrixXi history1Answers(3,3);
    history1Answers <<
    0,  0,  0,
    0,  1,  2,
    9,  0,  15;

    map.updateNongroundPts(cloud);

    // Check map against our answer mats
    EXPECT_TRUE(testMap.TestMapCells(map, "history_0", history0Answers));
    EXPECT_TRUE(testMap.TestMapCells(map, "history_1", history1Answers));

}

// Test doing a lot of non ground updates and 
// make sure we get expected results
TEST(SensorMap, nongroundManyFrames)
{
    MapTestEnv env;
    mitre_fast_layered_map::TestMap testMap;
    ros::NodeHandle nh("~");
    // Small map for this test
    env.smallMap_.len = grid_map::Length(3, 3);
    mitre_fast_layered_map::SensorMap map(&nh, env.smallMap_);
    EXPECT_EQ(0, map.init());
    EXPECT_TRUE(testMap.CheckPosition(map, 0, 0));

    // Create point cloud for input
    mitre_fast_layered_map::PointCloud cloud;
    cloud.width = 33;
    cloud.height = 1;
    // We have a 3mx3m map, so put points within that map
    // Uses FLU so x is forward and back, y is sideways
    // See answer matrix for where all points are expected to land
    Eigen::MatrixXi cloudPointStorage(7, 3);
    // x, y, num times
    cloudPointStorage <<
     1,  1, 1, // (0, 0)
     1,  0, 2, // (0, 1)
     1, -1, 3, // (0, 2)
     0,  0, 1, // (1, 1)
     0, -1, 2, // (1, 2)
    -1,  1, 9, // (2, 0)
    -1, -1, 15; // (2, 2)

    // Build our point cloud
    for (int i = 0; i < cloudPointStorage.rows(); i++)
    {
        auto row = cloudPointStorage.row(i);
        for (int j = 0; j < row(2); j++)
        {
            cloud.points.push_back(pcl::PointXYZ(row(0), row(1), 0));
        }
    }

    // FRAME 1

    // Number of hits we expect to see on the points in cell layer
    Eigen::MatrixXi historyAnswers(3,3);
    historyAnswers <<
    1, 2, 3,
    0, 1, 2,
    9, 0, 15;

    // Run 5 iterations of updating nonground points
    for (int i = 0; i < 5; i++)
    {
        map.updateNongroundPts(cloud);
    }

    // Check map against our answer mat
    EXPECT_TRUE(testMap.TestMapCells(map, "history_0", historyAnswers));
    EXPECT_TRUE(testMap.TestMapCells(map, "history_1", historyAnswers));
    EXPECT_TRUE(testMap.TestMapCells(map, "history_2", historyAnswers));
    EXPECT_TRUE(testMap.TestMapCells(map, "history_3", historyAnswers));
    EXPECT_TRUE(testMap.TestMapCells(map, "history_4", historyAnswers));

    // Run a lot more times, all frames should have the same points
    for (int i = 1; i < 100; i++)
    {
        map.updateNongroundPts(cloud);
        if (i % 20 == 0)
        {
            for (int j = 0; j < env.smallMap_.numHistoryLayers; j++)
            {
                std::string layerName = env.smallMap_.historyLayerPrefix + std::to_string(j);
                EXPECT_TRUE(testMap.TestMapCells(map, layerName, historyAnswers));
            }
        }
    }

}

/*****************
 * NOTE: THIS TEST SEEMS TO FAIL RANDOMLY DUE TO A BAD MALLOC. I HAVEN'T
 * BEEN ABLE TO IDENTIFY IF THIS IS A MEMORY LEAK OR SOMETHING ELSE.
 *****************/

/**
 * Adds lidar points and then runs our current best
 * filter pipeline. The pipeline should look like
 * Nonground points -> bayes -> threshold -> ray trace -> inflation
 */
TEST(SensorMap, fullPipeline)
{
    MapTestEnv env;
    mitre_fast_layered_map::TestMap testMap;
    ros::NodeHandle nh("~");
    env.smallMap_.obstacleFilterNs = "mitre_fast_layered_map_obstacle_filters";
    env.smallMap_.mapOperationsFilterNs = "mitre_fast_layered_map_map_operations";
    mitre_fast_layered_map::SensorMap map(&nh, env.smallMap_);
    EXPECT_EQ(0, map.init());
    EXPECT_TRUE(testMap.CheckPosition(map, 0, 0));

    // Create point cloud for input
    mitre_fast_layered_map::PointCloud cloud;
    cloud.width = 37;
    cloud.height = 1;
    

    // Points cloud builder. 
    Eigen::MatrixXi cloudPointStorage(8, 3);
    // Format: x, y, num times
    cloudPointStorage <<
     1,  1, 9, // (1, 1)
     1, -1, 1, // (1, 3)
     0, -1, 2, // (2, 3)
    -1, -1, 4, // (3, 3)
    -1,  0, 6, // (3, 2)
    -1,  1, 7, // (3, 1)
     0,  1, 8, // (2, 1)
     0,  0, 0; // Filler, we use this below

    // Build our point cloud
    for (int i = 0; i < cloudPointStorage.rows(); i++)
    {
        auto row = cloudPointStorage.row(i);
        for (int j = 0; j < row(2); j++)
        {
            cloud.points.push_back(pcl::PointXYZ(row(0), row(1), 0));
        }
    }

    // FRAME 1

    // Number of hits we expect to see on the points in cell layer
    Eigen::MatrixXi history0Answers(5,5);
    history0Answers <<
    0, 0, 0, 0, 0,
    0, 9, 0, 1, 0,
    0, 8, 0, 2, 0,
    0, 7, 6, 4, 0,
    0, 0, 0, 0, 0;

    Eigen::MatrixXi nongroundLayerAnswers(5, 5);
    nongroundLayerAnswers <<
    30,  30,  30,   0,  0,
    30, 100,  30,   0,  0,
    30, 100,  30,  30,  30,
    30, 100, 100, 100,  30,
    30,  30,  30,  30,  30;

    map.updateNongroundPts(cloud);

    // Check map against our answer mat
    EXPECT_TRUE(testMap.TestMapCells(map, "history_0", history0Answers));
    EXPECT_TRUE(testMap.TestMapCells(map, "nonground", nongroundLayerAnswers));

    // Create point cloud for input
    mitre_fast_layered_map::PointCloud cloud2;
    cloud2.width = 36;
    cloud2.height = 1;

    // x, y, num times
    cloudPointStorage <<
     1,  1, 8, // (1, 1)
     1,  0, 1, // (1, 2)
     1, -1, 2, // (1, 3)
     0, -1, 3, // (2, 3)
    -1, -1, 4, // (3, 3)
    -1,  0, 5, // (3, 2)
    -1,  1, 6, // (3, 1)
     0,  1, 7; // (2, 1)

    // Build our point cloud
    for (int i = 0; i < cloudPointStorage.rows(); i++)
    {
        auto row = cloudPointStorage.row(i);
        for (int j = 0; j < row(2); j++)
        {
            cloud2.points.push_back(pcl::PointXYZ(row(0), row(1), 0));
        }
    }

    // Number of hits we expect to see on the points in cell layer
    Eigen::MatrixXi history1Answers(5,5);
    history1Answers <<
    0, 0, 0, 0, 0,
    0, 8, 1, 2, 0,
    0, 7, 0, 3, 0,
    0, 6, 5, 4, 0,
    0, 0, 0, 0, 0;

    nongroundLayerAnswers <<
    30,  30,  30,  30,  30,
    30, 100,  30, 100,  30,
    30, 100,  30, 100,  30,
    30, 100, 100, 100,  30,
    30,  30,  30,  30,  30;

    map.updateNongroundPts(cloud2);

    // Check map against our answer mat
    EXPECT_TRUE(testMap.TestMapCells(map, "history_0", history0Answers));
    EXPECT_TRUE(testMap.TestMapCells(map, "history_1", history1Answers));
    EXPECT_TRUE(testMap.TestMapCells(map, "nonground", nongroundLayerAnswers));


}

// Height filter, make sure cloud points above a certain threshold are filtered out
// Since enabling 3d height filtering we would need to publish some frame transforms
// thus this test has been disabled until we take time to setup the tf tree.
// TEST(SensorMap, heightFilter)
// {
//     MapTestEnv env;
//     mitre_fast_layered_map::TestMap testMap;
//     ros::NodeHandle nh("~");
//     mitre_fast_layered_map::SensorMap map(&nh, env.smallMap_);
//     EXPECT_EQ(0, map.init());
//     EXPECT_TRUE(testMap.CheckPosition(map, 0, 0));

//     // Create point cloud for input
//     mitre_fast_layered_map::PointCloud cloud;
//     cloud.width = 6;
//     cloud.height = 1;
//     // We have a 5mx5m map, so put points within that map
//     // Uses FLU so x is forward and back, y is sideways
//     cloud.points.push_back(pcl::PointXYZ(2, -2, -100)); // (0, 4)
//     cloud.points.push_back(pcl::PointXYZ(1, -1, 3)); // (1, 3) filtered
//     cloud.points.push_back(pcl::PointXYZ(0, 0, 0)); // (2, 2)
//     cloud.points.push_back(pcl::PointXYZ(-2, 1, 1.5)); // (4,1)
//     cloud.points.push_back(pcl::PointXYZ(1, 1, -1)); // (1, 1)
//     cloud.points.push_back(pcl::PointXYZ(-1, -2, 100)); // (3, 4) filtered

//     int nan = -1; // Grid map will store nans
//     // -1 denotes a nan
//     Eigen::MatrixXi answerMat(5,5);
//     answerMat <<
//     0,   0,   0,   0, 100,
//     0, 100,   0,   0,   0,
//     0,   0, 100,   0,   0,
//     0,   0,   0,   0,   0,
//     0, 100,   0,   0,   0;

//     // Update ground points
//     map.updateNongroundPts(cloud);

//     // Check map against our answer mat
//     EXPECT_TRUE(testMap.TestMapCells(map, "nonground", answerMat));
// }
