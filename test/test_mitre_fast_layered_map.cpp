#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/console.h>

#define GTEST_COUT std::cerr << "[   INFO   ] "

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mitre_fast_layered_map_test_suite");

    // We don't want to see error messages that we mean to cause
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal)) 
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    testing::InitGoogleTest(&argc, argv);
    // ::testing::AddGlobalTestEnvironment(new KvhPackReqEnv);
    return RUN_ALL_TESTS();
}
