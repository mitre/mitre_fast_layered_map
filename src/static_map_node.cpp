/**
 * @file static_map_node.cpp
 * @brief Startup node for a static map instance. 
 * @author Trevor Bostic
 */
// ROS
#include <ros/ros.h>

// Us
#include "static_map.hpp"

int getParameters(ros::NodeHandle _nh, mitre_fast_layered_map::StaticMapConfiguration& _mapConfig)
{

    // Constants for now. Use <remap> in launch file
    // to point to correct topics
    _mapConfig.occupancyOutputTopic = "costmap";
    _mapConfig.gridmapOutputTopic = "gridmap";


    // Required Parameters
    if (!_nh.getParam("static_map_sub_topic", _mapConfig.staticMapSubTopic))
    {
        ROS_ERROR("No static map sub topic set.");
        return -1;
    }

    // Optional parameters
    _nh.param<std::string>("map_name", _mapConfig.mapName, "Static Map");
    // Empty string notifies we won't be subscribed to these
    _nh.param<std::string>("marker_sub_topic", _mapConfig.markerSubTopic, "");
    _nh.param<std::string>("gridmap_sub_topic", _mapConfig.gridMapSubTopic, "");
    _nh.param<std::string>("gridmap_layer", _mapConfig.gridMapLayer, "");


    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "t_static_map");
    ros::NodeHandle nh("~");
    mitre_fast_layered_map::StaticMapConfiguration mapConfig;

    if (getParameters(nh, mapConfig) < 0)
    {
        ROS_ERROR("Configuration error. Unable to get required parameters.");
        exit(1);
    }

    mitre_fast_layered_map::StaticMap staticMap(&nh, mapConfig);

    if (staticMap.init() < 0)
    {
        ROS_ERROR("Unable to initialize map. Exiting...");
        exit(-1);
    }

    ros::Rate rate(10);

    while(ros::ok())
    {
        if(staticMap.once() != 0)
        {
            ROS_WARN_THROTTLE(1, "Map unable to publish.");
        }
        rate.sleep();
        ros::spinOnce();
    }
}