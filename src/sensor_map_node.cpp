/**
 * @file dart_gridmap_node.cpp
 * @brief Startup node for the map controller that sets up configuration details
 * @author Trevor Bostic
 */
// ROS
#include <ros/ros.h>

// Us
#include "sensor_map.hpp"

int getParameters(ros::NodeHandle _nh, mitre_fast_layered_map::MapConfiguration& _mapConfig)
{

    // Constants for now. Use <remap> in launch file
    // to point to correct topics
    _mapConfig.occupancyOutputTopic = "costmap";
    _mapConfig.gridmapOutputTopic = "gridmap";


    // Required Parameters
    if (!_nh.getParam("odom_sub_topic", _mapConfig.odomSubTopic))
    {
        ROS_ERROR("No odometry sub topic set.");
        return -1;
    }
    
    if (!_nh.getParam("ground_points_sub_topic", _mapConfig.groundPointSubTopic))
    {
        ROS_ERROR("No ground point sub topic set.");
        return -1;
    }
    
    if (!_nh.getParam("nonground_points_sub_topic", _mapConfig.nonGroundPointSubTopic))
    {
        ROS_ERROR("No nonground points sub topic set.");
        return -1;
    }

    if (!_nh.getParam("map_frame_id", _mapConfig.mapFrameId))
    {
        ROS_ERROR("No map frame_id set.");
        return -1;
    }

    if (!_nh.getParam("vehicle_frame_id", _mapConfig.vehicleFrameId))
    {
        ROS_ERROR("No vehicle frame_id set.");
        return -1;
    }

    int mapLen;
    if (!_nh.getParam("map_len", mapLen))
    {
        ROS_ERROR("No map length set.");
        return -1;
    }

    _mapConfig.len = grid_map::Length(mapLen, mapLen);

    if (!_nh.getParam("resolution", _mapConfig.resolution))
    {
        ROS_ERROR("No map resolution set.");
        return -1;
    }

    if (!_nh.getParam("obstacle_filter_ns", _mapConfig.obstacleFilterNs))
    {
        ROS_ERROR("No obstacle filter ns specified.");
        return -1;
    }

    if (!_nh.getParam("map_operations_filter_ns", _mapConfig.mapOperationsFilterNs))
    {
        ROS_ERROR("No map operations filter ns specified.");
        return -1;
    }

    // Optional parameters
    _nh.param<std::string>("map_name", _mapConfig.mapName, "Map");
    
    // Empty string notifies we won't be subscribed to these
    _nh.param<std::string>("marker_sub_topic", _mapConfig.markerSubTopic, "");
    _nh.param<std::string>("static_map_sub_topic", _mapConfig.staticMapSubTopic, "");

    // Settigns for saving history of readings
    _nh.param<std::string>("history_layer_prefix", _mapConfig.historyLayerPrefix, "history_");
    _nh.param<int>("num_history_layers", _mapConfig.numHistoryLayers, 10);

    // Probability for setting permanent obstacles cells in the map
    _nh.param<bool>("enable_permanent_obstacles", _mapConfig.enablePermanentObstacles, false);
    _nh.param<double>("permanent_obstacle_probability", _mapConfig.permanentFilterProb, 1);

    // Enable or disable height filtering
    _nh.param<bool>("enable_height_point_filtering", _mapConfig.enablePointHeightFilter, false);

    // -1 for no filtering
    _nh.param<double>("max_point_height", _mapConfig.maxPointHeight, -1);
    // We assume that the odom message is from the center of the robot
    _nh.param<bool>("enable_bounding_box_filter", _mapConfig.enableBoundingBoxFilter, true);
    _nh.param<double>("footprint_len_m", _mapConfig.footPrintLen, 4);
    _nh.param<double>("footprint_width_m", _mapConfig.footPrintWidth, 2);

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "t_sensor_map");
    ros::NodeHandle nh("~");
    mitre_fast_layered_map::MapConfiguration mapConfig;

    if (getParameters(nh, mapConfig) < 0)
    {
        ROS_ERROR("Configuration error. Unable to get required parameters.");
        exit(1);
    }

    // ROS_INFO("Map Name: %s, Ground Topic: %s, Nonground Topic: %s, Odom Sub Topic: %s, Marker Topic: %s, "
    //          "Output Topic: %s, Frame Id: %s, Filter NS: %s, Len: %f, Res: %f",
    //          mapConfig.mapName.c_str(), mapConfig.groundPointSubTopic.c_str(),
    //          mapConfig.nonGroundPointSubTopic.c_str(), mapConfig.odomSubTopic.c_str(), mapConfig.markerSubTopic.c_str(),
    //          mapConfig.occupancyOutputTopic.c_str(), mapConfig.frameId.c_str(),
    //          mapConfig.filterNs.c_str(), mapConfig.len.x(), mapConfig.resolution);

    mitre_fast_layered_map::SensorMap sensorMap(&nh, mapConfig);

    if (sensorMap.init() < 0)
    {
        ROS_ERROR("Unable to initialize map. Exiting...");
        exit(-1);
    }

    ros::Rate rate(10);

    while(ros::ok())
    {
        if(sensorMap.once() != 0)
        {
            ROS_WARN_THROTTLE(1, "Map unable to publish.");
        }
        rate.sleep();
        ros::spinOnce();
    }
}