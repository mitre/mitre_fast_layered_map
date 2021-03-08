/**
 * @file static_map.hpp
 * @brief Mapping solution for largely static obstacles. Despite the name, it can be modified, but all modifications
 * are considered permanent unless explicitly removed. Not for use as a rapidly changing dynamic map. See the SensorMap
 * for that capability.
 * @author Trevor Bostic
 */
#ifndef mitre_fast_layered_map_static_map
#define mitre_fast_layered_map_static_map
#pragma once

// STD
#include <string>

// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Grid map
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

namespace mitre_fast_layered_map
{
    struct StaticMapConfiguration
    {
        std::string mapName; ///< Name identification for the map
        std::string staticMapSubTopic; ///< Static map from map server to use as starting configuration
        std::string markerSubTopic; ///< Topic users can post marker messages to to add obstacles
        std::string gridMapSubTopic; ///< Topic users can post grid map messages to. Any obstacles
        ///< in the grid map will be added to the static map instance.
        std::string gridMapLayer; ///< Layer to retrieve permanent obstacles from

        std::string occupancyOutputTopic; ///< Topic for occupancy grid
        std::string gridmapOutputTopic; ///< Topic for grid map msgs output
    };

    class StaticMap
    {
        public:
        StaticMap();
        StaticMap(ros::NodeHandle *, StaticMapConfiguration);

        ~StaticMap();

        int init();
        int once();

        void staticMapCb(const nav_msgs::OccupancyGrid &);
        void mapMarkerCb(const visualization_msgs::Marker &);
        void gridMapCb(const grid_map_msgs::GridMap &);

        int publishMap();

        private:

        // ROS ECOSYSTEM
        ros::NodeHandle nh_;
        ros::Subscriber staticMapSub_;
        ros::Subscriber markerSub_;
        ros::Subscriber gridMapSub_;
        ros::Publisher occGridPub_; 
        ros::Publisher gridMapPub_;

        // TRANSFORMS
        tf2_ros::Buffer tfBuffer_;              ///< Holds transformations from tf tree
        tf2_ros::TransformListener tfListener_; ///< Used in tfBuffer_

        // STATE DATA
        bool initialized_{false};
        bool recStaticMap_{false};
        grid_map::GridMap gridMap_;

        StaticMapConfiguration config_;

    };
}

#endif