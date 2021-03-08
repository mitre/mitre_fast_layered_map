/**
 * @file map.hpp
 * @brief Mapping solution that can be modified dynamically based on sensor readings.
 * @author Trevor Bostic
 */
#ifndef mitre_fast_layered_map_sensor_map
#define mitre_fast_layered_map_sensor_map
#pragma once

// STD
#include <string>

// ROS
#include <filters/filter_chain.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/buffer.h> // Transform point clouds
#include <tf2_ros/transform_listener.h>

// Grid map
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

// Pcl
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace mitre_fast_layered_map
{

    struct MapConfiguration
    {
        // SUBSCRIBED TOPICS
        std::string nonGroundPointSubTopic; ///< Ros topic with nonground (obstacle) points
        std::string groundPointSubTopic;    ///< Ros topic with ground points
        std::string odomSubTopic;           ///< Ros topic holding odometry information
        std::string markerSubTopic;         ///< Ros topic to recieve obstacles from other sources
        std::string staticMapSubTopic;      ///< Sub topic for static maps that should be integrated with our dynamic map
        std::string occupancyOutputTopic;   ///< Topic the occupancy map should be output on
        std::string gridmapOutputTopic;     ///< Topic to output the full grid map on

        // MAP CHARACTERISTICS
        std::string mapName;    ///< Name that will refer to this instance of map
        std::string mapFrameId; ///< TF frame that the map should use
        grid_map::Length len;   ///< Size of the map, assumed to be square
        double resolution;      ///< Cell size of map
        std::string historyLayerPrefix; ///< Prefix to use for history layers
        int numHistoryLayers;   ///< Number of layers to hold for history
        

        // VEHICLE CHARACTERISTICS
        // @TODO: Could this be pulled from the robot model automatically?
        // Used to remove lidar points that may be hitting the vehicle
        std::string vehicleFrameId; ///< Frame id of the robot
        double footPrintLen;        ///< Length of robot
        double footPrintWidth;      ///< Width of robot

        // FILTERS
        std::string obstacleFilterNs; ///< Filter chain for filtering sensor data into obstacles
        std::string mapOperationsFilterNs; ///< Filter chain for map operations after obstacles have been determined
        bool enablePointHeightFilter; ///< Whether to filter points by height
        bool enableBoundingBoxFilter; ///< Whether to filter lidar points if they are inside the hitbox of the vehicle
        double maxPointHeight;        ///< If height filter is enabled, filter points more than this height above vehicle

        // SAVING OBSTACLES
        bool enablePermanentObstacles; ///< Whether to try and save permanent obstacles
        double permanentFilterProb;   ///< Probability required to declare a cell as a permanent obstacle.
    };

    inline bool operator==(const MapConfiguration &conf1, const MapConfiguration &conf2)
    {
        return (conf1.nonGroundPointSubTopic == conf2.nonGroundPointSubTopic &&
                conf1.groundPointSubTopic == conf2.groundPointSubTopic &&
                conf1.odomSubTopic == conf2.odomSubTopic &&
                conf1.markerSubTopic == conf2.markerSubTopic &&
                conf1.staticMapSubTopic == conf2.staticMapSubTopic &&
                conf1.occupancyOutputTopic == conf2.occupancyOutputTopic &&
                conf1.mapName == conf2.mapName &&
                conf1.mapFrameId == conf2.mapFrameId &&
                conf1.len(0) == conf2.len(0) &&
                conf1.resolution == conf2.resolution &&
                conf1.historyLayerPrefix == conf2.historyLayerPrefix &&
                conf1.numHistoryLayers == conf2.numHistoryLayers &&
                conf1.vehicleFrameId == conf2.vehicleFrameId &&
                conf1.enableBoundingBoxFilter == conf2.enableBoundingBoxFilter &&
                conf1.footPrintLen == conf2.footPrintLen &&
                conf1.footPrintWidth == conf2.footPrintWidth &&
                conf1.obstacleFilterNs == conf2.obstacleFilterNs &&
                conf1.mapOperationsFilterNs == conf2.mapOperationsFilterNs &&
                conf1.enablePointHeightFilter == conf2.enablePointHeightFilter &&
                conf1.maxPointHeight == conf2.maxPointHeight &&
                conf1.enablePermanentObstacles == conf2.enablePermanentObstacles &&
                conf1.permanentFilterProb == conf2.permanentFilterProb);
    }

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    class SensorMap
    {
    public:
        SensorMap();

        /**
         * @code
         * ros::NodeHandle nh('~');
         * MapConfiguraton config;
         * // Set config
         * mitre_fast_layered_map::Map(&nh, config);
         * @endcode
         */
        SensorMap(ros::NodeHandle *, MapConfiguration);

        ~SensorMap();

        int init();
        int once();

        void odomCb(const nav_msgs::Odometry::ConstPtr);
        int moveMap(double, double);

        void groundPointCb(const sensor_msgs::PointCloud2 &);
        int updateGroundPts(const PointCloud &);

        void nonGroundPointCb(const sensor_msgs::PointCloud2 &);
        int updateNongroundPts(const PointCloud &);
        bool pointHeightFilter(const pcl::PointXYZ, double _vehicleHeight = 0);
        bool pointBoundingBoxFilter(const pcl::PointXYZ);

        int integrateStaticMap();
        void markerCb(const visualization_msgs::Marker &);

        void staticMapCb(const nav_msgs::OccupancyGrid &);

        int getOccupancyGrid(nav_msgs::OccupancyGrid &);
        int publishMap();

        int tfTransformCloud(const sensor_msgs::PointCloud2 &, sensor_msgs::PointCloud2 &, std::string);

        friend class TestMap;

    private:
        int runFilter();

        tf2_ros::Buffer tfBuffer_;              ///< Holds transformations from tf tree
        tf2_ros::TransformListener tfListener_; ///< Used in tfBuffer_
        bool initialized_{false};              // Stateful variable for determining if map has been initialized

        MapConfiguration config_;

        ros::NodeHandle nh_;                 ///< Passed in node handle for namespace
        ros::Subscriber markerSub_;          ///< Subscriber for marker topic to add points to map
        ros::Subscriber odomSub_;            ///< Subscribes to odom topic to position map within frame
        ros::Subscriber nonGroundPointsSub_; ///< Receives point cloud of nonground points
        ros::Subscriber groundPointsSub_;    ///< Receives point cloud of ground points
        ros::Subscriber staticMapSub_;       ///< Will subscripe to the static map topic. ASSUMPTION: Only one static map
        ros::Publisher occPub_;              ///< Publisher of occupancy grid
        ros::Publisher gridMapPub_;          ///< Publisher for the whole grid map using grid map msgs

        grid_map::GridMap gridMap_;          ///< Mapping data storage container
        grid_map::GridMap staticMap_;        ///< Storage for static map. For optimization we only want to receive it once
        filters::FilterChain<grid_map::GridMap> obstacleFilterChain_; ///< Filters to modify obstacle state of map
        filters::FilterChain<grid_map::GridMap> mapOperationsFilterChain_; ///< Filters to modify map after filters in place
        ///< after updating points
        uint64_t readingsReceived_{0}; ///< Records how many readings we have received
        bool cornersCalculated_{false}; ///< Record if we have received an odom update
        std::vector<geometry_msgs::Pose> corners_; ///< Holds corner of vehicle which will create polygon bounding box
        bool recStaticMap_{false}; ///< Stateful variable for receiving static map
        ros::Time odomLastUpdate_;     ///< Keeps track of last time we updated position of map
        ros::Time groundLastUpdate_; ///< Keeps track of the last time we updated ground points
        ros::Time nongroundLastUpdate_; ///< Keeps track of the last time we updated non ground points
    };

    // Class with helper functions for testing that the map above works as expected
    class TestMap
    {

    public:
        TestMap();
        ~TestMap();

        bool CheckConfigEqual(SensorMap &, MapConfiguration &);
        bool CheckMapsEqual(SensorMap &, grid_map::GridMap &);
        bool CheckGeometry(SensorMap &, grid_map::Length, double resolution);
        bool CheckFrame(SensorMap &, std::string);
        bool CheckPosition(SensorMap &, double x, double y);
        bool CheckNans(SensorMap &);
        bool TestMapCells(SensorMap &, std::string, Eigen::MatrixXi);
        bool TestMapCells(SensorMap& , std::string, Eigen::MatrixXf);
    };
} // namespace mitre_fast_layered_map

#endif