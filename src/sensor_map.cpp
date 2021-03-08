/**
 * @file sensor_map.cpp
 * @brief Provides functionality for rapidly changing local maps
 * @author Trevor Bostic
 */

#include "sensor_map.hpp"
#include <chrono>

namespace mitre_fast_layered_map
{

    // Static variable definition
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener Map::tfListener_(Map::tfBuffer_);

    /**
     * @fn SensorMap::SensorMap
     * @brief Constructor to initialize mapping
     * @param nh Node handle to use for this map instance
     * @param config Map configuraton variables
     */
    SensorMap::SensorMap(ros::NodeHandle *nh, MapConfiguration config) : nh_(*nh),
                                                                         config_(config),
                                                                         gridMap_({"ground", "nonground", "permanent", "vehicle_hitbox",
                                                                                   "current_probability", "occupancy", "elevation_min"}), // The layers should be parameterized at some point
                                                                         staticMap_({"map"}),
                                                                         obstacleFilterChain_("grid_map::GridMap"),
                                                                         mapOperationsFilterChain_("grid_map::GridMap"),
                                                                         tfListener_(tfBuffer_)
    {
    }

    // // Don't use default constructor
    SensorMap::SensorMap() : config_(),
                             obstacleFilterChain_("grid_map::GridMap&"),
                             mapOperationsFilterChain_("grid_map::GridMap&"),
                             tfListener_(tfBuffer_)
    {
    }

    SensorMap::~SensorMap()
    {
    }

    /**
     * @fn SensorMap::Init()
     * @brief Sets up the grid map and the filter chain with configuratoin settings
     * @returns < 0 if failure, 0 if successful
     */
    int SensorMap::init()
    {
        gridMap_.setFrameId(config_.mapFrameId);
        gridMap_.setGeometry(config_.len, config_.resolution);

        // We constantly fight nans with gridmap. Here we remove the nans
        // it initializes with. In the moveMap function we remove any nans that
        // appear as they can affect other functions and we don't want to handle
        // them everywhere
        gridMap_["ground"].setConstant(20.0);
        gridMap_["nonground"].setConstant(0.0);
        gridMap_["permanent"].setConstant(0.0);
        gridMap_["vehicle_hitbox"].setConstant(0.0);
        gridMap_["current_probability"].setConstant(0.5);
        gridMap_["elevation_min"].setConstant(FLT_MAX);

        // Calculate vehicle corners given the length and width
        // We calculate as a function of theta for ease when turning within map
        // see movemap function
        // Distance from center of vehicle to any corner
        // inscribedRadius_ = sqrt(pow(config_.footPrintLen, 2) + pow(config_.footPrintWidth, 2));
        // centToFrontLeftTheta_ = acos((config_.footPrintLen / 2) / inscribedRadius_);

        // Add corners in the vehicle's frame. We will transform these into the grid maps frame in MoveMap
        geometry_msgs::Pose frontLeft;
        frontLeft.position.x = config_.footPrintLen / 2;
        frontLeft.position.y = config_.footPrintWidth / 2; // Remember FLU, so left is positive
        geometry_msgs::Pose frontRight;
        frontRight.position.x = config_.footPrintLen / 2;
        frontRight.position.y = -(config_.footPrintWidth / 2);
        geometry_msgs::Pose backLeft;
        backLeft.position.x = -(config_.footPrintLen / 2);
        backLeft.position.y = config_.footPrintWidth / 2;
        geometry_msgs::Pose backRight;
        backRight.position.x = -(config_.footPrintLen / 2);
        backRight.position.y = -(config_.footPrintWidth / 2);
        // Order turns out to be important due to how the polygon iterator follows the vertexes
        corners_.push_back(frontLeft);
        corners_.push_back(frontRight);
        corners_.push_back(backRight);
        corners_.push_back(backLeft);

        // Add the number of history layers that are configured
        for (int i = 0; i < config_.numHistoryLayers; i++)
        {
            // Set to -1 to show now readings recorded
            gridMap_.add(config_.historyLayerPrefix + std::to_string(i), -1);
        }

        // Register the filter chains we want to use
        if (!obstacleFilterChain_.configure(config_.obstacleFilterNs, nh_))
        {
            ROS_ERROR("Unable to configure obstacle filter chain");
            return -1;
        }

        if (!mapOperationsFilterChain_.configure(config_.mapOperationsFilterNs, nh_))
        {
            ROS_ERROR("Unable to configure map operations filter chain");
            return -1;
        }

        // Used to determine how often to update map position
        odomLastUpdate_ = ros::Time::now();
        groundLastUpdate_ = ros::Time::now();
        nongroundLastUpdate_ = ros::Time::now();

        // Required topics
        odomSub_ = nh_.subscribe(config_.odomSubTopic, 1, &SensorMap::odomCb, this);
        nonGroundPointsSub_ = nh_.subscribe(config_.nonGroundPointSubTopic, 1, &SensorMap::nonGroundPointCb, this);
        groundPointsSub_ = nh_.subscribe(config_.groundPointSubTopic, 1, &SensorMap::groundPointCb, this);

        // Optional topics
        if (config_.staticMapSubTopic != "")
        {
            staticMapSub_ = nh_.subscribe(config_.staticMapSubTopic, 1, &SensorMap::staticMapCb, this);
        }

        if (config_.markerSubTopic != "")
        {
            markerSub_ = nh_.subscribe(config_.markerSubTopic, 1, &SensorMap::markerCb, this);
        }

        occPub_ = nh_.advertise<nav_msgs::OccupancyGrid>(config_.occupancyOutputTopic, 1);
        gridMapPub_ = nh_.advertise<grid_map_msgs::GridMap>(config_.gridmapOutputTopic, 1);

        initialized_ = true;

        return 0;
    }

    /**
     * @fn SensorMap::Once
     * @brief Instantiates subscribers and publishers to begin processing
     * @returns < 0 if error, 0 if running
     */
    int SensorMap::once()
    {
        if (!initialized_)
        {
            ROS_ERROR("SensorMap %s has not yet been initialized successfully. Cannot run.", config_.mapName.c_str());
            return -1;
        }

        // All we need to do is publish!
        publishMap();

        return 0;
    }

    /**
     * @fn SensorMap::odomCb
     * @brief Odometry callback function to update map position
     * @param odom Odometry message used to move map
     */
    void SensorMap::odomCb(const nav_msgs::Odometry::ConstPtr odom)
    {

        // Update at most every 1/10 of a second
        // TODO: Add update time as configurable variable
        if (odomLastUpdate_.toNSec() - ros::Time::now().toNSec() < 100000000)
        {
            ROS_DEBUG("Not long enough since last odom update. Ignoring.");
            return;
        }

        geometry_msgs::Pose pose = odom->pose.pose;
        ROS_DEBUG_THROTTLE(1, "Moving local map to postion: %f, %f", pose.position.x, pose.position.y);

        odomLastUpdate_ = ros::Time::now(); // Record time of udpate
        if (moveMap(pose.position.x, pose.position.y) > 0)
        {
            ROS_WARN("Map Jumped!");
        }
    }

    /**
     * @fn SensorMap::moveMap
     * @brief Move the center of the map to follow ego
     * @param _posX [in] Position x in map frame to move to
     * @param _posY [in] Position y in map frame to move to
     * 
     * @return 0 on successful move, -1 on failure, 1 if map move caused jump out of mapped area
     */
    int SensorMap::moveMap(double _posX, double _posY)
    {
        // PROFILE: ~<1ms with 10 history layers
        ROS_DEBUG("Setting uncovered cells.");

        // We have had errors before due to our map "jumping", usuall
        // at the very beginning when we start 0,0 then jump. Just a way
        // to notify that this has occured
        int mapInCurrentBounds = 0;
        if (!gridMap_.isInside(grid_map::Position(_posX, _posY)))
        {
            mapInCurrentBounds = 1;
        }

        gridMap_.move(grid_map::Position(_posX, _posY));

        // Cells that may have been uncovered due to the move will be set to nans
        // which can cause unforseen errors, thus we remove all nans and put in appropriate values

        grid_map::Matrix &ground = gridMap_["ground"];
        grid_map::Matrix &elevationMin = gridMap_["elevation_min"];
        grid_map::Matrix &nonground = gridMap_["nonground"];
        grid_map::Matrix &permanent = gridMap_["permanent"];
        grid_map::Matrix &currentProb = gridMap_["current_probability"];

        // TODO: Faster to use select or to do a loop over gridmap and set each cell?

        // For any unfamiliar, this is equivalent to if isnan ? set num : keep value
        ground = (ground.array().isNaN()).select(-1, ground); // 20 is unknown
        elevationMin = (elevationMin.array().isNaN()).select(FLT_MAX, elevationMin); // Set max if no min recorded
        nonground = (nonground.array().isNaN()).select(0, nonground); // 0 is clear
        permanent = (permanent.array().isNaN()).select(0, permanent); // 0 is clear
        currentProb = (currentProb.array().isNaN()).select(0.5, currentProb); // 0.5 for don't know

        // New history cells set to -1 for no readings
        for (int i = 0; i < config_.numHistoryLayers; i++)
        {
            grid_map::Matrix &layer = gridMap_[config_.historyLayerPrefix + std::to_string(i)];
            layer = (layer.array().isNaN()).select(-1, layer);
        }

        // Calculate the current hitbox for the vehicle given its current position and orientation
        grid_map::Matrix &vehicleHitBox = gridMap_["vehicle_hitbox"];
        vehicleHitBox.setConstant(0); 

        try
        {
            geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(config_.mapFrameId, config_.vehicleFrameId, ros::Time(0));

            // Convert vehicle corners into grid map frame and then draw a polygone
            grid_map::Polygon poly;
            poly.setFrameId(gridMap_.getFrameId());

            geometry_msgs::Pose tempPose;
            for(geometry_msgs::Pose pose : corners_)
            {
                tf2::doTransform(pose, tempPose, transform);
                poly.addVertex(grid_map::Position(tempPose.position.x, tempPose.position.y));
            }

            // Iterate on polygon to set hitbox
            for(grid_map::PolygonIterator it(gridMap_, poly); !it.isPastEnd(); ++it)
            {
                const grid_map::Index index(*it);
                vehicleHitBox(index(0), index(1)) = 100; // Set as collision
            }

        }
        catch (const std::exception &e)
        {
            ROS_WARN("Unable to find transform: %s. Disabling bounding box filter for this iteration.", e.what());
        }

        return mapInCurrentBounds;
    }

    /**
     * @fn SensorMap::groundPointCb
     * @brief Callback function for ground points
     * @param _cloud A point cloud of ONLY points labelled as part of the ground plane
     */
    void SensorMap::groundPointCb(const sensor_msgs::PointCloud2 &_cloud)
    {
        // Check if it has been long enough since the last update
        if (nongroundLastUpdate_.toNSec() - ros::Time::now().toNSec() < 100000000)
        {
            ROS_DEBUG("Not long enough since last nonground point update. Ignoring.");
            return;
        }

        // Check if the transform is possible
        if (!tfBuffer_.canTransform(config_.mapFrameId, _cloud.header.frame_id, ros::Time(0), ros::Duration(0.2)))
        {
            ROS_WARN("Unable to find transform from %s to %s.", config_.mapFrameId.c_str(), _cloud.header.frame_id.c_str());
            return;
        }

        // Convert the pcl point cloud into a ros point cloud and transform
        sensor_msgs::PointCloud2 tfCloud;
        tfTransformCloud(_cloud, tfCloud, config_.mapFrameId);

        // Convert back to pcl cloud
        PointCloud conCloud;
        pcl::fromROSMsg(tfCloud, conCloud);

        updateGroundPts(conCloud);
        nongroundLastUpdate_ = ros::Time::now();
    }

    /**
     * @fn SensorMap::updateGroundPts
     * @brief Records cells in the gridmap where the ground points landed.
     * @param _pointCloud [in] Point cloud of points on the ground plane
     * 
     * @returns 0 on success, 1 if problem with filters
     */
    int SensorMap::updateGroundPts(const PointCloud &_pointCloud)
    {
        grid_map::Matrix &ground = gridMap_["ground"];
        ground.setConstant(0.0);
        grid_map::Matrix &elevation_min = gridMap_["elevation_min"];
        grid_map::Index index;

        // Update ground cells with
        for (auto it = _pointCloud.begin(); it < _pointCloud.end(); it++)
        {

            // May be able to use isInside and atPosition function instead
            if (gridMap_.getIndex(grid_map::Position(it->x, it->y), index))
            {
                // Record as ground point
                ground(index(0), index(1)) += 1;

                // Record minimum elevation within the cell
                if (std::isnan(elevation_min(index(0), index(1))) || it->z < elevation_min(index(0), index(1)))
                {
                    elevation_min(index(0), index(1)) = it->z;
                }
            }
        }

        return 0;
    }

    /**
     * @fn SensorMap::nonGroundPointCb
     * @brief Callback function for nonground points
     * @param _cloud Point Cloud holding ONLY points that registered as nonground (i.e. obstacles or ledges)
     */
    void SensorMap::nonGroundPointCb(const sensor_msgs::PointCloud2 &_cloud)
    {

        // Determine if the transform is possible
        if (!tfBuffer_.canTransform(config_.mapFrameId, _cloud.header.frame_id, ros::Time(0), ros::Duration(0.2)))
        {
            ROS_WARN("Unable to find transform from %s to %s.", config_.mapFrameId.c_str(), _cloud.header.frame_id.c_str());
            return;
        }

        sensor_msgs::PointCloud2 tfCloud;
        tfTransformCloud(_cloud, tfCloud, config_.mapFrameId);

        PointCloud conCloud;
        pcl::fromROSMsg(tfCloud, conCloud);

        updateNongroundPts(conCloud);

    }

    /**
     * @fn SensorMap::updateNongroundPts
     * @brief Updates cells in the map that may have nonground (obstacle) points
     * @param _pointCloud [in] The point cloud consisting of only nonground points
     * 
     * @return 0 on success, 1 if filter failure
     */
    int SensorMap::updateNongroundPts(const PointCloud &_pointCloud)
    {

        // PROFILING: ~5ms to add lidar points
        // Determine if the height filter can and should run
        // Lets find transform between the grid map frame and the robot/vehicle frame
        double vehicleHeight = 0;
        bool runHeightFilter = config_.enablePointHeightFilter;

        if (runHeightFilter)
        {
            try
            {
                geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(config_.mapFrameId, config_.vehicleFrameId, ros::Time(0));
                vehicleHeight = transform.transform.translation.z;
                // ROS_INFO("Transform Z: %f", vehicleHeight);
            }
            catch (const std::exception &e)
            {
                ROS_WARN("Unable to find transform: %s. Disabling height filter for this iteration.", e.what());
                runHeightFilter = false;
            }
        }

        grid_map::Index index;
        // Determine history layer to use for this reading. Cycles through all history layers
        std::string historyLayerStr = config_.historyLayerPrefix + std::to_string(readingsReceived_ % config_.numHistoryLayers);
        grid_map::Matrix& historyLayer = gridMap_[historyLayerStr];
        // Layer is being overwritten, so set to all 0's
        historyLayer.setConstant(0);

        // Point cloud will be in map frame
        for (auto it = _pointCloud.begin(); it < _pointCloud.end(); it++)
        {
            // Don't consider points outside of the map
            if (!gridMap_.getIndex(grid_map::Position(it->x, it->y), index))
            {
                continue;
            }

            // Determine if the point is too high above the vehicle to be considered an obstacle
            if (runHeightFilter && pointHeightFilter(*it, vehicleHeight))
            {
                continue; // Ignore the point and continue
            }
            else if (config_.enableBoundingBoxFilter && pointBoundingBoxFilter(*it)) // Check if the point may be hitting the vehicle
            {
                continue;
            }

            // Point in bounds and not filtered out, record it!
            historyLayer(index(0), index(1)) += 1;
        }

        if (runFilter() != 0)
        {
            return 1;
        }

        if (config_.enablePermanentObstacles)
        {
            // Add objects to permanent layer
            grid_map::Matrix tempMat = gridMap_["current_probability"].unaryExpr([this](const float x) { 
                if (x >= config_.permanentFilterProb) 
                {
                    return (float)100;
                }
                return (float)0.0;
            });
            gridMap_["permanent"] = gridMap_["permanent"] + tempMat;
        }

        // Only count readings that we succesfully filtered
        readingsReceived_++;
        return 0;
    }

    bool SensorMap::pointHeightFilter(const pcl::PointXYZ _point, double _vehicleHeight)
    {
        /*
        * Determine if the point is too high for consideration. This often catches objects such as
        * bridges or tree branches that may register as obstacles but be safely above the vehicle.
        */
        if (_point.z > (config_.maxPointHeight + _vehicleHeight))
        {
            // ROS_INFO("Height Filter Point: %f, max height allowed: %f", point.z, (config_.maxPointHeight + transform.transform.translation.z));
            return true;
        }

        return false;
    }

    bool SensorMap::pointBoundingBoxFilter(const pcl::PointXYZ point)
    {
        // Check if point is in cell of our vehicle bounding box
        if (gridMap_.atPosition("vehicle_hitbox", grid_map::Position(point.x, point.y)) == 100)
        {
            return true;
        }

        return false;
    }

    /**
     * @fn SensorMap::runFilter
     * @brief Run the grid map through the filter chain
     * 
     * @return 0 on success, -1 if failure within filter chain
     */
    int SensorMap::runFilter()
    {
        // Reset nonground layer to empty, the filters will build obstacles based off past history
        gridMap_["nonground"].setConstant(0);

        // Currently taking ~25ms for local map
        if (!obstacleFilterChain_.update(gridMap_, gridMap_))
        {
            ROS_ERROR("Unable to run obstacle filter chain.");
            return -1;
        }

        // Add any obstacles that have been determined to be permanent
        if (config_.staticMapSubTopic != "") 
        {
            integrateStaticMap(); // Will add obstacles in static map to gridMap
        }

        gridMap_["nonground"] = gridMap_["nonground"] + gridMap_["permanent"];
        // nonground, permanent, and static map may all share obstacles, adding them together
        // will result in cells >100, set them to 0
        gridMap_["nonground"] = (gridMap_["nonground"].array() > 100).select(100, gridMap_["nonground"]);

        // Runs in ~3ms
        // Run any operations such as inflation or ray tracing on obstacle map
        if (!mapOperationsFilterChain_.update(gridMap_, gridMap_))
        {
            ROS_ERROR("Unable to run map operations filter chain.");
            return -1;
        }

        return 0;
    }

    /**
     * @fn SensorMap::IntegrateStaticMap
     * @brief Adds any obstacles from a received static map into the current running map instance
     * @returns 1 = no static map received, 2 = transform not found, 0 = success
     */
    int SensorMap::integrateStaticMap()
    {
        if (!recStaticMap_)
        {
            return 1; // Haven't received a map so nothing to do
        }

        ROS_DEBUG_THROTTLE(1, "Integrating static map.");

        // Run the map server and add mark any cells that should be occupied
        geometry_msgs::TransformStamped tfStamp;

        // Need to wait for the tf buffer to fill up.
        try
        {
            // ROS_DEBUG_THROTTLE(1, "Trying to transform from: %s to %s", _inCloud.header.frame_id.c_str(), _outFrame.c_str());
            tfStamp = tfBuffer_.lookupTransform(staticMap_.getFrameId(), gridMap_.getFrameId(), ros::Time(0));
        }
        catch (const std::exception &e)
        {
            ROS_WARN("Unable to perform transform: %s", e.what());
            return 2; // No error, but just not able to add the static map
        }

        // grid_map::Matrix& staticMapLayer = staticMap_["map"];
        grid_map::Matrix &gridMapLayer = gridMap_["permanent"];

        geometry_msgs::PoseStamped gridMapStamped;
        gridMapStamped.header.frame_id = gridMap_.getFrameId();
        gridMapStamped.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped staticMapStamped;

        for (grid_map::GridMapIterator it(gridMap_); !it.isPastEnd(); ++it)
        {
            const grid_map::Index index(*it);

            // Get the position of the current cell in grid map
            grid_map::Position pos;
            gridMap_.getPosition(index, pos);

            gridMapStamped.pose.position.x = pos.x();
            gridMapStamped.pose.position.y = pos.y();

            // Transform the position into the static map frame
            tf2::doTransform(gridMapStamped, staticMapStamped, tfStamp);

            grid_map::Position staticMapPos(staticMapStamped.pose.position.x, staticMapStamped.pose.position.y);

            // Check if the position is in the map and the map has registered it as an obstacle
            if (staticMap_.isInside(staticMapPos) && staticMap_.atPosition("map", staticMapPos) == 100)
            {
                gridMapLayer(index(0), index(1)) = 100;
            }
        }

        return 0;
    }

    /**
     * @fn SensorMap::MarkerCb
     * @brief Call back to add obstacles to map from a given marker message
     * @param marker Message sent with Pose and stamp to add to map
     */
    void SensorMap::markerCb(const visualization_msgs::Marker &marker)
    {
        ROS_INFO("Recieved marker!");

        // TODO: If published in the ned frame we should switch the x and y before we look at it in utm_enu frame
        // If we have a ned frame published then we might be okay.

        // Make sure the point is inside the map
        grid_map::Position pos(marker.pose.position.x, marker.pose.position.y);
        if (!gridMap_.isInside(pos))
        {
            ROS_WARN("Recieved Marker not inside of map.");
            return;
        }

        // ASSUMPTION: Obstacle is a circle/sphere
        double radius = marker.scale.x;
        for (grid_map::CircleIterator it(gridMap_, pos, radius); !it.isPastEnd(); ++it)
        {
            gridMap_.at("permanent", *it) = 100;
        }
    }

    /**
     * @fn SensorMap::StaticMapCb
     * @brief Callback function to record static map
     * @param _map SensorMap to record
     */
    void SensorMap::staticMapCb(const nav_msgs::OccupancyGrid &_map)
    {
        if (recStaticMap_) // Only getting the map once for now
        {
            return;
        }

        // Turn the occupancy grid into a grid map message
        grid_map::GridMapRosConverter::fromOccupancyGrid(_map, "map", staticMap_);

        // Grid map seems to be using nans as obstacles, so we make all nans 100
        staticMap_["map"] = (staticMap_["map"].array().isNaN()).select(100, staticMap_["map"]);

        grid_map::Position pos = staticMap_.getPosition();
        grid_map::Size size = staticMap_.getSize();

        // DEBUG Stuff
        ROS_INFO("Recieved static map.");
        ROS_INFO("Map Size: %d, %d", size(0, 0), size(1, 0));
        ROS_INFO("Resolution: %f", staticMap_.getResolution());
        ROS_INFO("Position: %f, %f", pos(0), pos(1));

        recStaticMap_ = true; // Nofity we have received the map
    }

    /**
     * @fn SensorMap::getOccupancyGrid
     * @brief Compiles the ground and nonground layers to create an occupancy grid
     * @param _mapOut [out] The occupancy grid to return
     * 
     * @return 0
     */
    int SensorMap::getOccupancyGrid(nav_msgs::OccupancyGrid &_mapOut)
    {
        gridMap_["occupancy"] = gridMap_["nonground"];
        grid_map::GridMapRosConverter::toOccupancyGrid(gridMap_, "occupancy", 0, 100, _mapOut);

        return 0;
    }

    /**
     * @fn SensorMap::PublishMap
     * @brief Helper function for creating and publishing occupancy grid of the local map
     * 
     * @return 0
     */
    int SensorMap::publishMap()
    {
        // PROFILE: ~2ms with 10 history layers
        ROS_DEBUG_THROTTLE(1, "Publishing map.");

        // Publish basic occupancy map
        nav_msgs::OccupancyGrid message;
        getOccupancyGrid(message);

        message.header.frame_id = config_.mapFrameId;
        message.header.stamp = ros::Time::now();

        occPub_.publish(message);

        // Publish the whole gridmap if anyone wants it
        ROS_DEBUG_THROTTLE(1, "Publishing grid map message");
        gridMap_.setTimestamp(ros::Time::now().toNSec());
        grid_map_msgs::GridMap gridMapMessage;
        grid_map::GridMapRosConverter::toMessage(gridMap_, gridMapMessage);
        gridMapPub_.publish(gridMapMessage);

        return 0;
    }

    /**
     * @fn SensorMap::TfTransformCloud
     * @brief Transform helper function to help transform point clouds into the necessary frame.
     * @param _inCloud [in] The lidar point cloud in the lidars frame
     * @param _outCloud [out] The point cloud transformed into _outFrame
     * @param _outFrame [in] The frame to transform the point cloud into
     * 
     * @return 0 on success, -1 on transform error
     */
    int SensorMap::tfTransformCloud(const sensor_msgs::PointCloud2 &_inCloud, sensor_msgs::PointCloud2 &_outCloud, std::string _outFrame)
    {
        // Important to catch exception, since this call usually fails the first couple of times for some reason
        // I would guess the tfBuffer is still getting updated before we call this and doesn't contain the transform
        // geometry_msgs::TransformStamped tfStamped;
        try
        {
            ROS_DEBUG_THROTTLE(1, "Trying to transform from: %s to %s", _inCloud.header.frame_id.c_str(), _outFrame.c_str());
            tfBuffer_.transform(_inCloud, _outCloud, _outFrame, ros::Duration(0.3));
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            return -1;
        }

        return 0;
    }

} // namespace mitre_fast_layered_map
