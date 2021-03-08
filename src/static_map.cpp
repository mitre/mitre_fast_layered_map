#include "static_map.hpp"

namespace mitre_fast_layered_map
{

    StaticMap::StaticMap() : tfListener_(tfBuffer_)
    {}

    StaticMap::StaticMap(ros::NodeHandle *_nh, StaticMapConfiguration _config) : nh_(*_nh),
                                                                                 config_(_config),
                                                                                 gridMap_({"static_map"}),
                                                                                 tfListener_(tfBuffer_)
    {
    }

    StaticMap::~StaticMap() {}

    int StaticMap::init()
    {
        // Subscribers
        
        // Required
        staticMapSub_ = nh_.subscribe(config_.staticMapSubTopic, 1, &StaticMap::staticMapCb, this);

        // Optional
        if (config_.markerSubTopic != "")
        {
            markerSub_ = nh_.subscribe(config_.markerSubTopic, 1, &StaticMap::mapMarkerCb, this);
        }
            
        if (config_.gridMapSubTopic != "")
        {
            gridMapSub_ = nh_.subscribe(config_.gridMapSubTopic, 1, &StaticMap::gridMapCb, this);
        }
            

        // Publishers
        occGridPub_ = nh_.advertise<nav_msgs::OccupancyGrid>(config_.occupancyOutputTopic, 1);
        gridMapPub_ = nh_.advertise<grid_map_msgs::GridMap>(config_.gridmapOutputTopic, 1);

        initialized_ = true;
        return 0;
    }

    int StaticMap::once()
    {
        // Make the class has been initialized
        if (!initialized_)
        {
            return 1;
        }
        else if (!recStaticMap_) // Don't publish unless a static map has been received
        {
            return 2;
        }

        publishMap();

        return 0;
    }

    void StaticMap::staticMapCb(const nav_msgs::OccupancyGrid &_occGrid)
    {
        // Only get the static map once
        if (recStaticMap_)
        {
            return;
        }

        // Update our gridmap to be the same as the received static map
        grid_map::GridMapRosConverter::fromOccupancyGrid(_occGrid, "static_map", gridMap_);

        // Grid map seems to be using nans as obstacles, so we make all nans 100
        gridMap_["static_map"] = (gridMap_["static_map"].array().isNaN()).select(100, gridMap_["static_map"]);

        // Logging information about the received gridmap
        ROS_INFO("Static Map received!");

        grid_map::Size size = gridMap_.getSize();
        grid_map::Position pos = gridMap_.getPosition();

        ROS_INFO("Map Size: %d, %d", size(0, 0), size(1, 0));
        ROS_INFO("Resolution: %f", gridMap_.getResolution());
        ROS_INFO("Position: %f, %f", pos(0), pos(1));

        recStaticMap_ = true;
    }

    void StaticMap::mapMarkerCb(const visualization_msgs::Marker &_marker)
    {
        ROS_INFO("Recieved marker!");

        // TODO: If published in the ned frame we should switch the x and y before we look at it in utm_enu frame
        // If we have a ned frame published then we might be okay.
        if (!recStaticMap_)
        {
            ROS_WARN("Static map has not been received yet. Ignoring marker.");
            return;
        }

        // For now, assume the marker is in utm_enu and is operating on the static map
        // Make sure the point is inside the map
        grid_map::Position pos(_marker.pose.position.x, _marker.pose.position.y);
        if (!gridMap_.isInside(pos))
        {
            ROS_WARN("Recieved Marker not inside of map.");
            return;
        }

        // ASSUMPTION: Obstacle is a circle/sphere
        double radius = _marker.scale.x;
        for (grid_map::CircleIterator it(gridMap_, pos, radius); !it.isPastEnd(); ++it)
        {
            gridMap_.at("static_map", *it) = 100;
        }
    }

    void StaticMap::gridMapCb(const grid_map_msgs::GridMap &_inMap)
    {
        ROS_DEBUG_THROTTLE(1,"Received grid map");
        // TO BE IMPLEMENTED
        // Update static map with obstacles from a grid map instance
        grid_map::GridMap tempMap;
        grid_map::GridMapRosConverter::fromMessage(_inMap, tempMap);

        // We need to determine the transform between the two maps
        geometry_msgs::TransformStamped transform;
        try
        {
            transform = tfBuffer_.lookupTransform(gridMap_.getFrameId(), tempMap.getFrameId(), ros::Time(0));
        }
        catch (const std::exception &e)
        {
            ROS_WARN("Unable to find transform: %s. Not including map in permanent obstacles.", e.what());
            return;
        }

        // FOR NOW WE ASSUME A PERMANENT LAYER
        grid_map::Matrix &permanent = tempMap[config_.gridMapLayer];

        // Layer to add obstacles to
        grid_map::Matrix &staticMap = gridMap_["static_map"];

        for (grid_map::GridMapIterator it(tempMap); !it.isPastEnd(); ++it)
        {
            const grid_map::Index index(*it);

            // We only care if the point is an actual obstacle
            if (permanent(index(0), index(1)) != 100)
            {
                continue;
            }

            grid_map::Position pos;
            tempMap.getPosition(index, pos);

            // Convert the position into the gridMap_ frame
            geometry_msgs::Pose poseIn;
            poseIn.position.x = pos.x();
            poseIn.position.y = pos.y();

            geometry_msgs::Pose poseOut;

            tf2::doTransform(poseIn, poseOut, transform);

            grid_map::Position gPos(poseOut.position.x, poseOut.position.y);

            // Remember, we found about this point holds an obstacle in tempMap
            if (gridMap_.isInside(gPos))
            {
                // Is it faster to get the index and operate on the matrix, or just use the line below?
                gridMap_.atPosition("static_map", gPos) = 100;
                // grid_map::Index gIndex;
                // gridMap_.getIndex(pos, gIndex);
                // staticMap(gIndex(0), gIndex(1)) = 100; // Set a static obstacle cell
            }
        }
    }

    int StaticMap::publishMap()
    {
        // Occupancy grid publisher
        nav_msgs::OccupancyGrid occGrid;
        grid_map::GridMapRosConverter::toOccupancyGrid(gridMap_, "static_map", 0, 100, occGrid);

        occGridPub_.publish(occGrid);

        // Grid map message publisher
        grid_map_msgs::GridMap gridMapMessage;
        grid_map::GridMapRosConverter::toMessage(gridMap_, gridMapMessage);
        gridMapPub_.publish(gridMapMessage);
    }

} // namespace mitre_fast_layered_map