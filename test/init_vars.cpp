#include "init_vars.hpp"

MapTestEnv::MapTestEnv()
{
    defaultConfig_.obstacleFilterNs = "mitre_fast_layered_map_obstacle_filters";
    defaultConfig_.mapOperationsFilterNs = "mitre_fast_layered_map_map_operations";
    defaultConfig_.mapFrameId = "odom";
    defaultConfig_.vehicleFrameId = "base_link";
    defaultConfig_.gridmapOutputTopic = "gridmap";
    defaultConfig_.occupancyOutputTopic = "costmap";
    defaultConfig_.groundPointSubTopic = "ground";
    defaultConfig_.nonGroundPointSubTopic = "ground";
    defaultConfig_.markerSubTopic = "marker";
    defaultConfig_.odomSubTopic = "odom";
    defaultConfig_.staticMapSubTopic = "staticmap";
    defaultConfig_.len = grid_map::Length(50, 50);
    defaultConfig_.resolution = 0.5;
    defaultConfig_.historyLayerPrefix = "history_";
    defaultConfig_.numHistoryLayers = 10;
    defaultConfig_.enableBoundingBoxFilter = false;
    defaultConfig_.footPrintLen = 0;
    defaultConfig_.footPrintWidth = 0;
    defaultConfig_.enablePointHeightFilter = false;
    defaultConfig_.maxPointHeight = 2;
    defaultConfig_.enablePermanentObstacles = false;
    defaultConfig_.permanentFilterProb = 0.999;

    smallMap_.obstacleFilterNs = "none";
    smallMap_.mapOperationsFilterNs = "none";
    smallMap_.mapFrameId = "odom";
    smallMap_.vehicleFrameId = "base_link";
    smallMap_.gridmapOutputTopic = "gridmap";
    smallMap_.occupancyOutputTopic = "costmap";
    smallMap_.groundPointSubTopic = "ground";
    smallMap_.nonGroundPointSubTopic = "ground";
    smallMap_.markerSubTopic = "marker";
    smallMap_.odomSubTopic = "odom";
    smallMap_.staticMapSubTopic = "staticmap";
    smallMap_.len = grid_map::Length(5, 5);
    smallMap_.resolution = 1;
    smallMap_.historyLayerPrefix = "history_";
    smallMap_.numHistoryLayers = 10;
    smallMap_.enableBoundingBoxFilter = false;
    smallMap_.footPrintLen = 0;
    smallMap_.footPrintWidth = 0;
    smallMap_.enablePointHeightFilter = false;
    smallMap_.maxPointHeight = 2;
    smallMap_.enablePermanentObstacles = false;
    smallMap_.permanentFilterProb = 0.999;

}
