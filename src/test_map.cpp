/**
 * @file test_map.cpp
 * @brief Implementation file for testing map implementation
 * @author Trevor Bostic
 */

#include "sensor_map.hpp"

namespace mitre_fast_layered_map
{
    TestMap::TestMap()
    {}

    TestMap::~TestMap()
    {}

    bool TestMap::CheckMapsEqual(SensorMap& _map, grid_map::GridMap& _answerMap)
    {
        std::cout << "Inside of check map equal" << std::endl;
    }

    bool TestMap::CheckConfigEqual(SensorMap& _map, MapConfiguration& _config)
    {
        MapConfiguration mapConfig = _map.config_;
        return mapConfig == _config;
    }

    bool TestMap::CheckGeometry(SensorMap& _map, grid_map::Length _len, double _resolution)
    {
        grid_map::Length mapLen = _map.gridMap_.getLength();

        return (mapLen(0) == _len(0)) &&
            (mapLen(1) == _len(1)) &&
            (_map.gridMap_.getResolution() == _resolution);
    }

    bool TestMap::CheckFrame(SensorMap& _map, std::string _frameId)
    {
        return _map.gridMap_.getFrameId() == _frameId;
    }

    bool TestMap::CheckPosition(SensorMap& _map, double _x, double _y)
    {
        grid_map::Position pos = _map.gridMap_.getPosition();
        return (pos(0) == _x) && (pos(1) == _y);
    }

    bool TestMap::CheckNans(SensorMap& _map)
    {
        grid_map::GridMap gridMap = _map.gridMap_;
        for(grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it)
        {
            // Check ground, nonground, and permanent layers for nan
            const grid_map::Index index(*it);

            if (std::isnan(gridMap.at("ground", index)) || std::isnan(gridMap.at("nonground", index)) ||
                std::isnan(gridMap.at("permanent", index)))
            {
                return false; // Found a nan
            }
        }

        return true;
    }

    bool TestMap::TestMapCells(SensorMap& _map, std::string _layer, Eigen::MatrixXi _answerMat)
    {

        for (grid_map::GridMapIterator it(_map.gridMap_); !it.isPastEnd(); ++it)
        {
            const grid_map::Index index(*it);

            if (_map.gridMap_.at(_layer, index) != _answerMat(index(0), index(1)))
            {
                // std::cerr << "Index " << index(0) << "," << index(1) << std::endl;
                std::cerr << "Map: " << _map.gridMap_[_layer] << std::endl;
                std::cerr << "Answer Mat: " << _answerMat << std::endl;
                return false;
            }
        }

        return true;
    }

    bool TestMap::TestMapCells(SensorMap& _map, std::string _layer, Eigen::MatrixXf _answerMat)
    {

        for (grid_map::GridMapIterator it(_map.gridMap_); !it.isPastEnd(); ++it)
        {
            const grid_map::Index index(*it);

            if (_map.gridMap_.at(_layer, index) != _answerMat(index(0), index(1)))
            {
                // std::cerr << "Index " << index(0) << "," << index(1) << std::endl;
                // std::cerr << "Map: " << _map.gridMap_[_layer] << std::endl;
                // std::cerr << "Answer Mat: " << _answerMat << std::endl;
                return false;
            }
        }

        return true;
    }
}