#pragma once

#include <gtest/gtest.h>
#include "grid_map_core/grid_map_core.hpp"
#include "sensor_map.hpp"

class MapTestEnv : public ::testing::Environment
{
public:

    MapTestEnv();

    t_mapping::TestMap testMap;

    t_mapping::MapConfiguration defaultConfig_;
    t_mapping::MapConfiguration smallMap_;
};