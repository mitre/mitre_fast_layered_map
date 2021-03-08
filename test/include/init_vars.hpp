#pragma once

#include <gtest/gtest.h>
#include "grid_map_core/grid_map_core.hpp"
#include "sensor_map.hpp"

class MapTestEnv : public ::testing::Environment
{
public:

    MapTestEnv();

    mitre_fast_layered_map::TestMap testMap;

    mitre_fast_layered_map::MapConfiguration defaultConfig_;
    mitre_fast_layered_map::MapConfiguration smallMap_;
};