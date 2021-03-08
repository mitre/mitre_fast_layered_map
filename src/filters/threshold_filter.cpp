/**
 * @file bayes_update.cpp
 * @brief Accumulates probability a cell is occupied
 * @author Trevor Bostic
 */
#include "filters.hpp"
#include <iostream>

#include <grid_map_core/GridMap.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace mitre_fast_layered_map
{

    ThresholdFilter::ThresholdFilter()
    {
    }

    ThresholdFilter::~ThresholdFilter()
    {
    }

    bool ThresholdFilter::configure()
    {
        // Load Parameters
        if (!ThresholdFilter::getParam(std::string("value_layer"), valueLayerStr_))
        {
            ROS_ERROR("Bayes update did not find parameter value_layer.");
            return false;
        }

        if (!ThresholdFilter::getParam(std::string("output_layer"), outputLayerStr_))
        {
            ROS_ERROR("Bayes update did not find parameter output_layer.");
            return false;
        }

        double tempThreshold;
        if (!ThresholdFilter::getParam(std::string("threshold"), tempThreshold))
        {
            ROS_ERROR("Bayes update did not find parameter threshold.");
            return false;
        }

        double tempOutputValue;
        if (!ThresholdFilter::getParam(std::string("output_value"), tempOutputValue))
        {
            ROS_ERROR("Threshold filter did not find parameter output_value.");
            return false;
        }

        // ROS only allows doubles on parameter server, but gridmap uses floats
        // so we get as double and cast to float
        threshold_ = (float) tempThreshold;
        outputValue_ = (float) tempOutputValue;

        ROS_INFO("Threshold Filter configured with parameters: Value Layer = %s, Output Layer = %s,"
        " Threshold = %f, Output Value = %f", valueLayerStr_.c_str(), outputLayerStr_.c_str(), threshold_, outputValue_);

        return true;
    }

    bool ThresholdFilter::update(const grid_map::GridMap &_mapIn, grid_map::GridMap &_mapOut)
    {
        if (!_mapIn.exists(valueLayerStr_))
        {
            ROS_ERROR("Layer %s does not exist in this map.", valueLayerStr_.c_str());
            return false;
        }
        else if (!_mapIn.exists(outputLayerStr_))
        {
            ROS_ERROR("Layer %s does not exist in this map.", outputLayerStr_.c_str());
            return false;
        }

        _mapOut = _mapIn;
        grid_map::Matrix &valueLayer = _mapOut[valueLayerStr_];
        grid_map::Matrix &outputLayer = _mapOut[outputLayerStr_];

        // Calculation -> Map value layer to matrix of 1's and 0's, then
        // multiply by the outputValue desired.
        outputLayer = valueLayer.unaryExpr([this](const float x) {
            return (float)(x >= threshold_); // 1 if true, 0 if false
        }) * outputValue_;

        return true;
    }
} // namespace mitre_fast_layered_map

PLUGINLIB_EXPORT_CLASS(mitre_fast_layered_map::ThresholdFilter, filters::FilterBase<grid_map::GridMap>);
