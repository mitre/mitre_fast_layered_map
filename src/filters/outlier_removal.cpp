/**
 * @file outlier_removal.cpp
 * @brief Filter implementation for removing isolated outliers
 * @author Trevor Bostic
 */
#include "filters.hpp"
#include <iostream>

#include <grid_map_core/GridMap.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace mitre_fast_layered_map
{

OutlierRemoval::OutlierRemoval()
{
}

OutlierRemoval::~OutlierRemoval()
{
}

bool OutlierRemoval::configure()
{
    // Load Parameters
    if (!OutlierRemoval::getParam(std::string("layer"), layer_))
    {
        ROS_ERROR("Outlier Removal did not find parameter layer.");
        return false;
    }

    ROS_INFO("Outlier Removal using layer: %s", layer_.c_str());

    return true;
}

/**
 * Attempts to remove outliers from the 
 * TODO: Integrate with prior readings to get a more trustworthy solution
 */
bool OutlierRemoval::update(const grid_map::GridMap& _mapIn, grid_map::GridMap& _mapOut)
{
    if (!_mapIn.exists(layer_))
    {
        ROS_ERROR("Layer %s does not exist in this map.", layer_.c_str());
        return false;
    }

    _mapOut = _mapIn;
    grid_map::Matrix& outlierLayer = _mapOut[layer_];
    for(grid_map::GridMapIterator it(_mapOut); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);
        
        // Check if the cell is claimed to be occupied, but if we should consider it an outlier
        if(outlierLayer(index(0), index(1)) == 100 && IsOutlierPoint(_mapOut, index))
        {
            outlierLayer(index(0), index(1)) = 0;
        }

    }

    return true;
}

/**
 * @fn OutlierRemoval::IsOutlierPoint
 * @brief Algorithm to determine if an occupied cell should be considered an outlier
 * 
 * Checks all cells surrounding a supposed occupied cell
 * If any of the surrounding cells are occupied then we believe this one
 * If not, then we will clear it
 * ASSUMPTION: We will use the same layer here that was passed in as a parameter
 * 
 * @param _map [in] The grid map that stores the data
 * @param _index [in] The index within the grid map that we are checking
 * @return True if considered an outlier point, false if not
 */
bool OutlierRemoval::IsOutlierPoint(grid_map::GridMap& _map, const grid_map::Index& _index)
{
    grid_map::Position pos;
    _map.getPosition(_index, pos);

    // Use resolution to check all surrounding cells
    double res = _map.getResolution();

    /* Position + mat.col(i) = position of a surrounding cell
    Example: All surrounding cells are the contents plus position
    
    res = resolution
    pos(x/y) = position x or position y

    [(-res, res)], [(0, res)], [(res, res)],
    [(-res, 0)], [(posx, posy)], [(res, 0)],
    [(-res, -res)], [(0, -res)], [(res, -res)]
    */
    Eigen::MatrixXd mat(2, 8);
    // Values from matrix above in column major order
    mat <<
        -res, -res, -res,   0,    0, res, res, res,
         res,  0  , -res, res, -res, res,   0, -res;

    for(int i = 0; i < mat.cols(); i++)
    {
        const grid_map::Position cellPos = pos + mat.col(i);

        // Boundary check to make sure the position is within the map
        if(_map.isInside(cellPos) && _map.atPosition(layer_, cellPos) == 100)
        {
            return false;
        }
    }

    // If no surrounding cell was occupied, return that this is an outlier.
    return true;

}

} // namespace mitre_fast_layered_map


PLUGINLIB_EXPORT_CLASS(mitre_fast_layered_map::OutlierRemoval, filters::FilterBase<grid_map::GridMap>);
