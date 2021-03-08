/**
 * @file inflation.cpp
 * @brief Filter implementation for inflating area surrounding obstacles
 * @author Trevor Bostic
 */
#include "filters.hpp"

#include <ros/ros.h>

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace mitre_fast_layered_map
{
Inflation::Inflation()
{
}

Inflation::~Inflation()
{
}

bool Inflation::configure()
{
    if (!Inflation::getParam(std::string("layer"), layer_))
    {
        ROS_ERROR("Unable to find nonground_layer parameter.");
        return false;
    }

    if (!Inflation::getParam(std::string("inflation_side_len_m"), inflationSideLen_m_))
    {
        ROS_ERROR("Unable to find inflation radius parameter.");
        return false;
    }

    ROS_INFO("Running inflation filter with layer: %s, and inflation radius: %f", layer_.c_str(), inflationSideLen_m_);

    return true;
}

bool Inflation::update(const grid_map::GridMap &_mapIn, grid_map::GridMap &_mapOut)
{
    if (!_mapIn.exists(layer_))
    {
        ROS_ERROR("Layer %s does not exist within the map.", layer_.c_str());
        return false;
    }

    _mapOut = _mapIn;

    grid_map::Matrix &inflateLayer = _mapOut[layer_];
    // We will set inflation on the inflate layer then add it into the layer above at the end
    grid_map::Matrix layer = _mapOut[layer_];

    int sideLen = _mapOut.getLength().x(); // We assume a square map
    int cellSideLen = _mapOut.getSize()(0);
    double resolution = _mapOut.getResolution();

    // Rim is the outermost ring of cells
    double distToRim = sideLen / 2;

    grid_map::Position pos = _mapOut.getPosition();

    double posXMax = pos.x() + distToRim;
    double posXMin = pos.x() - distToRim;
    double posYMax = pos.y() + distToRim;
    double posYMin = pos.y() - distToRim;

    // Cells to inflate to unless hitting edge of map
    int numCellsForInflation = (int)floor(inflationSideLen_m_ / resolution);

    for (grid_map::GridMapIterator mapIt(_mapOut); !mapIt.isPastEnd(); ++mapIt)
    {
        const grid_map::Index index(*mapIt);
        
        // Use the original layer to find obstacles, they may have been wiped out on inflate layer
        if (layer(index(0), index(1)) != 100)
        {
            continue;
        }

        // We want to select a block of matrix cells for inflation
        // to make it as fast as possible. We will calculate +x and +y to
        // determine where the block will start, and then use -x and -y
        // to calculate the size of the block

        grid_map::Position cellPos;
        _mapOut.getPosition(index, cellPos);

        // Calculate cells from edge in +x direction
        int xCells = (int)floor((posXMax - cellPos.x()) / resolution);
        int yCells = (int)floor((posYMax - cellPos.y()) / resolution);

        //
        // CALCULATE INFLATION BOX START CELL
        //

        int rowStartIdx;
        int colStartIdx;

        // If there are more cells than needed for inflation size
        if (xCells > numCellsForInflation)
        {
            // Use inflation size, +x so subtract from current idx
            rowStartIdx = index.x() - numCellsForInflation;
        }
        else
        {
            // Otherwise, use the max number of cells before hitting boundary
            rowStartIdx = index.x() - xCells;
        }

        // If there are more cells than needed for inflation size
        if (yCells > numCellsForInflation)
        {
            // Use inflation size, +y so subtract from current idx
            colStartIdx = index.y() - numCellsForInflation;
        }
        else
        {
            // Otherwise, use the max number of cells before hitting boundary
            colStartIdx = index.y() - yCells;
        }

        //
        // CALCULATE HEIGHT AND WIDTH OF BOX
        //

        int blockXSize;
        int blockYSize;

        // Do the same for minimum
        xCells = (int)floor((cellPos.x() - posXMin) / resolution);
        yCells = (int)floor((cellPos.y() - posYMin) / resolution);

        if (xCells > numCellsForInflation)
        {
            // Calculate size of block in x direction
            // index.x() - rowStartIdx gives num cells going up, num cells for inflation
            // adds on for the number of cells going down, and +1 is to include the
            // starting cell
            blockXSize = (index.x() - rowStartIdx) + numCellsForInflation + 1;
        }
        else
        {
            blockXSize = (index.x() - rowStartIdx) + xCells + 1;
        }

        if (yCells > numCellsForInflation)
        {
            blockYSize = (index.y() - colStartIdx) + numCellsForInflation + 1;
        }
        else
        {
            blockYSize = (index.y() - colStartIdx) + yCells + 1;
        }

        // Two cases of wrapping.
        // 1. Either start index of x or y is negative, which means that
        // the obstacle is near enough to 0 in the matrix that inflation in the map needs
        // to be placed in the higher rows of the matrix.
        // 2. The start index + inflation wraps back over to 0.
        // These are both essentially the same problem, so we should map one to the other.

        if (rowStartIdx < 0)
        {
            rowStartIdx = cellSideLen + rowStartIdx;
        }

        if (colStartIdx < 0)
        {
            colStartIdx = cellSideLen + colStartIdx;
        }

        /*
         * Below is a potential worst case scenario, where we need to break up into 4 matrices.
         * The numbers within the cells of the bottom matrix correspond to where the case gets taken
         * care of.
         * 
         * [2,3], [2,4], [2,0], [2,1], [2,2],
         * [3,3], [3,4], [3,0], [3,1], [3,2],
         * [4,3], [4,4], [4,0], [4,1], [4,2],
         * [0,3], [0,4], [0,0], [0,1], [0,2],
         * [1,3], [1,4], [1,0], [1,1], [1,2],
         * 
         * [2], [2], [1], [1], [1],
         * [2], [2], [1], [1], [1],
         * [3], [3], [4], [4], [4],
         * [3], [3], [4], [4], [4],
         * [3], [3], [4], [4], [4],
         */

        // If our block matrix is wrapping on both rows and columns
        if (rowStartIdx + blockXSize > cellSideLen && colStartIdx + blockYSize > cellSideLen)
        {
            int rowsBeforeWrap = cellSideLen - rowStartIdx;
            int rowsAfterWrap = blockXSize - rowsBeforeWrap;

            int colsBeforeWrap = cellSideLen - colStartIdx;
            int colsAfterWrap = blockYSize - colsBeforeWrap;

            // Case 1: Need to wrap the rows back around
            inflateLayer.block(rowStartIdx, 0, rowsBeforeWrap, colsAfterWrap).array() = 30;

            // Case 2: Get wrapped around columns
            inflateLayer.block(rowStartIdx, colStartIdx, rowsBeforeWrap, colsBeforeWrap).array() = 30;

            // Case 3: Get positive wrapped around rows
            inflateLayer.block(0, colStartIdx, rowsAfterWrap, colsBeforeWrap).array() = 30;

            // Case 4: Get regular row matrix
            inflateLayer.block(0, 0, rowsAfterWrap, colsAfterWrap).array() = 30;
        }
        else if (rowStartIdx + blockXSize > cellSideLen)
        {
            int rowsBeforeWrap = cellSideLen - rowStartIdx;
            int rowsAfterWrap = blockXSize - rowsBeforeWrap;

            // The top wrapped around matrix
            inflateLayer.block(rowStartIdx, colStartIdx, rowsBeforeWrap, blockYSize).array() = 30;

            // Unwrapped bit
            inflateLayer.block(0, colStartIdx, rowsAfterWrap, blockYSize).array() = 30;
        }
        else if (colStartIdx + blockYSize > cellSideLen)
        {
            int colsBeforeWrap = cellSideLen - colStartIdx;
            int colsAfterWrap = blockYSize - colsBeforeWrap;

            // Wrapped around columns
            inflateLayer.block(rowStartIdx, colStartIdx, blockXSize, colsBeforeWrap).array() = 30;

            // Unwrapped bit
            inflateLayer.block(rowStartIdx, 0, blockXSize, colsAfterWrap).array() = 30;
        }
        else
        {
            // No wrapping required
            inflateLayer.block(rowStartIdx, colStartIdx, blockXSize, blockYSize).array() = 30;
        }
    }

    // Replace any overwritten obstacles
    for (grid_map::GridMapIterator it(_mapOut); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        if (layer(index(0), index(1)) == 100)
        {
            inflateLayer(index(0), index(1)) = 100;
        }
    }

    return true;
}


} // namespace mitre_fast_layered_map

PLUGINLIB_EXPORT_CLASS(mitre_fast_layered_map::Inflation, filters::FilterBase<grid_map::GridMap>);