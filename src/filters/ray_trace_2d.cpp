/**
 * @file ray_trace_2d.cpp
 * @brief Filter implementation of ray tracing algorithm in 2d space
 * @author Trevor Bostic
 */
#include <filters.hpp>

// Gridmap
#include <grid_map_core/grid_map_core.hpp>

// Ros
#include <pluginlib/class_list_macros.hpp>

namespace mitre_fast_layered_map
{

RayTrace2d::RayTrace2d()
{
}

RayTrace2d::~RayTrace2d()
{
}

bool RayTrace2d::configure()
{
    if(!RayTrace2d::getParam(std::string("nonground_layer"), nongroundLayer_))
    {
        ROS_ERROR("Unable to find obstacle layer parameter.");
        return false;
    }

    if(!RayTrace2d::getParam(std::string("ground_layer"), groundLayer_))
    {
        ROS_ERROR("Unable to find ground layer parameter.");
        return false;
    }

    if(!RayTrace2d::getParam(std::string("mode"), mode_))
    {
        mode_ = "prod"; // Default is production mode
    }

    ROS_INFO("Configured RayTrace2d with nonground layer: %s, and ground layer: %s", nongroundLayer_.c_str(), groundLayer_.c_str());

    return true;
}

bool RayTrace2d::update(const grid_map::GridMap& _mapIn, grid_map::GridMap& _mapOut)
{
    if(!_mapIn.exists(nongroundLayer_))
    {
        ROS_ERROR("Layer %s does not exist within the map.", nongroundLayer_.c_str());
        return false;
    }
    else if(!_mapIn.exists(groundLayer_))
    {
        ROS_ERROR("Layer %s does not exist within the map.", groundLayer_.c_str());
        return false;
    }

    _mapOut = _mapIn;

    /*
    The trace function below takes a start and an end point for its ray calculations.
    To ensure that we check every cell with rays emitted from the vehicles position, 
    we draw the lines from the vehicles position to each of the outermost cells.

    Example: For the grid below with the vehicle positioned at x, we want to draw
    lines from x to i so that we can interpolate which cells are clear or unknown
    due to a barrier blocking the way. 

    [i], [i], [i], [i], [i],
    [i], [],  [],  [],  [i],
    [i], [],  [x], [],  [i],
    [i], [],  [],  [],  [i],
    [i], [i], [i], [i], [i]

    GridMap does not give any nice way (that we know of) for calculating the 
    boundary cells of the map. We cannot use constant indices due to their implementation
    of the grid as a circular buffer. The calculations below get each of the cells
    and then runs the tracing algorithm.

    ASSUMPTION: The grid is square

    To get the outermost rows and columns, we start with the gridMap.getStartIndex() function
    which gives us the index of the top left cell in the grid. this immediately gives us the top
    row and the left column.

    From there, we can calculate the bottom row and right column by adding the number of rows - 1
    (to offset for the current row) modulus the number of rows to account for the wrapping. If this seems 
    strange, try drawing a 5x5 grid and manually doing the calculations.
    */

    grid_map::Position vehPosition = _mapOut.getPosition();

    grid_map::Index vehIndex;
    _mapOut.getIndex(vehPosition, vehIndex);  // Lines always start at the vehicle

    int numRows = _mapOut.getSize()(0);

    grid_map::Index startIdx = _mapOut.getStartIndex(); // Index of cell at top left corner
    int topRow = startIdx(0);
    int bottomRow = (startIdx(0) + (numRows - 1)) % numRows;
    int leftCol = startIdx(1);
    int rightCol = (startIdx(1) + (numRows - 1)) % numRows;

    for(int i = 0; i < numRows; i++)
    {
        trace(_mapOut, vehIndex, grid_map::Index(topRow, i)); // Ray traces going towards top row
        trace(_mapOut, vehIndex, grid_map::Index(bottomRow, i)); // Ray traces going towards bottom row
        trace(_mapOut, vehIndex, grid_map::Index(i, leftCol)); // Ray traces going towards left column
        trace(_mapOut, vehIndex, grid_map::Index(i, rightCol)); // Ray traces going towards right column
    }

    return true;

}

bool RayTrace2d::trace(grid_map::GridMap& _map, const grid_map::Index& start, const grid_map::Index& end)
{
    grid_map::Matrix& ground = _map[groundLayer_];
    grid_map::Matrix& nonground = _map[nongroundLayer_];
    bool obstacleFound = false;

    for (grid_map::LineIterator lit(_map, start, end); !lit.isPastEnd(); ++lit)
    {
        const grid_map::Index lineIndex(*lit);

        // If we find an obstacle, assume it blocks sight along the line past here
        if (nonground(lineIndex(0), lineIndex(1)) == 100)
        {
            obstacleFound = true;
        }
        else if (obstacleFound && ground(lineIndex(0), lineIndex(1)) > 0)
        {
            // We had found an obstacle, but are also registering ground points beyond it
            // We might be seeing over, around, or sometimes through the obstacle
            obstacleFound = false;
            nonground(lineIndex(0), lineIndex(1)) = 0;
            // ground(lineIndex(0), lineIndex(1)) = 0;
        }
        else if (obstacleFound) // If obstacle found but no ground points registered, mark as unknown
        {
            // ground(lineIndex(0), lineIndex(1)) = 20;
            nonground(lineIndex(0), lineIndex(1)) = 20;
        }
        else // If no obstacle found yet, mark as clear
        {
            nonground(lineIndex(0), lineIndex(1)) = 0;
            // ground(lineIndex(0), lineIndex(1)) = 0;
        }
    }

    return true;
}


} // namespace mitre_fast_layered_map

PLUGINLIB_EXPORT_CLASS(mitre_fast_layered_map::RayTrace2d, filters::FilterBase<grid_map::GridMap>);