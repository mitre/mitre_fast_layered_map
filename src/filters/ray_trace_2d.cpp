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

namespace t_mapping
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

    To calculate the boundary cells we first notice that we can get the position of the 4 corners
    of the grid with the equation (x, y) = (+- grid_len / 2, +- grid_len / 2)

    We can then calculate the position of each cell in a row/col by adding/subtracting 
    multiples of the resolution to the corner points.   
    */

    grid_map::Position vehPosition = _mapOut.getPosition();
    double resolution = _mapOut.getResolution();
    double length = _mapOut.getLength().x(); // ASSUMPTION: Square grid where length = width

    grid_map::Index vehIndex;
    _mapOut.getIndex(vehPosition, vehIndex);  // Lines always start at the vehicle

    double halfLen = length / 2;
    double halfRes = resolution / 2;
    
    grid_map::Position pos;
    grid_map::Index posIndex;
    Eigen::Vector2d addVector;

    /*
    VISUALIZATION NOTE:
    Due to starting at -halfLen and going to halfLen, you can imagine this as starting
    at the top left and bottom left corners and working to the right for rows, and starting
    at the bottom left and bottom right and working our way up the columns of the grid.
    */
    for (double cellDist = -halfLen; cellDist < halfLen; cellDist += resolution)
    {
        // We add resolution / 2 in many cases to aim for the center of the cell

        Eigen::MatrixXd mat(2, 4);
        // [right col, left col, top row, bottom row] where each element is a 2d vertical vector
        mat <<
             halfLen - halfRes, -halfLen + halfRes, cellDist + halfRes, cellDist + halfRes,
            cellDist + halfRes, cellDist + halfRes,  halfLen - halfRes, -halfLen + halfRes;

        for (int i = 0; i < mat.cols(); i++)
        {
            pos = vehPosition + mat.col(i);

            if(_mapOut.getIndex(pos, posIndex))
            {
                trace(_mapOut, vehIndex, posIndex);
            }
            else
            {
                ROS_WARN("Calculation outside of map: x - %f, y - %f", pos[0], pos[1]);
            }
        }
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


} // namespace t_mapping

PLUGINLIB_EXPORT_CLASS(t_mapping::RayTrace2d, filters::FilterBase<grid_map::GridMap>);