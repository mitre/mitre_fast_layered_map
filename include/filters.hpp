/**
 * @file filters.hpp
 * @brief Filters that operate on a grid map instance
 * @author Trevor Bostic
 *
 */

#pragma once

#include <filters/filter_base.h>
#include <grid_map_core/grid_map_core.hpp>

namespace mitre_fast_layered_map
{
/**
 * General approach to using ros filter will look like the code below. You
 * can use configuration files to determine which filters are run and in which order.
 * NOTE: FILTER ORDER MATTERS - For example, outlier removal could remove an obstacle, which
 * will then affect how the ray tracing is calculated
 * 
 * @code
 * grid_map::GridMap gridMap;
 * filters::FilterChain<grid_map::GridMap> filterChain_("grid_map::GridMap");
 * filterChain.configure():
 * // Update grid map
 * filterChain.update(gridMap, gridMap);
 * @endcode
 */

    /*
    * This filter finds all obstacles that are in the current map
    * and adds cost buffers around them. Users need to specify the layer to
    * operate on and the inflation size for space around obstacles.
    */
    class Inflation : public filters::FilterBase<grid_map::GridMap>
    {
    public:
        Inflation();

        virtual ~Inflation();

        virtual bool configure();

        virtual bool update(const grid_map::GridMap &_mapIn, grid_map::GridMap &_mapOut);

    private:
        std::string layer_;         ///< Layer of grid map to perform outlier removal on
        double inflationSideLen_m_; ///< Determines length of side of square to inflate around an obstacle
    };

    /**
     * Find outlier cells that are unlikely to be actual obstacles due to their neighborhood
     * having no other obstacles
     */
    class OutlierRemoval : public filters::FilterBase<grid_map::GridMap>
    {
    public:
        OutlierRemoval();

        virtual ~OutlierRemoval();

        virtual bool configure();

        virtual bool update(const grid_map::GridMap &_mapIn, grid_map::GridMap &_mapOut);

        /**
         * @code
         * grid_map::Index index(x, y);
         * if(IsOutlierPoint(gridMap, index))
         * {
         *     // Do stuff
         * }
         * @endcode
         */
        bool IsOutlierPoint(grid_map::GridMap &, const grid_map::Index &);

    private:
        std::string layer_; ///< Layer of grid map to perform outlier removal on
    };

    /**
     * Draws rays from the center of the grid map to each of the outside squares and
     * calculates if an obstacle is blocking the way. From this it can calculate if certain
     * squares are clear, blocked, or unknown.
     */
    class RayTrace2d : public filters::FilterBase<grid_map::GridMap>
    {
    public:
        RayTrace2d();

        virtual ~RayTrace2d();

        virtual bool configure();

        virtual bool update(const grid_map::GridMap &_mapIn, grid_map::GridMap &_mapOut);

        bool trace(grid_map::GridMap &_map, const grid_map::Index &start, const grid_map::Index &end);

    private:
        std::string nongroundLayer_; ///< Layer that holds non ground points. Will be used to check objects
        ///< blocking rays
        std::string groundLayer_; ///< Layer for setting ground plane points
        std::string mode_; ///< Production or testing. If testing mode will throw errors if calculations break bounds.
    };


    class BayesUpdate: public filters::FilterBase<grid_map::GridMap>
    {
        public:
            BayesUpdate();
            
            virtual ~BayesUpdate();
            
            virtual bool configure();

            virtual bool update(const grid_map::GridMap &_mapIn, grid_map::GridMap &_mapOut);

            double dynamicSenseOccGivenOcc(int numHits);
            double dynamicSenseOccGivenEmp(int numHits);

        private:
            std::string probabilityLayerStr_; ///< Layer that holds accumulated probability until now
            std::string historyLayerPrefix_; ///< Prefix for historys layers. Should follow prefix + (int) i standard
            int historyCount_; ///< Num history layers use

            double startingProb_{0.5};
            double probSenseEmpGivenOcc_{0.4};
            double probSenseEmpGivenEmp_{0.8};
            double probSenseOccGivenOccRate_{0.1};
            double probSenseOccGivenOccOffset_{0.3};
            double probSenseOccGivenEmpRate_{-0.1};
            double probSenseOccGivenEmpOffset_{0.3};
    };

    class ThresholdFilter: public filters::FilterBase<grid_map::GridMap>
    {
        public:
            ThresholdFilter();

            virtual ~ThresholdFilter();

            virtual bool configure();

            virtual bool update(const grid_map::GridMap &_mapIn, grid_map::GridMap &_mapOut);

        private:
            std::string valueLayerStr_;
            std::string outputLayerStr_;
            float threshold_;
            float outputValue_;
    };
} // namespace mitre_fast_layered_map