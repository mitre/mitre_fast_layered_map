/**
 * @file bayes_update.cpp
 * @brief Accumulates probability a cell is occupied
 * @author Trevor Bostic
 */
#include "filters.hpp"
#include <iostream>

#include <grid_map_core/GridMap.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <chrono>

namespace mitre_fast_layered_map
{

BayesUpdate::BayesUpdate()
{
}

BayesUpdate::~BayesUpdate()
{
}

bool BayesUpdate::configure()
{
    // Required Parameters
    if (!BayesUpdate::getParam(std::string("probability_layer"), probabilityLayerStr_))
    {
        ROS_ERROR("Bayes update did not find parameter probability layer.");
        return false;
    }

    if (!BayesUpdate::getParam(std::string("history_layer_prefix"), historyLayerPrefix_))
    {
        ROS_ERROR("Bayes update did not find parameter history_layer_prefix.");
        return false;
    }

    if (!BayesUpdate::getParam(std::string("history_count"), historyCount_))
    {
        ROS_ERROR("Bayes update did not find parameter history_count.");
        return false;
    }

    // Advanced features (optional)

    // Below are all sensor model parameters for how likely it is for your sensors are to 
    // register TP, FP, TN, FN's. We allow a "dynamic" setting for TP and FP based on number
    // of points received. 

    if (!BayesUpdate::getParam(std::string("starting_prob"), startingProb_))
    {
        ROS_DEBUG("Starting prob not set. Using default %f", startingProb_);
    }


    if (!BayesUpdate::getParam(std::string("prob_sense_emp_given_occ"), probSenseEmpGivenOcc_))
    {
        ROS_DEBUG("Prob sense emp given occ not set. Using default %f", probSenseEmpGivenOcc_);
    }

    if (!BayesUpdate::getParam(std::string("prob_sense_emp_given_emp"), probSenseEmpGivenEmp_))
    {
        ROS_DEBUG("Prob sense emp given emp not set. Using default %f", probSenseEmpGivenEmp_);
    }

    // We model our sensor as a RELU function, so these give the initial offset and linear rate of growth/decay
    if (!BayesUpdate::getParam(std::string("prob_sense_occ_given_occ_rate"), probSenseOccGivenOccRate_))
    {
        ROS_DEBUG("Prob sense occ given occ rate not set. Using default %f", probSenseOccGivenOccRate_);
    }

    if (!BayesUpdate::getParam(std::string("prob_sense_occ_given_occ_offset"), probSenseOccGivenOccOffset_))
    {
        ROS_DEBUG("Prob sense occ given occ offset not set. Using default %f", probSenseOccGivenOccOffset_);
    }

    if (!BayesUpdate::getParam(std::string("prob_sense_occ_given_emp_rate"), probSenseOccGivenEmpRate_))
    {
        ROS_DEBUG("Prob sense occ given emp rate not set. Using default %f", probSenseOccGivenEmpRate_);
    }

    if (!BayesUpdate::getParam(std::string("prob_sense_occ_given_emp_offset"), probSenseOccGivenEmpOffset_))
    {
        ROS_DEBUG("Prob sense occ given emp offset not set. Using default %f", probSenseOccGivenEmpOffset_);
    }

    ROS_INFO("Bayes filter configured with parameters: Probability Layer = %s", probabilityLayerStr_.c_str());

    return true;
}

/**
 * Updates the probability that a cell is occupied
 * Algorithm found in https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=30720
 * @TODO: This function can probably be vectorized for performance gains
 */
bool BayesUpdate::update(const grid_map::GridMap& _mapIn, grid_map::GridMap& _mapOut)
{
    if (!_mapIn.exists(probabilityLayerStr_))
    {
        ROS_ERROR("Layer %s does not exist in this map.", probabilityLayerStr_.c_str());
        return false;
    }

    _mapOut = _mapIn;
    grid_map::Matrix& probabilityLayer = _mapOut[probabilityLayerStr_];

    // Add history layers and make sure they exist
    std::vector<std::shared_ptr<grid_map::Matrix>> historyLayers;
    for(int i = 0; i < historyCount_; i++)
    {
        std::string layer = historyLayerPrefix_ + std::to_string(i);
        if (!_mapIn.exists(historyLayerPrefix_ + std::to_string(i)))
        {
            ROS_ERROR("Layer %s expected but not found in map", (historyLayerPrefix_ + std::to_string(i)).c_str());
            return false;
        }

        // If it exists add to vector of history layers
        historyLayers.push_back(std::make_shared<grid_map::Matrix>(_mapOut[layer]));
    }

    // Temp variables for ease
    double probGivenOccToUse, probGivenEmpToUse, beliefOcc, beliefEmp, normalizer;
    int numFramesInScope;
    /*
     * Below we run a bayes filter to determine our confidence in an obstacle
     * being present in a given cell based on the last 10 readings. We specify
     * the most recent readings so that the map can cells can update quickly.
     */
    for(grid_map::GridMapIterator it(_mapOut); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);

        double currentProb = startingProb_;

        // Construct history of probability readings
        std::vector<int> hitsHistory;

        // We may evaluate in a different order from which things were recorded,
        // but order shouldn't matter
        for (int i = 0; i < historyLayers.size(); i++)
        {
            grid_map::Matrix& history = *(historyLayers[i].get());
            // Negative values represent no readings ever placed in layer
            // Likely when cell has just come into scope
            if (history(index(0), index(1)) >= 0)
            {
                hitsHistory.push_back(history(index(0), index(1)));
            }
        }
        
        // Update the probability for each reading currently stored in history
        for (int i = 0; i < hitsHistory.size(); i++)
        {
            // Sense nothing in cell this iteration
            if (hitsHistory[i] == 0)
            {
                probGivenOccToUse = probSenseEmpGivenOcc_;
                probGivenEmpToUse = probSenseEmpGivenEmp_;
            }
            else // Received positive readings
            {
                probGivenOccToUse = dynamicSenseOccGivenOcc(hitsHistory[i]);
                probGivenEmpToUse = dynamicSenseOccGivenEmp(hitsHistory[i]);
            }

            // Apply bayes to determine probability of readings 
            beliefOcc = probGivenOccToUse * currentProb;
            beliefEmp = probGivenEmpToUse * (1 - currentProb);
            normalizer = 1 / (beliefOcc + beliefEmp);

            currentProb = beliefOcc * normalizer;
        }

        probabilityLayer(index(0), index(1)) = currentProb;

    }

    return true;
}


/**
 * @TODO: Make this customizable to allow for other sensor architectures
 * This and the function below can be thought of a bit like
 * a modified RELU function. Using this function we can dynamically
 * modify our confidence in our sensor readings based on the number of points 
 * that were sensed in a given cell.
 * y = 0.05 * readings + 0.5 - line formula
 * 0.99 is max confidence of obstacle we will allow
 */ 
double BayesUpdate::dynamicSenseOccGivenOcc(int numHits)
{
    return std::min(0.99, (numHits * probSenseOccGivenOccRate_) + probSenseOccGivenOccOffset_);
}

/**
 * Opposite of previous function. We lose confidence
 * that a cell is empty occupied as the number of readings increases
 * y = -0.1 * readings + 0.3
 */
double BayesUpdate::dynamicSenseOccGivenEmp(int numHits)
{
    return std::max(0.01, (numHits * probSenseOccGivenEmpRate_) + probSenseOccGivenEmpOffset_);
}

} // namespace mitre_fast_layered_map


PLUGINLIB_EXPORT_CLASS(mitre_fast_layered_map::BayesUpdate, filters::FilterBase<grid_map::GridMap>);
