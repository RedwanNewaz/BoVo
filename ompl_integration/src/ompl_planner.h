//
// Created by redwan on 4/3/22.
//

#ifndef VFRRT_OMPL_PLANNER_H
#define VFRRT_OMPL_PLANNER_H

#include <utility>
#include <vector>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <boost/math/constants/constants.hpp>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include "ompl/geometric/planners/prm/PRM.h"
#include <ompl/base/PlannerData.h>
#include <Eigen/Dense>

#include "polygonal_obstacles.h"
#include <map>


namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

using namespace std;

class prm_mod:public og::PRM{
    using vertex = unsigned long;
public:
    virtual ~prm_mod(){

    }
    vector<float> vertex_to_real_vec(vertex v)
    {
        auto state = si_->cloneState(stateProperty_[v]);
        const ob::RealVectorStateSpace::StateType* state2D =
                state->as<ob::RealVectorStateSpace::StateType>();
        auto x = (float) state2D->values[0];
        auto y = (float) state2D->values[1];


        return {x,y};
    }


    vertex get_milestone(float x, float y)
    {
        vertex best = 0;
        float best_val = std::numeric_limits<float>::max();
        for (int m = 0; m < milestoneCount(); ++m) {
            auto v = vertex_to_real_vec(m);
            auto d = dist(x - v[0], y -v[1]);
            if(d < best_val)
            {
                best_val = d;
                best = m;
            }
        }
        return best;
    }



protected:
    float dist(float x, float y)
    {
        return sqrt(x*x + y*y);
    }

};


class ompl_planner {

    using WP = vector<vector<float>>;

public:
    using PATH = pair<vector<float>, vector<float>>;
    ompl_planner();
    void set_locs(const vector<float> &start, const vector<float> &goal);
    void setup(ObstclesPtr obstacles);
    PATH get_solution(float solve_time);

    ob::PlannerPtr get_planner_ptr();





private:
    ob::ProblemDefinitionPtr pdef_;
    ob::SpaceInformationPtr si_;
    ob::PlannerPtr planner_;
    ob::StateSpacePtr space_;


protected:
    // Returns a structure representing the optimization objective to use
    // for optimal motion planning. This method returns an objective which
    // attempts to minimize the length in configuration space of computed
    // paths.
    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);
    ob::ValidStateSamplerPtr allocValidStateSamplerStrategy(const ob::SpaceInformation *si);
};



// Our collision checker. For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius_ 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si, ObstclesPtr obstacles);

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const override;
    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const override;

private:
    ObstclesPtr obstacles_;
};



#endif //VFRRT_OMPL_PLANNER_H
