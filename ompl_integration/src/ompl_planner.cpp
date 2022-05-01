//
// Created by redwan on 4/3/22.
//

#include "ompl_planner.h"
#include <memory>
#include <utility>

ompl_planner::ompl_planner() {

    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.
    space_ = make_shared<ob::RealVectorStateSpace>(2);
}


void ompl_planner::set_locs(const vector<float> &in_start, const vector<float> &in_goal) {


    // Set our robot's starting state to be the bottom-left corner of
// the environment, or (0,0).
    ob::ScopedState<> start(space_);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = in_start[0];
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = in_start[1];

// Set our robot's goal state to be the top-right corner of the
// environment, or (1,1).
    ob::ScopedState<> goal(space_);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = in_goal[0];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = in_goal[1];

// Create a problem instance
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);

// Set the start and goal states
    pdef_->setStartAndGoalStates(start, goal);
    pdef_->setOptimizationObjective(getPathLengthObjective(si_));

}
void ompl_planner::setup(ObstclesPtr obstacles) {
 // make sure run it one time

// Set the bounds of space to be in [0,1].
    auto low = obstacles->low;
    auto high = obstacles->high;
    space_->as<ob::RealVectorStateSpace>()->setBounds(low, high);

// Construct a space information instance for this state space
    si_ = std::make_shared<ob::SpaceInformation>(space_);

// Set the object used to check which states in the space are valid
    si_->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si_, std::move(obstacles))));


    si_->setValidStateSamplerAllocator([&](const ob::SpaceInformation *si){return allocValidStateSamplerStrategy(si);});


    si_->setup();

    planner_ = make_shared<og::PRM>(si_);

}

ompl_planner::PATH ompl_planner::get_solution(float solve_time) {

// Set the problem instance for our planner to solve
    planner_->setProblemDefinition(pdef_);
//    optimizingPlanner->setup();


// attempt to solve the planning problem within one second of
// planning time
    ob::PlannerStatus solved = planner_->solve(solve_time);

    ompl_planner::PATH solution;

    // Output the length of the path found
    std::cout
            << planner_->getName()
            << " found a solution of length "
            << pdef_->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef_->getSolutionPath()->cost(pdef_->getOptimizationObjective()) << std::endl;

//    pdef_->getSolutionPath()->print(cout);

    auto res = pdef_->getSolutionPath()->as<og::PathGeometric>();

    for(auto state: res->getStates())
    {
        const ob::RealVectorStateSpace::StateType* state2D =
                state->as<ob::RealVectorStateSpace::StateType>();
        float x = state2D->values[0];
        float y = state2D->values[1];
//        cout << x << ", " << y << endl;
        solution.first.push_back(x);
        solution.second.push_back(y);

    }


    return solution;
}


ob::OptimizationObjectivePtr ompl_planner::getPathLengthObjective(const ompl::base::SpaceInformationPtr &si) {
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

ob::PlannerPtr ompl_planner::get_planner_ptr() {
    return planner_;
}

ob::ValidStateSamplerPtr ompl_planner::allocValidStateSamplerStrategy(const ob::SpaceInformation *si) {
//    return std::make_shared<ob::MaximizeClearanceValidStateSampler>(si);
    return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}


ValidityChecker::ValidityChecker(const ompl::base::SpaceInformationPtr &si, ObstclesPtr obstacles)  :
        ob::StateValidityChecker(si), obstacles_(std::move(obstacles)) {

}


bool ValidityChecker::isValid(const ob::State *state) const {
    const auto* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();
    auto x = (float) state2D->values[0];
    auto y = (float) state2D->values[1];
    return obstacles_->isValidState(x, y);
}

double ValidityChecker::clearance(const ob::State *state) const {
    // We know we're working with a RealVectorStateSpace in this
    // example, so we downcast state into the specific type.
    const ob::RealVectorStateSpace::StateType* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

    // Extract the robot's (x,y) position from its state
    double x = state2D->values[0];
    double y = state2D->values[1];

    // Distance formula between two points, offset by the circle's
    // radius_
    return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;
}

