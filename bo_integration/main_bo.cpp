#define USE_NLOPT
#include <iostream>
#include <limbo/bayes_opt/boptimizer.hpp>
#include <limbo/opt/nlopt_no_grad.hpp>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>


#include "polygonal_obstacles.h"
#include "ompl_planner.h"

using namespace limbo;
using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct Params {
    struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer {
    };

    struct opt_nloptnograd : public defaults::opt_nloptnograd {
    };
    // enable / disable the writing of the result files
    struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
        BO_PARAM(int, stats_enabled, false);
    };

    // no noise
    struct kernel : public defaults::kernel {
        BO_PARAM(double, noise, 1e-10);
    };

    struct kernel_maternfivehalves : public defaults::kernel_maternfivehalves {
    };

    // we use 10 random samples to initialize the algorithm
    struct init_randomsampling {
        BO_PARAM(int, samples, 10);
    };

    // we stop after 40 iterations
    struct stop_maxiterations {
        BO_PARAM(int, iterations, 40);
    };

    // we use the default parameters for acqui_ucb
    struct acqui_ucb : public defaults::acqui_ucb {
    };

};

struct Eval {
    // number of input dimension (x.size())
    BO_PARAM(size_t, dim_in, 2);
    // number of dimensions of the result (res.size())
    BO_PARAM(size_t, dim_out, 1);

    // the function to be optimized
    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {
        double y = -((5 * x(0) - 2.5) * (5 * x(1) - 2.5)) + 5;
        // we return a 1-dimensional vector
        return tools::make_vector(y);
    }
};


ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si)
{
    // we can perform any additional setup / configuration of a sampler here,
    // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
    return std::make_shared<ob::MaximizeClearanceValidStateSampler>(si);
}

void ompl_state_samples_demo()
{
    auto  space = make_shared<ob::RealVectorStateSpace>(2);
    ob::SpaceInformationPtr si;

    auto obstacles = make_shared<polygonal_obstacles>(0, 25);
    vector<float> x0{10, 15, 15, 10, 10}, x1{ 2, 3, 7, 7, 2, 2}, y0{2, 2, 15, 15, 2}, y1{15, 10, 15, 20, 20, 15};
    obstacles->append(x0, y0);
    obstacles->append(x1, y1);

    // Set the bounds of space to be in [0,1].
    auto low = obstacles->low;
    auto high = obstacles->high;
    space->as<ob::RealVectorStateSpace>()->setBounds(low, high);

// Construct a space information instance for this state space
    si = std::make_shared<ob::SpaceInformation>(space);
    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si, std::move(obstacles))));

    si->setValidStateSamplerAllocator(allocOBValidStateSampler);

    si->setup();

    auto sampler = si->allocValidStateSampler();

    vector<ob::State*>sample_states;
    for (int i = 0; i < 10; ++i) {
        ob::State *state = space->allocState();
        sampler->sample(state);
        sample_states.emplace_back(state);
    }


    vector<vector<double>> real_samples(sample_states.size());
    std::transform(sample_states.begin(), sample_states.end(), real_samples.begin(), [](ob::State *state){
        auto state2D = state->as<ob::RealVectorStateSpace::StateType>();
        auto x = state2D->values[0];
        auto y = state2D->values[1];
        return vector<double>{x, y};
    });

    for(const auto& item:real_samples)
    {
        cout << item[0] << " " << item[1] << endl;
    }

    si->freeStates(sample_states);
}

int main()
{
    // we use the default acquisition function / model / stat / etc.
    bayes_opt::BOptimizer<Params> boptimizer;
    // run the evaluation
    boptimizer.optimize(Eval());
    // the best sample found
    std::cout << "Best sample: " << boptimizer.best_sample()(0) << " - Best observation: " << boptimizer.best_observation()(0) << std::endl;


    return 0;
}