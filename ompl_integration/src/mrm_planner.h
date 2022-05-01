//
// Created by redwan on 4/30/22.
//

#ifndef MPCHRVO_MRM_PLANNER_H
#define MPCHRVO_MRM_PLANNER_H
#include "helper.h"
#include "polygonal_obstacles.h"
#include "ompl_planner.h"
#include "hrvo_agent.h"
#include <unordered_set>

class MMP{
    /**
     * MMP: multi-robot motion planner
     */
public:
    MMP(ObstclesPtr obs);

    /**
     *
     * @param start start location of robot
     * @param goal destination location of robot
     * @return geometric path based on PRM
     */
    WP generate_path(const vector<float>& start, const vector<float>& goal);

    /**
     * generate robot motions for given geometric paths
     * @param paths PRM geometric paths
     * @param robo_traj collision free trajectories for each robots
     */
    void generate_motion(const vector<WP>& paths, TRAJECTORY *robo_traj);

    /**
     * TRAJECTORY class is defined in helper.h
     * @param A trajectory of robot A : vector<float>x, y
     * @param B trajectory of robot B : vector<float>x, y
     * @return true if collision otherwise false
     */
    bool isCollision(const TRAJECTORY& A, const TRAJECTORY& B);

    WP get_neightbors(float x, float y, unsigned long k);

private:
    ompl_planner prm_;
    shared_ptr<polygonal_obstacles> obstacles_;



private:
    const float radius_ = 0.345f;
    const float sim_time_ = 0.50f;
    const float contrl_time_ = 0.01f;
    const PIDGains gain_{0.9, 1.5, 0.3};
    const HRVOParams vo_params_{20, 10, 0.5, 1.0};


protected:
    /**
     * omple geometric path to hrvo waypoint conversion
     * @param p vector<float> x, y
     * @return 2D waypoints : vector<vector<float>>
     */
    WP waypoint_converter(const ompl_planner::PATH& p);

};




#endif //MPCHRVO_MRM_PLANNER_H
