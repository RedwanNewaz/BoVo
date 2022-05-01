//
// Created by redwan on 4/30/22.
//

#include "mrm_planner.h"



MMP::MMP(ObstclesPtr obs): obstacles_(obs)
{
    prm_.setup(obstacles_);
}

WP MMP::generate_path(const vector<float>& start, const vector<float>& goal)
{
    prm_.set_locs(start, goal);
    auto sol1 = prm_.get_solution(1.0);
    return waypoint_converter(sol1);
}

void MMP::generate_motion(const vector<WP>& paths, TRAJECTORY *robo_traj)
{
    assert(paths.size() > 1 && "at least 2 robots are required!!");

    vector<PIDGains> gains{gain_, gain_};
    MissionSetup mission{paths, radius_, sim_time_, contrl_time_, gains, vo_params_};


    auto hrvo_callback= [&](int robotID, const Vector2& cmd_vel)
    {
        robo_traj[robotID].x.push_back(cmd_vel.getX());
        robo_traj[robotID].y.push_back(cmd_vel.getY());
        //    cout << "[callback] robotID " << robotID << " msg " << cmd_vel << endl;
    };

    MissionCoordinator missionCoordinator;
    missionCoordinator.doSetup(mission, hrvo_callback);
    missionCoordinator.execute();
}

bool MMP::isCollision(const TRAJECTORY& A, const TRAJECTORY& B)
{
    int right= min(A.x.size(), B.x.size()) - 1;
    int left = 0;
    auto dist = [](float x1, float x2,  float y1, float y2){
        return sqrt(pow(x1-x2,2) + pow(y1-y2, 2));
    };

    while (left <= right)
    {
        auto d_left = dist(A.x[left], B.x[left], A.y[left], B.y[left]);
        auto d_right = dist(A.x[right], B.x[right], A.y[right], B.y[right]);

        bool invalid = (!obstacles_->isValidState(A.x[left], A.y[left]) ||
                        !obstacles_->isValidState(B.x[left], B.y[left]) ||
                        !obstacles_->isValidState(A.x[right], A.y[right]) ||
                        !obstacles_->isValidState(B.x[right], B.y[right]));

        if(d_left < radius_ || d_right < radius_ || invalid)
        {
            cout << d_left <<" " <<  d_right << " invalid " << invalid << " => ";
            break;
        }


        ++left;
        --right;
    }
    return (right - left) > 1;
}

WP MMP::waypoint_converter(const ompl_planner::PATH& p)
{
    WP res;
    for (int i = 0; i < p.first.size(); ++i) {
        float x = p.first[i];
        float y = p.second[i];
        res.push_back({x,y});
    }
    return res;
}

WP MMP::get_neightbors(float x, float y, unsigned long k) {
    auto planner = prm_.get_planner_ptr();
    auto prm = planner->as<prm_mod>();
    auto nn = prm->getNearestNeighbors();

    vector<unsigned long> nbh;
    auto milestone = prm->get_milestone(x, y);
    nn->nearestK(milestone, k,nbh);
    WP res;

    for(const auto& v: nbh)
    {
        auto stateX = prm->vertex_to_real_vec(v);
        res.emplace_back(stateX);
    }
    return res;
}




