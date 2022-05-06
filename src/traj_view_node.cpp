//
// Created by redwan on 5/6/22.
//

#include "hrvo_agent.h"
#include <iostream>
#include <memory>
#include "viz_plan.h"
#include "map_parser.h"


using namespace hrvo;
using namespace std;
#include <ros/ros.h>


int Robot::robotID = 0;

class RobotTracker{
public:
    using state = vector<float>;
    RobotTracker(const vector<WP> &paths, double dt) : dt_(dt) {
        trakers_.resize(paths.size());
        for (int i = 0; i < paths.size(); ++i) {
            float theta = M_PI_2;
            trakers_[i] = vector<float>{paths[i][0][0], paths[i][0][1], theta};

        }

        viz_ = make_unique<VizPlan>("/roomba20/path");
        map_ = make_unique<MapParser>("/home/redwan/catkin_ws/src/traj_view/config/map/chlorophyll.png", 0, 2);
    }

    void hrvo_callback(int robotId, const Vector2& cmd_vel)
    {
        auto v = cmd_vel.getX();
        auto w = cmd_vel.getY();
        float theta = trakers_[robotId][2] + w * dt_;
        trakers_[robotId][0] += v * cos(theta) * dt_;
        trakers_[robotId][1] += v * sin(theta) * dt_;
        trakers_[robotId][2] = theta;

        float z = map_->get_reading(trakers_[robotId][0], trakers_[robotId][1])/255.0;
        ROS_INFO("[Reading] robot = %d coord = (%f, %f) z = %f", robotId, trakers_[robotId][0], trakers_[robotId][1], z);
        viz_->update_robot(robotId, Vector2{trakers_[robotId][0], trakers_[robotId][1]}, trakers_[robotId][2] + M_PI_2);
    }


private:
    vector<state> trakers_;
    double dt_;
    unique_ptr<VizPlan> viz_;
    unique_ptr<MapParser> map_;
};


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "traj_view_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("traj view node started");
    // simulation setup
    WP path1{{1, 0}, {1.5, 0}, {1.5, 1}, {0, 1}, {0, 2}};
    WP path2{{0, 2}, {1.5, 2}, {1.5, 1}, {0, 1}, {1, 0}};
    vector<WP> paths{path1, path2};
    float radius = 0.345/2.0;
    float sim_time = 0.10f;
    float contrl_time = 0.01f;
    PIDGains gain{0.9, 1.5, 0.3};
    HRVOParams param{20,10, 0.25, 0.50};
    vector<PIDGains> gains{gain, gain};
    MissionSetup mission{paths, radius, sim_time, contrl_time, gains, param};

    RobotTracker traker({path1, path2}, contrl_time);

    MissionCoordinator missionCoordinator;
    missionCoordinator.doSetup(mission, [&](int robotId, const Vector2& cmd_vel){traker.hrvo_callback(robotId, cmd_vel);});
    missionCoordinator.execute();



    return 0;
}