//
// Created by redwan on 5/6/22.
//

#include "hrvo_agent.h"
#include <iostream>
#include <memory>
#include "viz_plan.h"
#include "map_parser.h"
#include <map>

using namespace hrvo;
using namespace std;
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <fmt/format.h>



int Robot::robotID = 0;

class RobotTracker{
public:
    // x, y , theta, mes
    using state = vector<float>;
    RobotTracker(const vector<WP> &paths, double dt) : dt_(dt) {
        auto node_name = ros::this_node::getName();
        auto current_path = ros::package::getPath("traj_view");

        string image_file;
        double low, high;

        string param_low = fmt::format("{}/map/low", node_name);
        string param_high = fmt::format("{}/map/high", node_name);
        string param_img = fmt::format("{}/map/img", node_name);
        string topic_viz = fmt::format("{}/map/topics/viz", node_name);
        string topic_field = fmt::format("{}/map/topics/field", node_name);

        nh_.getParam(param_low, low);
        nh_.getParam(param_high, high);
        nh_.getParam(param_img, image_file);

        nh_.getParam(topic_viz, topic_viz);
        nh_.getParam(topic_field, topic_field);

        image_file = fmt::format("{}/{}", current_path, image_file);

        map_ = make_unique<MapParser>(image_file, low, high);
        viz_ = make_unique<VizPlan>(topic_viz);
        pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(topic_field, 1);

        trakers_.resize(paths.size());
        for (int robotId = 0; robotId < paths.size(); ++robotId) {
            float theta = M_PI_2;
            float z = map_->get_reading(paths[robotId][0][0], paths[robotId][0][1])/255.0;
            trakers_[robotId] = vector<float>{paths[robotId][0][0], paths[robotId][0][1], theta, z};
        }
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
        trakers_[robotId][3] = z;
        ROS_INFO("[Reading] robot = %d coord = (%f, %f) z = %f", robotId, trakers_[robotId][0], trakers_[robotId][1], z);
        viz_->update_robot(robotId, Vector2{trakers_[robotId][0], trakers_[robotId][1]}, trakers_[robotId][2] + M_PI_2);
        publish_message(robotId);
    }
protected:
    void publish_message(int robotID)
    {
        geometry_msgs::Vector3Stamped msg;
        msg.header.frame_id = to_string(robotID);
        msg.header.stamp = ros::Time::now();
        msg.vector.x = trakers_[robotID][0];
        msg.vector.y = trakers_[robotID][1];
        msg.vector.z = trakers_[robotID][3];
        pub_.publish(msg);
    }


private:
    vector<state> trakers_;
    double dt_;
    unique_ptr<VizPlan> viz_;
    unique_ptr<MapParser> map_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};


void copy_path(const vector<float>& x, const vector<float>& y, WP& data)
{
    for (int i = 0; i < x.size(); ++i) {
        data.push_back(std::vector<float>{x[i], y[i]});
    }
}


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "traj_view_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("traj view node started");

    // read parameters
    auto node_name = ros::this_node::getName();
    auto current_path = ros::package::getPath("traj_view");

    float sim_time, contrl_time, radius;

    nh.getParam(fmt::format("{}/hrvo/dt", node_name), sim_time);
    nh.getParam(fmt::format("{}/controller/dt", node_name), contrl_time);
    nh.getParam(fmt::format("{}/hrvo/radius", node_name), radius);

    vector<float> x0, y0, x1, y1;

    nh.getParam(fmt::format("{}/paths/x0", node_name), x0);
    nh.getParam(fmt::format("{}/paths/y0", node_name), y0);
    nh.getParam(fmt::format("{}/paths/x1", node_name), x1);
    nh.getParam(fmt::format("{}/paths/y1", node_name), y1);

    WP path1, path2;
    copy_path(x0, y0, path1);
    copy_path(x1, y1, path2);

//    // simulation setup
    vector<WP> paths{path1, path2};
    PIDGains gain{};
    vector<double> gain_params;
    nh.getParam(fmt::format("{}/controller/gain", node_name), gain_params);
    std::copy_n(gain_params.begin(), gain_params.size(), gain.values.begin());
    HRVOParams param{20,10, 0.25, 0.50};
    MissionSetup mission{paths, radius, sim_time, contrl_time, gain, param};

    RobotTracker traker({path1, path2}, contrl_time);

    MissionCoordinator missionCoordinator;
    missionCoordinator.doSetup(mission, [&](int robotId, const Vector2& cmd_vel){traker.hrvo_callback(robotId, cmd_vel);});
    missionCoordinator.execute();

    return 0;
}