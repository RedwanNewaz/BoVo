//
// Created by redwan on 5/7/22.
//
#include <ros/ros.h>
#include <HRVO.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <condition_variable>
#include "bovo/map_parser.h"
#include "bovo/viz_plan.h"

using namespace std;
using namespace hrvo;


std::condition_variable condVar;
std::mutex mu;

class Robot{
public:
    Robot(const string& goalTopic, const string& stateTopic, const shared_ptr<Simulator> &sim, const Vector2& curr_pos, int id) : sim_(sim),
    curr_goal_(curr_pos), curr_pos_(curr_pos), robotID_(id) {
        goal_sub_ = nh_.subscribe(goalTopic, 10, &Robot::goal_callback, this);
        state_sub_ = nh_.subscribe(stateTopic, 10, &Robot::state_callback, this);
        state_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/stf/data", 1);

        sim_->addAgent(curr_pos_, sim_->addGoal(curr_goal_));
        string topic_viz = "/roomba20/path";
        string topic_field = "/stf/data";
        string image_file = "/home/redwan/catkin_ws/src/traj_view/config/map/chlorophyll.png";
        double low(0), high(2);
        viz_ = make_unique<VizPlan>(topic_viz);
        map_ = make_unique<MapParser>(image_file, low, high);

        initialize_ = false;
        angle_ = 0;
    }

    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        ROS_INFO_STREAM(*msg);
        curr_goal_.setX(msg->pose.position.x);
        curr_goal_.setY(msg->pose.position.y);
        sim_->setAgentGoal(robotID_, sim_->addGoal(curr_goal_));
        condVar.notify_all();
    }

    void state_callback(const nav_msgs::Odometry::ConstPtr& msg){
        Vector2 msg_pos;
        msg_pos.setX(msg->pose.pose.position.x);
        msg_pos.setY(msg->pose.pose.position.y);
        sim_->setAgentPosition(robotID_, msg_pos);
        tf::Quaternion q;
        q.setX(msg->pose.pose.orientation.x);
        q.setY(msg->pose.pose.orientation.y);
        q.setZ(msg->pose.pose.orientation.z);
        q.setW(msg->pose.pose.orientation.w);
        initialize_ = true;
        angle_ = q.getAngle() + M_PI_2;
        viz_->update_robot(robotID_, msg_pos, angle_);
    }


    float progress()
    {
        curr_pos_ = sim_->getAgentPosition(robotID_);
        auto diff = curr_goal_-curr_pos_;
        float theta = (initialize_)?angle_:atan2(diff.getY(), diff.getX());
//        viz_->update_robot(robotID_, curr_pos_, theta);
//        if(sim_->getAgentRadius(robotID_) > abs(diff))
            publish_state_message();
        return abs(diff);
    }
protected:
    void publish_state_message()
    {
        geometry_msgs::Vector3Stamped msg;
        msg.header.frame_id = to_string(robotID_);
        msg.header.stamp = ros::Time::now();
        msg.vector.x = curr_pos_.getX();
        msg.vector.y = curr_pos_.getY();
        msg.vector.z = map_->get_reading(curr_pos_.getX(), curr_pos_.getY())/255.0;
        state_pub_.publish(msg);
    }
private:
    shared_ptr<Simulator> sim_;
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    ros::Subscriber goal_sub_, state_sub_;
    ros::Timer timer_;
    Vector2 curr_pos_, curr_goal_;
    unique_ptr<VizPlan> viz_;
    unique_ptr<MapParser> map_;
    double angle_;
    int robotID_;
    bool initialize_;
};

typedef shared_ptr<Robot> RobotPtr;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "bovo");
    ROS_INFO("BOVO node initialized");

    auto hrvo_sim = make_shared<Simulator>();
    float radius = 0.345;
    float prefSpeed = 0.25;
    float maxSpeed = 0.50;
    float sim_time = 0.25f;
    hrvo_sim->setAgentDefaults(20, 10, radius, radius/2.0, prefSpeed, maxSpeed);
    hrvo_sim->setTimeStep(sim_time);


    auto r1 = make_shared<Robot>("roomba20/goal", "roomba20/odometry/filtered",  hrvo_sim, Vector2{0, 2}, 0);
    auto r2 = make_shared<Robot>("roomba21/goal","roomba21/odometry/filtered", hrvo_sim, Vector2{1, 0}, 1);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Rate simTime(1.0/sim_time);
    radius /= 16.0;
    while(ros::ok() && !ros::isShuttingDown())
    {
        unique_lock<mutex> lk(mu);
        condVar.wait(lk,[&]{
         return r1->progress() > radius || r2->progress() > radius;
        });
        hrvo_sim->doStep();
        ROS_INFO("[HRVO]  r1 prog =  %f | r2 prog = %f", r1->progress(), r2->progress());
        simTime.sleep();
        // manually update



        condVar.notify_all();
    }

    return 0;
}