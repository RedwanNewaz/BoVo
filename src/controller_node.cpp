//
// Created by redwan on 5/8/22.
//
#include <ros/ros.h>
#include "controller/controller.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <thread>

class StateTransition: public controller{
public:
    StateTransition(const vector<double> &state, double kpRho, double kpAlpha, double kpBeta, double dt,
                    double goalThres, const std::string& poseTopic, const std::string& robotID) :
                    controller(state, kpRho, kpAlpha, kpBeta, dt, goalThres), robotID_(robotID) {
        timer_ = nh_.createTimer(ros::Duration(dt), &StateTransition::control_loop, this);
        pose_pub_ = nh_.advertise<nav_msgs::Odometry>(poseTopic, 10);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        goal_sub_ = nh_.subscribe("/stf/data", 10, &StateTransition::goal_callback, this);
        nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("cmd_vel_ekf", 10);
        timer_.start();
        initialized_ = false;
    }
    void control_loop(const ros::TimerEvent& event)
    {
        if(!initialized_)
        {
            return;
        }
        auto cmd_vel = compute_control();
        double v = cmd_vel.first;
        double w = cmd_vel.second;
        double theta = state_[2] + w * dt_;
        state_[0] += v * cos(theta) * dt_;
        state_[1] += v * sin(theta) * dt_;
        state_[2] = theta;
//        publish_cmd_message(v, w);
        publish_pose_message();
    }

    void goal_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        if(msg->header.frame_id == robotID_)
        {
            initialized_ = true;
            ROS_INFO_STREAM(*msg);
            set_points(msg->vector.x, msg->vector.y);
        }

    }

protected:

    void publish_pose_message()
    {
        nav_msgs::Odometry msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();

        msg.pose.pose.position.x = state_[0];
        msg.pose.pose.position.y = state_[1];

        tf::Quaternion q;
        q.setRPY(0, 0, state_[2]);
        msg.pose.pose.orientation.x = q.getX();
        msg.pose.pose.orientation.y = q.getY();
        msg.pose.pose.orientation.z = q.getZ();
        msg.pose.pose.orientation.w = q.getW();
        pose_pub_.publish(msg);
    }

    void publish_cmd_message(double v, double w)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = v;
        msg.angular.z = w;
        cmd_pub_.publish(msg);

        geometry_msgs::TwistWithCovarianceStamped twist;
        twist.header.stamp = ros::Time::now();
        twist.header.frame_id = "map";

        twist.twist.twist.linear.x = v;
        twist.twist.twist.angular.z = w;
        cmd_pub_ekf_.publish(twist);
    }
private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Subscriber goal_sub_;
    ros::Publisher pose_pub_, cmd_pub_, cmd_pub_ekf_;
    string robotID_;
    bool initialized_;
};


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "bovoController");
    ros::NodeHandle nh("~");
    ROS_INFO("BOVO controller node initialized");

    vector<double> state, gain;
    nh.getParam("state", state);
    nh.getParam("pidGain", gain);

    string poseTopic, robotID;
    nh.getParam("poseTopic", poseTopic);
    nh.getParam("robotID", robotID);

    double dt, goalThres;
    nh.getParam("dt", dt);
    nh.getParam("thres", goalThres);
    ROS_INFO_STREAM(state[0] << " " << state[1]);
    StateTransition pid(state, gain[0], gain[1], gain[2], dt, goalThres, poseTopic, robotID);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}