//
// Created by redwan on 5/6/22.
//
#ifndef TRAJ_VIEW_VIZ_PLAN_H
#define TRAJ_VIEW_VIZ_PLAN_H
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <fmt/format.h>
#include <string>
#include <fstream>
#include <unordered_map>

class DataLogger{
public:
    DataLogger(int robotID): robotID_(robotID)
    {
        auto line = fmt::format("x, y \n");
        data_ = line;
    }
    void add_point(float x, float y)
    {
        auto line = fmt::format("{}, {} \n", x, y);
        data_ += line;
    }

    void save()
    {
        auto resultDir = "/home/redwan/catkin_ws/src/bovo/results";
        ofstream myfile;
        auto outdir = fmt::format("{}/robo{}.csv", resultDir, robotID_);
        ROS_INFO_STREAM("saving results @ " << outdir);
        myfile.open (outdir);
        myfile << data_;
        myfile.close();
    }
    ~DataLogger()
    {
        save();
    }

private:
    string data_;
    int robotID_;
};


class VizPlan{
public:

    VizPlan(const std::string& topic){
        state_pub_ = nh_.advertise<visualization_msgs::Marker>(topic, 100);

    }
    ~VizPlan()
    {
        logger_.clear();
    }
    template<typename T>
    void update_robot(int robotID, const T& pos, double ori)
    {
        // generate a visualization marker message
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "/viz/state/robot" + std::to_string(robotID);
        marker.id = robotID;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = "package://traj_view/config/ROOMBA.dae";

//         set up scale of the points
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

//        marker.color.a = 0.50;
//        marker.color.r = 0.663;
//        marker.color.g = 0.663;
//        marker.color.b = 0.663;
        marker.color.a = 0.5;
        marker.color.r = 1.0 - robotID;
        marker.color.g = robotID;
        marker.color.b = 0.0;

        marker.pose.position.x = pos.getX();
        marker.pose.position.y = pos.getY();
        marker.pose.position.z = 0.0;


        tf::Quaternion q;
        q.setEuler(0, 0, ori);

        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        state_pub_.template publish(marker);

        if(logger_.find(robotID) == logger_.end())
        {
            logger_[robotID] = new DataLogger(robotID);
        }

        logger_[robotID]->add_point(pos.getX(), pos.getY());
    }

    void save()
    {
        for(auto log:logger_)
        {
            log.second->save();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    unordered_map<int, DataLogger*> logger_;
};
#endif //TRAJ_VIEW_VIZ_PLAN_H
