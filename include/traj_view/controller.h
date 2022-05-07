//
// Created by redwan on 4/26/22.
//

#ifndef MPCHRVO_CONTROLLER_H
#define MPCHRVO_CONTROLLER_H

#include <vector>
#include <math.h>
#include <tuple>
#include <iostream>
#include <memory>
using namespace std;


class controller;
class StateTransition;
typedef shared_ptr<StateTransition> StateTransitionPtr;

class StateTransition{
public:
    StateTransition(double dt):dt_(dt)
    {
        this->max_w_ = 0.7 * 2;
        this->max_v_ = 1.5 * 2;
    }
    void set_control(const std::pair<double, double>& cmd_vel)
    {
        double v = cmd_vel.first;
        double w = cmd_vel.second;
        double theta = state_[2] + w * dt_;
        state_[0] += v * cos(theta) * dt_;
        state_[1] += v * sin(theta) * dt_;
        state_[2] = theta;
        cmd_vel_ = make_pair(v, w);
    }
    void set_state(const vector<double>& state)
    {
        state_.clear();
        std::copy(state.begin(), state.end(), back_inserter(state_));
    }

    void set_max_vel(double vmax)
    {
        max_v_ = vmax;
    }

    void set_max_yaw_rate(double wmax)
    {
        max_w_ = wmax;
    }
    template <typename T>
    void update_cmd_vel(T* data)
    {
        data = new T{cmd_vel_.first, cmd_vel_.second};
    }

    template <typename T>
    void update_state(T& data)
    {
        for (int i = 0; i < state_.size(); ++i) {
            data[i] = state_[i];
        }
    }
    double clip_vel(double v)
    {
        return clip(v, max_v_);
    }
    double clip_yaw_rate(double w)
    {
        return clip(w, max_w_);
    }
protected:
    double clip(double val, double clip_val){
        auto sign = (val < 0)? -1 : 1;
        auto x = abs(val);
        val = min(x, clip_val) * sign;
        return val;
    }
private:
    vector<double> state_;
    std::pair<double, double> cmd_vel_;
    double dt_, max_v_, max_w_;
};


class controller {
public:
    /**
     *
     * @param state initial state of the vehicle
     * @param kpRho primarily responsible for linear velocity
     * @param kpAlpha primarily responsible for angular velocity
     * @param kpBeta 0 responsible for orientation
     * @param dt control time
     */
    controller(const vector<double>&state, double kpRho, double kpAlpha, double kpBeta, double dt, double goal_thres = 0.01);
    void set_points(double x, double y);
    std::pair<double, double> compute_control(StateTransitionPtr transition);
    bool isFinished();
    /**
     * get robot state [x, y, theta]
     */
    vector<double> get_state();


private:
    double kp_rho_, kp_alpha_, kp_beta_;
    vector<double> state_, goal_;
    double dt_, rho_;
    const double goal_thres_;


protected:
    /**
     *
     * @param x_diff The position of target with respect to current robot position
                 in x direction
     * @param y_diff The position of target with respect to current robot position
                 in y direction
     * @param theta The current heading angle of robot with respect to x axis
     * @param theta_goal The target angle of robot with respect to x axis
     * Returns
        -------
        rho : The distance between the robot and the goal position
        v : Command linear velocity
        w : Command angular velocity
     */
    std::tuple<double, double, double> calc_control_command(double x_diff, double y_diff, double theta, double theta_goal);

};


#endif //MPCHRVO_CONTROLLER_H
