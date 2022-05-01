
#include "hrvo_agent.h"
#include <iostream>
#include <thread>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace hrvo;
using namespace std;
#include <map>
//condition_variable conditionVariable;
//mutex mu;


int Robot::robotID = 0;
struct TRAJECTORY{
    vector<float>x, y;
};

TRAJECTORY robo_traj[2];

void hrvo_callback(int robotID, const Vector2& cmd_vel)
{
    robo_traj[robotID].x.push_back(cmd_vel.getX());
    robo_traj[robotID].y.push_back(cmd_vel.getY());

//    cout << "[callback] robotID " << robotID << " msg " << cmd_vel << endl;
}


int main(int argc, char *argv[])
{


    // simulation setup
    WP path1{{1, 0}, {1.5, 0}, {1.5, 1}, {0, 1}, {0, 2}};
    WP path2{{0, 2}, {1.5, 2}, {1.5, 1}, {0, 1}, {1, 0}};
    vector<WP> paths{path1, path2};
    float radius = 0.345f;
    float sim_time = 1.0f;
    float contrl_time = 0.01f;
    PIDGains gain{0.9, 1.5, 0.3};
    HRVOParams param{20,10, 0.5, 1.0};
    vector<PIDGains> gains{gain, gain};
    MissionSetup mission{paths, radius, sim_time, contrl_time, gains, param};



    MissionCoordinator missionCoordinator;
    missionCoordinator.doSetup(mission, hrvo_callback);
    missionCoordinator.execute();

    int N = robo_traj[0].x.size();
    for (int j = 0; j < N; ++j) {
        for (int robotID = 0; robotID < paths.size(); ++robotID) {
//        plt::scatter(robo_traj[robotID].x, robo_traj[robotID].y, 50);

            vector<float>x{robo_traj[robotID].x[j]}, y{robo_traj[robotID].y[j]};
            map<string, string> cmap;
            cmap["color"] = (robotID == 0)?"red":"blue";
            plt::scatter(x, y, 50, cmap);

        }
        plt::pause(0.001);


    }

    plt::show();


    return 0;
}