//
// Created by redwan on 4/28/22.
//
#include "src/mrm_planner.h"
#include "src/map_parser.h"

using namespace std;

int Robot::robotID = 0;

void motion_planning_demo(ObstclesPtr obstacles, int N=2)
{
    MMP mmp(obstacles);

    //disable cout output in the runtime

    std::cout.setstate(std::ios_base::failbit);
    const vector<float> s{0, 1}, g{20, 15};

    string image_path = "../../resources/chlorophyll.png";
    MapParser z_map(image_path, obstacles);

    TRAJECTORY meas_traj[N];
    auto path1 = mmp.generate_path(s, g );
    auto path2 = mmp.generate_path( g, s);


    int elapsed = 0;
    bool isValid = false;
    int iteration = 0;
    const auto start = std::chrono::steady_clock::now();

    do {
        vector<WP> paths{path1, path2};
        TRAJECTORY robo_traj[N];
        mmp.generate_motion(paths, robo_traj);

        isValid = !mmp.isCollision(robo_traj[0], robo_traj[1]);
        if(isValid)
        {
            for (int i = 0; i < N; ++i) {
                z_map.add_path(paths[i]);
                robo_traj[i].copy(meas_traj[i]);
            }
        }
        const auto end = std::chrono::steady_clock::now();

        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        ++iteration;

    }while( !isValid && elapsed < 15000.0);


    if(isValid)
    {
//        cout << meas_traj[0].x.size() << endl;
        z_map.add_meas_to_traj(meas_traj, N);
//        cout << meas_traj[0] << endl;
        auto f = [&](float x, float y, unsigned long k)
        {
            return mmp.get_neightbors(x, y, k);
        };
        z_map.animation(meas_traj, N, f);
    }

    // enable cout again
    std::cout.clear();
    std::cout << "[Plan] found valid plan at iteration = " << iteration << " | elapsed time " << elapsed << " ms\n";

}



int main(int argc, char *argv[])
{
    auto obstacles = make_shared<polygonal_obstacles>(0, 25);
    //TODO read obstacles_ from a map
    vector<float> x0{10, 15, 15, 10, 10}, x1{ 2, 3, 7, 7, 2, 2}, y0{2, 2, 15, 15, 2}, y1{15, 10, 15, 20, 20, 15};
    obstacles->append(x0, y0);
    obstacles->append(x1, y1);

    motion_planning_demo(obstacles);

    return 0;
}
