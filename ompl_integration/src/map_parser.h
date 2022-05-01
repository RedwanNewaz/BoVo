//
// Created by redwan on 4/30/22.
//

#ifndef MPCHRVO_MAP_PARSER_H
#define MPCHRVO_MAP_PARSER_H
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>

#include "polygonal_obstacles.h"
#include "helper.h"

using namespace std;




class MapParser{
    using PWP = vector<pair<cv::Point, cv::Point>>; // piecewise path
    using WP = vector<vector<float>>;
public:

    MapParser(const string& filename, ObstclesPtr obs);
    /**
     * convert coordinate value from workspace to image domain
     * @param X scalar coordinate value in workspace e.g., x
     * @param img_max maximum axis in image domain
     * @return coordinate in image domain
     */
    int image_scale(float X, float img_max);
    /**
     * for visualization add geometric path from PRM planner
     * @param path geometric path vector<vector<float>>
     */
    void add_path(const WP& path);

    /**
     * obtain field intensity for a particular point
     * @param rx robot x coordinate
     * @param ry robot y coordinate
     * @return z measured value from the image i.e., filed intensity
     */
    int get_reading(float rx, float ry);

    /**
     * add in measurement value as an additional dim (z dim) for each robot trajectory
     * @param rtraj robot trajectory with position values
     * @param N number of robots
     */
    void add_meas_to_traj(TRAJECTORY *rtraj, int N);

    /**
     * visualize motion animation using open cv
     * @param motion trajectories from HRVO planner
     * @param N number of robots
     */
    void animation(const TRAJECTORY* motion, int N=2, const std::function<WP(float, float, unsigned long)>&callback = nullptr);
protected:
    cv::Mat background_show();
private:
    cv::Mat img_;
    ObstclesPtr obs_;
    vector<PWP> lines_;
};



#endif //MPCHRVO_MAP_PARSER_H
