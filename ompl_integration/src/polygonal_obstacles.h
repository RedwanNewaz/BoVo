//
// Created by redwan on 4/3/22.
//

#ifndef VFRRT_POLYGONAL_OBSTACLES_H
#define VFRRT_POLYGONAL_OBSTACLES_H

#include <iostream>
#include <memory>
#include <vector>
#include <numeric>

using namespace std;

class polygonal_obstacles;
typedef shared_ptr<polygonal_obstacles> ObstclesPtr;

/**
 * The key idea is to check point inside a polygon
 * https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
 */

class polygonal_obstacles: public enable_shared_from_this<polygonal_obstacles>{
    using OBS = pair<vector<float>, vector<float>>;
    struct Point
    {
        float x;
        float y;
    };
public:
    polygonal_obstacles(float lower, float higher);
    void append(const vector<float>&x, const vector<float>&y);
    ObstclesPtr get_ptr();
    bool isValidState(float x, float y);
    vector<OBS> get_obstacles();
    float low, high;


private:
    vector<OBS> obstacles_;
    const float offset_ = 1;

protected:
    vector<Point> obstacle_to_points(const OBS& obs);
    /*
     * Returns true if the point p lies inside the polygon[] with n vertices
     */
    bool isInside(Point polygon[], int n, Point p);
    /*
     * Given three collinear points p, q, r, the function checks if
     * point q lies on line segment 'pr'
     */
    bool onSegment(Point p, Point q, Point r);
    /**
     * To find orientation of ordered triplet (p, q, r).
     * The function returns following values
     * 0 --> p, q and r are collinear
     * 1 --> Clockwise
     * 2 --> Counterclockwise
     */
    int orientation(Point p, Point q, Point r);
    /*
     * The function that returns true if line segment 'p1q1'
     * and 'p2q2' intersect.
     */
    bool doIntersect(Point p1, Point q1, Point p2, Point q2);

};


#endif //VFRRT_POLYGONAL_OBSTACLES_H
