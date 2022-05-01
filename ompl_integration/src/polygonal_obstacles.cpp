//
// Created by redwan on 4/3/22.
//

#include "polygonal_obstacles.h"

polygonal_obstacles::polygonal_obstacles(float lower, float higher):low(lower), high(higher)
{

}

void polygonal_obstacles::append(const vector<float> &x, const vector<float> &y) {

    obstacles_.emplace_back(x, y);
}

ObstclesPtr polygonal_obstacles::get_ptr() {
    return shared_from_this();
}

vector<polygonal_obstacles::Point> polygonal_obstacles::obstacle_to_points(const polygonal_obstacles::OBS &obs) {
    vector<Point> results;
    results.reserve(obs.first.size());
    // last point is not need if it is same as first point
    for (int i = 0; i < obs.first.size() - 1; ++i)
    {
        results.emplace_back(Point{obs.first[i], obs.second[i]});
    }
    return results;
}

bool polygonal_obstacles::isInside(polygonal_obstacles::Point *polygon, int n, polygonal_obstacles::Point p) {
    // There must be at least 3 vertices in polygon[]
    if (n < 3) return false;

    // Create a point for line segment from p to infinite
    Point extreme = {1000, p.y};

    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%n;

        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(polygon[i], polygon[next], p, extreme))
        {
            // If the point 'p' is collinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(polygon[i], p, polygon[next]) == 0)
                return onSegment(polygon[i], p, polygon[next]);

            count++;
        }
        i = next;
    } while (i != 0);

    // Return true if count is odd, false otherwise
    return count&1; // Same as (count%2 == 1)
}

bool polygonal_obstacles::onSegment(polygonal_obstacles::Point p, polygonal_obstacles::Point q,
                                    polygonal_obstacles::Point r) {

    if (q.x - offset_ <= max(p.x, r.x) && q.x + offset_ >= min(p.x, r.x) &&
        q.y - offset_ <= max(p.y, r.y) && q.y + offset_ >= min(p.y, r.y))
        return true;
    return false;
}

int polygonal_obstacles::orientation(polygonal_obstacles::Point p, polygonal_obstacles::Point q,
                                     polygonal_obstacles::Point r) {
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0; // collinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool polygonal_obstacles::doIntersect(polygonal_obstacles::Point p1, polygonal_obstacles::Point q1,
                                      polygonal_obstacles::Point p2, polygonal_obstacles::Point q2) {
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}

bool polygonal_obstacles::isValidState(float x, float y) {
    if (obstacles_.empty())return true;

    for(const auto& obs: obstacles_)
    {
        auto poly = obstacle_to_points(obs);
        if(isInside(poly.data(), poly.size(), Point{x, y}))
        {
            return false;
        }
    }
    return true;
}

vector<polygonal_obstacles::OBS> polygonal_obstacles::get_obstacles() {
    return obstacles_;
}
