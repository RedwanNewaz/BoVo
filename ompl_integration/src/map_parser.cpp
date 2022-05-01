//
// Created by redwan on 4/30/22.
//

#include "map_parser.h"



MapParser::MapParser(const string& filename, ObstclesPtr obs):obs_(obs)
{
    img_ = cv::imread(filename, 0);

}

int MapParser::image_scale(float X, float img_max)
{
    float img_min = 0;
    float X_std = (X - obs_->low) / (obs_->high - obs_->low);
    int X_scaled = X_std * (img_max - img_min) + img_min;
    return X_scaled;
}

void MapParser::add_path(const WP& path)
{
    PWP res;
    int height = img_.size[0]; // y_axis
    int width = img_.size[1]; // x_axis

    for (int i = 0; i < path.size() - 1; ++i) {
        int x1 = image_scale(path[i][0], width);
        int y1 = height - image_scale(path[i][1], height);
        int x2 = image_scale(path[i+1][0], width);
        int y2 = height - image_scale(path[i+1][1], height);
        res.emplace_back(make_pair(cv::Point(x1, y1), cv::Point(x2, y2)));
    }
    lines_.emplace_back(res);

}

int MapParser::get_reading(float rx, float ry)
{
    if( (obs_->low <= rx < obs_->high) && (obs_->low <= ry < obs_->high))
    {
        int height = img_.size[0]; // y_axis
        int width = img_.size[1]; // x_axis
        int x = image_scale(rx, width);
        int y = height - image_scale(ry, height);
        int pixelValue = (int)img_.at<uchar>(x, y);
        return pixelValue;
    }
    return -1;
}

void MapParser::add_meas_to_traj(TRAJECTORY *rtraj, int N)
{
    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < rtraj[k].size(); ++i) {
            rtraj[k].z.push_back(get_reading(rtraj[k].x[i], rtraj[k].y[i]));
        }
    }

}


void MapParser::animation(const TRAJECTORY* motion, int N, const std::function<WP(float, float, unsigned long)>&callback)
{
    auto img = background_show();
    int height = img_.size[0]; // y_axis
    int width = img_.size[1]; // x_axis
    size_t max_len = 0;
    // if robots have different trajectory we need to figure out when to stop iteration
    // for the one which has shorter trajectory length
    for (int i = 0; i < N; ++i) {
        auto rtraj_len = motion[i].x.size();
        max_len = max(rtraj_len, max_len);
    }
    cv::Scalar colors[] ={{0,0,255}, {0,255,255}, {255,0,255}, {79,79,47}, {105,105,105}, {144,128,112} };
    for (int i = 0; i < max_len; ++i) {
        auto working_img = img.clone();
        for (int j = 0; j < N; ++j) {

            // this robot already reached its destination, so don't iterate more!
            if(i >= motion[j].x.size())
                continue;


            cout << "[Robot]" << j << " | z = "<< get_reading(motion[j].x[i], motion[j].y[i]) << endl;
            float rx(motion[j].x[i]), ry(motion[j].y[i]); // robot position in workspace
            int x = image_scale(rx, width);
            int y = height - image_scale(ry, height);
            int radius = image_scale(0.345, width);


            cv::circle(
                    working_img,
                    cv::Point(x, y),
                    radius,
                    colors[j],
                    cv::FILLED,
                    cv::LINE_8
            );

            if(callback)
            {
                // this callback will return neighbor milestones from current locations based on the PRM road net
                unsigned long K_neigh = 20;
                for (const auto& v :callback(rx, ry,K_neigh))
                {
                    cout << v[0] << " " << v[1] << endl;
                    int xx = image_scale(v[0], width);
                    int yy = height - image_scale(v[1], height);

                    cv::circle(
                            working_img,
                            cv::Point(xx, yy),
                            radius/3,
                            cv::Scalar(0, 255, 0),
                            cv::FILLED,
                            cv::LINE_8
                    );
                }
            }
        }
        cv::imshow("frame", working_img);
        cv::waitKey(200);
    }

    cv::waitKey(0);

}

cv::Mat MapParser::background_show()
{
    auto img = img_.clone();
    // draw obstacles_
    int height = img_.size[0]; // y_axis
    int width = img_.size[1]; // x_axis

    // draw obstacles
    auto obstacles = obs_->get_obstacles();
    for(const auto &obs:obstacles)
    {
        int N = obs.first.size();
        cv::Point obs_points[1][N];
        int lineType = cv::LINE_8;
        for (int i = 0; i < N; ++i) {
            int x = image_scale(obs.first[i], width);
            int y = height - image_scale(obs.second[i], height);
            obs_points[0][i]  = cv::Point(x, y);
        }
        const cv::Point* ppt[1] = { obs_points[0] };
        int npt[] = { N };
        cv::fillPoly( img,
                      ppt,
                      npt,
                      1,
                      cv::Scalar( 0, 0, 0 ),
                      lineType );
    }

    // show line
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    if(!lines_.empty())
    {
        int thickness = 2;
        int lineType = cv::LINE_8;
        for(const auto& path : lines_)
            for(const auto& seg: path)
                cv::line( img,
                          seg.first,
                          seg.second,
                          cv::Scalar( 255, 0, 0 ),
                          thickness,
                          lineType );
    }

    return img;
}
