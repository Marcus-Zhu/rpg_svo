#include <svo_ros/level_map.h>

#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstdio>

const int LENGTH = 20;
const int WIDTH = 20;
const double X_MIN = -2.5;
const double X_MAX = 2.5;
const double Y_MIN = -2.5;
const double Y_MAX = 2.5;
const double MAX_HEIGHT = -0.1;
const double Z_START = -2;

const double sigma = 0.5;
const double mu = 0.0;

double LevelMap::normaldist(const Vector3d &pos1, const Vector3d &pos2)
{
    double dist = sqrt(pow((pos1[0]-pos2[0]),2) + pow((pos1[1]-pos2[1]),2));
    double f_x = (pos1[2]-z_start_)*exp(-1.0*dist*dist/(2.0*sigma*sigma));
    return f_x + z_start_;
}

LevelMap::LevelMap()
{
    length_ = LENGTH;
    width_ = WIDTH;
    x_min_ = X_MIN;
    x_max_ = X_MAX;
    y_min_ = Y_MIN;
    y_max_ = Y_MAX;
    size_ = LENGTH * WIDTH;
    err = 1e-6;
    max_height_ = MAX_HEIGHT;
    z_start_ = Z_START;
    
    map_.clear();
    map_.resize(length_, vector<float>(width_, z_start_));
    updated_.clear();
    updated_.resize(length_, vector<bool>(width_, false));
    x_range_ = x_max_ - x_min_;
    y_range_ = y_max_ - y_min_;
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);
}

void LevelMap::update (const Vector3d& pos)
{
    // convert to grid coordinate
    int x = (pos[0] - x_min_) / x_range_ * length_;
    int y = (pos[1] - y_min_) / y_range_ * width_;
    double h = pos[2];

    // if input range illegal
    if (x < 0 || x >= length_)
    {
        cout << "\033[1;31mx out of range!\033[0;0m\n";
        return;
    }
    if (y < 0 || y >= width_)
    {
        cout << "\033[1;31my out of range!\033[0;0m\n";
        return;
    }

    // update
    // NEED CHANGE, USING NORMAL DISTRIBUTION MODEL
    double f_x = normaldist(pos, pos);
    if(f_x >= max_height_) max_height_ = f_x;

    Vector3d temp;
    temp[2] = 0;
    if (!updated_[x][y] || map_[x][y] < f_x)
    {
        for (int i = max(0,x-2); i < min(length_,x+3); i++)
            for (int j = max(0,y-2); j < min(width_,y+3); j++)
                if (updated_[i][j] == false)
                {
                    temp[0] = 1.0 * (i+0.5) * x_range_ / length_ + x_min_;
                    temp[1] = 1.0 * (j+0.5) * y_range_ / width_ + y_min_;
                    double temp_distance = normaldist(pos, temp);
                    map_[i][j] = temp_distance;
                }

        map_[x][y] = f_x;
        updated_[x][y] = true;
    }
}

void LevelMap::publish()
{
    visualization_msgs::MarkerArray markerarray;
    markerarray.markers.resize(size_);

    for(int i = 0; i < size_; ++i)
    {
        double height = map_[i / length_][i % length_];
        if (height <= z_start_) height = z_start_ + err;

        markerarray.markers[i].header.frame_id = "/world";
        markerarray.markers[i].header.stamp = ros::Time();

        markerarray.markers[i].ns = "levelmap";
        markerarray.markers[i].id = i;
        markerarray.markers[i].type = visualization_msgs::Marker::CUBE;
        markerarray.markers[i].action = visualization_msgs::Marker::ADD;

        markerarray.markers[i].scale.x = x_range_ / length_;
        markerarray.markers[i].scale.y = y_range_ / width_;
        markerarray.markers[i].scale.z = height - z_start_;

        markerarray.markers[i].pose.position.x = (i / length_) * x_range_ / length_ - x_range_ / 2;
        markerarray.markers[i].pose.position.y = (i % length_) * y_range_ / width_ - y_range_ / 2;
        markerarray.markers[i].pose.position.z = markerarray.markers[i].scale.z / 2.0 + z_start_;
        markerarray.markers[i].pose.orientation.x = 0.0;
        markerarray.markers[i].pose.orientation.y = 0.0;
        markerarray.markers[i].pose.orientation.z = 0.0;
        markerarray.markers[i].pose.orientation.w = 1.0;

        double input = (height - z_start_) / (max_height_ - z_start_);
        markerarray.markers[i].color.r = input > 0.5 ? 1 : input * 2;
        markerarray.markers[i].color.g = input > 0.5 ? (1 - input) * 2 : 1;
        markerarray.markers[i].color.b = 0;
        markerarray.markers[i].color.a = 0.5;

        markerarray.markers[i].lifetime = ros::Duration();
    }

    marker_pub.publish(markerarray);
    ros::spinOnce();
}

void LevelMap::print()
{
    cout << "\033[0;33m";
    for (int i = 0; i < length_; i++)
    {
        for (int j = 0; j < width_; j++)
            cout << fixed << showpoint << setprecision(1)
                 << map_[i][j] << " ";
        cout << endl;
    }
    cout << "\033[0;0m";
}
