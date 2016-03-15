#ifndef __level_map__
#define __level_map__

#include <ros/ros.h>
#include <vector>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

class LevelMap
{
private:
    vector<vector<bool> > updated_;
    vector<vector<float> > map_;

    // grid number in length and width direction
    // size = length * width
    int length_, width_, size_;

    // range of level map
    double x_min_, x_max_, y_min_, y_max_;
    double x_range_, y_range_;
    // start level(height) of the plain
    double z_start_;
    // max_height is used to determine color of the column
    double err, max_height_;

    // ros publisher
    ros::NodeHandle n;
    ros::Publisher marker_pub;

    //f(x)=N(mu,sigma^2)
    double normaldist(const Vector3d &pos1, const Vector3d &pos2);

public:
    LevelMap();
    ~LevelMap() {}

    // update the map using a 3d point
    void update(const Vector3d &pos);
    // publish the whole map using ros
    void publish();
    // print the map in console
    void print();
};

#endif //__level_map__
