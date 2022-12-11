

#ifndef LOCATE_H
#define LOCATE_H

#include "LineDetect.h"

using namespace cv;
using namespace std;

class Locate
{
public:

    vector<Point2f> image_p; //图像坐标集合
    vector<Point3f> target_p; //对应目标的世界坐标集合

    vector<Eigen::Vector3f> target_p_cam; //对应目标的相机坐标集合

    Mat camera_matrix;
    Mat dist_coeffs;


    Locate();

    void pnp();

    void computeAngel();
};

#endif