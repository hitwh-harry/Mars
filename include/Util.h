#ifndef UTIL_H
#define UTIL_H

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
// #include "opencv2/calib3d.hpp"


#include <vector>
#include <list>
#include <utility>
#include "string.h"

#include <iostream>
#include <fstream>


using namespace std;
using namespace cv;

class Util {
public:
    static pair<double, double> leastSquare(vector<Point> p_edge); //最小二乘法

    static void calibrate();

    static void downSample();
};

#endif 