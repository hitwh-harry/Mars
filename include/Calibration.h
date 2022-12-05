

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
// #include <opencv2\imgproc\types_c.h>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

class Calibration {
public:
    static void Calibrate();
};

#endif 