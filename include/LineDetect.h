#ifndef LINEDETECT_H
#define LINEDETECT_H

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include <list>
#include <utility>

using namespace std;
using namespace cv;

class LineDecect
{
public:
    Mat edgeDetect(Mat srcImage);

    void edgeKbLine(vector<Point> p, Mat srcImage, int edge_n, int template_size);

    void verticalFindLine(vector<Point> p, Mat srcImage, int edge_n);

    vector<Point> harrisCornorDetect(Mat srcImage, int abs_dist);

    int downSample();

};

#endif