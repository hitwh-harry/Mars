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
    Mat srcImage;
    Mat srcGray;

public:
    LineDecect(Mat srcImage);

    Mat edgeDetect();

    void edgeFindLine(vector<Point> p, int edge_n, int template_size);

    list<Point> verticalPoint(Point p, Point p1, Point p2, int dist);

    void verticalFindLine(vector<Point> pv, int edge_n, int dist);

    vector<Point> harrisCornorDetect(int abs_dist);

    int downSample();
};

#endif