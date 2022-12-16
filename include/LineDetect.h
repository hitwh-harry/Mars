#ifndef LINEDETECT_H
#define LINEDETECT_H

#include "Util.h"

class LineDecect
{
    Mat srcImage;
    Mat srcGray;

    Mat targetImg; //模板匹配后的图像

    // void verticalPoint(list<Point> &p_result, Point p, Point p1, Point p2, int dist);

public:
    LineDecect(Mat srcImage);

    Mat edgeDetect();

    // void edgeFindLine(vector<Point> p, int edge_n, int template_size);

    // void verticalFindLine(vector<Point> pv, int edge_n, int dist);

    void templateMatch(Mat temp);

    void harrisCornorDetect(vector<Point> &p, int abs_dist);

    void houghDetect();

};

#endif