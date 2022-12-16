#include "LineDetect.h"

void LineDecect::houghDetect()
{
    Mat image_gray, dst;
    Canny(targetImg, image_gray, 130, 200); // 100,200分别是低阈值和高阈值输出二值图
    // namedWindow("img", 0);
    // imshow("img", image_gray);
    // waitKey(0);
    // cvtColor(image_gray, dst, COLOR_GRAY2BGR);

    vector<Vec4f> plines;                                          // 吧每个像素点的平面坐标转化为极坐标产生的曲线放入集合中
    HoughLinesP(image_gray, plines, 1, CV_PI / 180.0, 20, 0, 40); // 从平面坐标转换到霍夫空间,最终输出的是直线的两个点（x0,y0,x1,y1）

    cout << "line num: " << plines.size() << endl;
    for (size_t i = 0; i < plines.size(); i++)
    {
        Vec4f hline = plines[0];
        line(targetImg, Point(hline[0], hline[1]), Point(hline[2], hline[3]), Scalar(0, 0, 255), 1);
    }
    namedWindow("hough", 0);
    imshow("hough", targetImg);
    waitKey(0);
}

void LineDecect::templateMatch(Mat temp)
{
    Mat result;

    matchTemplate(srcImage, temp, result, TM_CCOEFF);
    normalize(result, result, 0, 1, NORM_MINMAX, -1); // 归一化到0-1范围
    double minValue, maxValue;
    Point minLoc, maxLoc;

    minMaxLoc(result, &minValue, &maxValue, &minLoc, &maxLoc);

    // 裁剪复制，取中间元素填充边缘
    cv::Vec3b pixel = srcImage.at<cv::Vec3b>(maxLoc.y + temp.cols / 2, maxLoc.x + temp.rows / 2);
    
    cv::Rect area(maxLoc.x, maxLoc.y, temp.cols, temp.rows); // 裁剪区域的矩形表示
    cv::Mat roi = srcImage(area);

    cv::Mat frame_tape = cv::Mat::zeros(srcImage.rows, srcImage.cols, CV_8UC3);
    frame_tape.setTo(cv::Scalar(pixel[0], pixel[1], pixel[2]));

    cv::Rect roi_rect = cv::Rect(maxLoc.x, maxLoc.y, temp.cols, temp.rows);
    roi.copyTo(frame_tape(roi_rect));

    // rectangle(srcImage, maxLoc, Point(maxLoc.x + temp.cols, maxLoc.y + temp.rows), Scalar(0, 255, 0), 2, 8);
    // namedWindow("dst", 0);
    // imshow("dst", frame_tape);
    // waitKey(0);

    this->targetImg = frame_tape.clone();
}

void LineDecect::harrisCornorDetect(vector<Point> &p, int abs_dist)
{
    // Harris corner parameters
    int kThresh = 160;
    int kBlockSize = 5;
    int kApertureSize = 3;
    double k = 0.04;

    Mat dst, dst_norm, dst_norm_scaled;
    // Harris corner detect
    cornerHarris(srcGray, dst, kBlockSize, kApertureSize, k);

    normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1);
    convertScaleAbs(dst_norm, dst_norm_scaled);

    vector<pair<pair<double, double>, int>> v;

    // find detected corners
    for (int j = 0; j < dst_norm.rows; j++)
    {
        for (int i = 0; i < dst_norm.cols; i++)
        {
            if ((int)dst_norm.at<float>(j, i) > kThresh)
            {
                if (v.size() != 0)
                {
                    int n = v.size();
                    bool flag = false;
                    for (int k = n - 1; k >= 0; k--)
                    {
                        if (abs(v[k].first.first - i) + abs(v[k].first.second - j) <= abs_dist)
                        {
                            v[k].first.first = v[k].first.first * v[k].second / (v[k].second + 1) + (double)i / (v[k].second + 1);
                            v[k].first.second = v[k].first.second * v[k].second / (v[k].second + 1) + (double)j / (v[k].second + 1);
                            v[k].second++;
                            flag = true;
                            break;
                        }
                    }
                    if (!flag)
                    {
                        v.push_back({{(double)i, (double)j}, 1});
                    }
                }
                else
                {
                    v.push_back({{(double)i, (double)j}, 1});
                }
            }
        }
    }

    cout << "find cornor: " << v.size() << endl;

    // cout << "final cornor: " << v_new.size() << endl;

    for (int i = 0; i < v.size(); i++)
    {
        p.push_back(Point((int)v[i].first.first, (int)v[i].first.second));
    }

    vector<Point>::iterator it = p.begin();
    while (it != p.end())
    {
        cout << *it << endl;
        circle(srcImage, *it, 2, Scalar(0, 0, 255), 1);
        it++;
    }
    namedWindow("harris corner", 0);
    imshow("harris corner", srcImage);
    waitKey(0);
}

LineDecect::LineDecect(Mat srcImage)
{
    this->srcImage = srcImage;
    cvtColor(srcImage, srcGray, COLOR_BGR2GRAY);
}