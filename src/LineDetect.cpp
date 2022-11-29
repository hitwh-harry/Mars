#include "../include/LineDetect.h"

//从p的垂直线查找梯度变化大的点
void LineDecect::verticalPoint(list<Point> &p_result, Point p, Point p1, Point p2, int dist)
// p_find存找到的两个点
{
    //找垂线
    double k, b;
    if (p2.y != p1.y)
    {
        k = (double)(p1.x - p2.x) / (p2.y - p1.y); //垂直线的斜率-1 / k
        b = p.y - k * p.x;
    }
    else
    {
        k = DBL_MAX;
    }

    vector<Point> pv; //存垂线上的点

    if (k >= -1 && k <= 1)
    {
        int x = p.x - dist;
        if (x < 0)
            x = 0;
        for (; x - p.x <= dist && x < srcImage.cols; x++)
            pv.push_back(Point(x, k * x + b));
    }
    else if (k == DBL_MAX)
    {
        int y = p.y - dist;
        if (y < 0)
            y = 0;
        for (; y - p.y <= dist && y < srcImage.rows; y++)
            pv.push_back(Point(p.x, y));
    }
    else
    {
        int y = p.y - dist;
        if (y < 0)
            y = 0;
        for (; y - p.y <= dist && y < srcImage.rows; y++)
            pv.push_back(Point((y - b) / k, y));
    }

    int n = pv.size();
    int pixel_value[n]; //垂线上的像素值

    for (int i = 0; i < n; i++)
        pixel_value[i] = (int)srcGray.at<uchar>(pv[i]);

    // for (int i = 0; i < n; i++)
    //     cout<<pixel_value[i]<<" ";
    // cout<<endl;

    // p_result.push_back(pv[i + 1]);
    int start = 0, now = -1;
    Point p_temp = Point(0, 0);
    for (int i = 0, j = 2; j < n; i++, j++)
    {
        if (abs(pixel_value[i] - pixel_value[j]) > 100)
        {
            // cout<<pv[i + 1]<<endl;
            p_temp += pv[i + 1];
            if (start == 0)
            {
                start = i + 1;
                now = i + 1;
            }
            else if (i == now)
                now++;
        }
        else if (i == now && abs(pixel_value[i] - pixel_value[j]) <= 100)
        {
            p_temp /= (now - start + 1);
            p_result.push_back(p_temp);
            start = 0;
        }
    }
}

void LineDecect::verticalFindLine(vector<Point> pv, int edge_n, int dist)
// edge_n为每两个点之间采样几个点
// dist为垂线从多远开始
{
    list<pair<double, double>> line_kb; //储存检测到的直线的k和b
    int p_num = pv.size();
    // for (int i = 0; i < p_num; i++)
    // {
    //     for (int j = i + 1; j < p_num; j++)
    //     {
    //         vector<Point> p_edge; //收集待拟合点
    //         vector<Point> p_edge1;
    //         for (int l = 1; l < edge_n; l++)
    //         {
    //             Point p = pv[i] + (double)l / edge_n * (pv[j] - pv[i]);
    //             list<Point> p_temp;
    //             verticalPoint(p_temp, p, pv[i], pv[j], dist);

    //             if (p_temp.size() >= 2)
    //             {
    //                 p_edge.push_back(p_temp.front());
    //                 p_edge1.push_back(p_temp.back());
    //             }
    //         }

    //         if (p_edge.size() >= 0.8 * edge_n)
    //         {
    //             line_kb.push_back(leastSquare(p_edge));
    //             line_kb.push_back(leastSquare(p_edge1));
    //             for (int l = 0; l < p_edge.size(); l++)
    //             {
    //                 circle(srcImage, p_edge[l], 3, Scalar(0, 0, 255));
    //                 circle(srcImage, p_edge1[l], 3, Scalar(0, 0, 255));
    //             }
    //         }
    //     }
    // }

    // list<pair<double, double>>::iterator it = line_kb.begin();
    // while (it != line_kb.end())
    // {
    //     if ((*it).first != DBL_MAX)
    //         line(srcImage, Point(0, (*it).second), Point(1022, 1022 * (*it).first + (*it).second), Scalar(0, 0, 255));
    //     else
    //         line(srcImage, Point((*it).second, 0), Point((*it).second, 1022), Scalar(0, 0, 255));

    //     it++;
    // }

    int i = 1;
    int j = 3;
    vector<Point> p_edge;
    for (int l = 1; l < edge_n; l++)
    {
        Point p = pv[i] + (double)l / edge_n * (pv[j] - pv[i]);

        list<Point> p_temp;
        verticalPoint(p_temp, p, pv[i], pv[j], dist);
        cout << p_temp.back();
        p_edge.push_back(p_temp.front());
        circle(srcImage, p_temp.front(), 3, Scalar(0, 0, 255));
        // circle(srcImage, p_temp.back(), 3, Scalar(0, 0, 255));
    }

    namedWindow("line", 0);
    imshow("line", srcImage);
    waitKey(0);
}

void LineDecect::edgeFindLine(vector<Point> p, int edge_n, int template_size)
// edge_n为每两个点之间采样几个点
{
    // Canny 边缘检测
    Mat dstImage, edge;
    dstImage.create(srcImage.size(), srcImage.type());

    blur(srcGray, edge, Size(3, 3));
    Canny(edge, edge, 3, 9, 3);

    // imshow("Canny边缘检测", edge);
    // waitKey(0);

    int p_num = p.size();
    int line_num = 0;
    list<pair<double, double>> line_kb; //储存检测到的直线的k和b

    //每两点之间检测是由有足够边缘点
    for (int i = 0; i < p_num; i++)
    {
        for (int j = i + 1; j < p_num; j++)
        {
            double dist = sqrt(pow(p[i].x - p[j].x, 2) + pow(p[i].y - p[j].y, 2));
            if (dist < 1.5 * edge_n * template_size)
                continue;

            vector<Point> p_edge; //收集边缘点
            for (int k = 1; k < edge_n; k++)
            {
                Point pp = p[i] + (double)k / edge_n * (p[j] - p[i]);
                Mat region = edge(Rect(pp.x - template_size / 2, pp.y - template_size / 2, template_size, template_size));
                // cout<<region<<endl;

                //有边缘点，则确定边缘点位置
                if (cv::sum(region)[0] != 0)
                {
                    Point loc;
                    minMaxLoc(region, 0, 0, 0, &loc);
                    p_edge.push_back(pp - Point(template_size / 2, template_size / 2) + loc);
                }

                // circle(srcImage, pp, 3, Scalar(0, 0, 255));
            }

            //若有，则重新拟合直线，保存k和b
            if (p_edge.size() >= 0.8 * edge_n)
            {

                line_num++;
                line_kb.push_back(leastSquare(p_edge));

                // cout << k <<" "<< b << endl << endl;
                // line(srcImage, Point(0, b), Point(1022, 1022 * k + b), Scalar(0, 0, 255));
                // if (line_num == 3)
                // {
                //     cout << p[i] << endl
                //          << p[j] << endl
                //          << endl;
                //     line(srcImage, Point(0, b), Point(1022, 1022 * k + b), Scalar(0, 0, 255));
                //     goto draw;
                // }
            }
        }
    }
    cout << "line num: " << line_num << endl;

    //筛选3对直线
    for (int k = 0; k < 3; k++)
    {
        list<pair<double, double>>::iterator i;
        list<pair<double, double>>::iterator j;

        double max = 0;
        list<pair<double, double>>::iterator m_i;
        list<pair<double, double>>::iterator m_j;

        for (i = line_kb.begin(); i != line_kb.end(); i++)
        {
            j = i;
            j++;
            for (; j != line_kb.end(); j++)
            {
                double x = abs((*i).first / (*j).first);
                double y = abs((*i).second / (*j).second);
                x = x > 1 ? 1 / x : x;
                y = y > 1 ? 1 / y : y;

                if ((*i).first == 0 || (*j).first == 0)
                {
                    x = y;
                }
                else if ((*i).second != 0 && (*j).second != 0)
                {
                    x = (x + y) / 2;
                }

                if (x > max)
                {
                    max = x;
                    m_i = i;
                    m_j = j;
                }
            }
        }

        line(srcImage, Point(0, (*m_i).second), Point(1022, 1022 * (*m_i).first + (*m_i).second), Scalar(0, 0, 255));
        line(srcImage, Point(0, (*m_j).second), Point(1022, 1022 * (*m_j).first + (*m_j).second), Scalar(0, 0, 255));

        // for (i = line_kb.begin(); i != line_kb.end(); i++)
        // {
        //     cout << (*i).first << " " << (*i).second << endl;
        // }
        // cout << endl;

        line_kb.erase(m_i);
        line_kb.erase(m_j);

        for (i = line_kb.begin(); i != line_kb.end(); i++)
        {
            cout << (*i).first << " " << (*i).second << endl;
        }
        cout << endl;
    }
draw:
    namedWindow("line", 0);
    imshow("line", srcImage);
    waitKey(0);
}

pair<double, double> LineDecect::leastSquare(vector<Point> p_edge)
{
    int t1, t2, t3, t4;
    t1 = t2 = t3 = t4 = 0;
    int n = p_edge.size();
    for (int i = 0; i < n; i++)
    {
        t1 += p_edge[i].x;
        t2 += p_edge[i].y;
        t3 += (p_edge[i].x * p_edge[i].y);
        t4 += pow(p_edge[i].x, 2);
    }

    double k, b;
    // k为无穷时b存x的值
    if (n * t4 == t1 * t1)
    {
        k = DBL_MAX;
        b = p_edge[0].x;
    }
    else
    {
        k = (double)(n * t3 - t1 * t2) / (n * t4 - t1 * t1);
        b = (double)(t4 * t2 - t1 * t3) / (n * t4 - t1 * t1);
    }

    return {k, b};
}

Mat LineDecect::edgeDetect()
{
    Mat dstImage, edge;
    dstImage.create(srcImage.size(), srcImage.type());

    blur(srcGray, edge, Size(3, 3));
    Canny(edge, edge, 3, 9, 3);

    imshow("Canny边缘检测", edge);
    waitKey(0);
    return edge;
}

void LineDecect::harrisCornorDetect(vector<Point> &p, int abs_dist)
{
    // Harris corner parameters
    int kThresh = 150;
    int kBlockSize = 3;
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

    // vector<Point>::iterator it = p.begin();
    // while (it != p.end())
    // {
    //     // cout << *it << endl;
    //     circle(srcImage, *it, 5, Scalar(0, 0, 255));
    //     it++;
    // }
    // namedWindow("harris corner", 0);
    // imshow("harris corner", srcImage);
    // waitKey(0);
}

int LineDecect::downSample()
{
    String s1 = "../pictures/";
    String s2 = ".jpg";
    for (int i = 1; i < 9; i++)
    {
        Mat src, src_down, src_down1;
        src = imread(s1 + to_string(i) + s2);
        if (src.empty())
        {
            cout << "image not found" << endl;
            return -1;
        }

        pyrDown(src, src_down, Size(src.cols / 2, src.rows / 2));
        pyrDown(src_down, src_down1, Size(src_down.cols / 2, src_down.rows / 2));

        imwrite(s1 + to_string(i) + "_1" + s2, src_down1);
    }
    return 1;
}

LineDecect::LineDecect(Mat srcImage)
{
    this->srcImage = srcImage;
    cvtColor(srcImage, srcGray, COLOR_BGR2GRAY);
}