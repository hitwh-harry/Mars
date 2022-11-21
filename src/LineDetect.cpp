#include "LineDetect.h"

Mat LineDecect::edgeDetect(Mat img)
{
    Mat dstImage, edge, grayImage;
    dstImage.create(img.size(), img.type());
    cvtColor(img, grayImage, COLOR_BGR2GRAY);
    blur(grayImage, edge, Size(3, 3));
    Canny(edge, edge, 3, 9, 3);

    imshow("Canny边缘检测", edge);
    waitKey(0);
    return edge;
}

void LineDecect::confirmLine(vector<Point> p, Mat srcImage, int edge_n, int template_size)
// edge_n为每两个点之间采样几个点
{
    // Canny 边缘检测
    Mat dstImage, edge, grayImage;
    dstImage.create(srcImage.size(), srcImage.type());
    cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);
    blur(grayImage, edge, Size(3, 3));
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
                int t1, t2, t3, t4;
                t1 = t2 = t3 = t4 = 0;
                int n = p_edge.size();
                for (int k = 0; k < n; k++)
                {
                    t1 += p_edge[k].x;
                    t2 += p_edge[k].y;
                    t3 += (p_edge[k].x * p_edge[k].y);
                    t4 += pow(p_edge[k].x, 2);
                }
                double k = (double)(n * t3 - t1 * t2) / (n * t4 - t1 * t1);
                double b = (double)(t4 * t2 - t1 * t3) / (n * t4 - t1 * t1);
                line_kb.push_back({k, b});

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

        for (i = line_kb.begin(); i != line_kb.end(); i++)
        {
            cout << (*i).first << " " << (*i).second << endl;
        }
        cout << endl;

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

vector<Point> LineDecect::harrisCornorDetect(Mat srcImage, int abs_dist)
{
    // Harris corner parameters
    int kThresh = 150;
    int kBlockSize = 3;
    int kApertureSize = 3;
    double k = 0.04;

    Mat src_gray;
    cvtColor(srcImage, src_gray, COLOR_BGR2GRAY);

    Mat dst, dst_norm, dst_norm_scaled;
    // Harris corner detect
    cornerHarris(src_gray, dst, kBlockSize, kApertureSize, k);

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

    // vector<Point> v_new;

    // vector<Point>::iterator i = v.begin();
    // vector<Point>::iterator j = v.begin();
    // j++;
    // n = 1;
    // while (i != v.end())
    // {
    //     if ((j == v.end()) || (abs((*i).x - (*j).x) + abs((*i).y - (*j).y) >= 4))
    //     {
    //         Point p = Point(0, 0);
    //         while ((*i) != (*j))
    //         {
    //             p = p + (*i);
    //             i++;
    //         }
    //         p /= n;
    //         v_new.push_back(p);

    //         n = 1;
    //     }
    //     else
    //     {
    //         n++;
    //     }
    //     j++;
    // }

    // cout << "final cornor: " << v_new.size() << endl;
    vector<Point> p;
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

    return p;
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