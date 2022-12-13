#include "Locate.h"

Locate::Locate()
{
    // image_p.push_back(Point2f(245, 336));
    // image_p.push_back(Point2f(241, 175));
    // image_p.push_back(Point2f(398, 173));
    // image_p.push_back(Point2f(405, 329));

    image_p.push_back(Point2f(254, 412));
    image_p.push_back(Point2f(257, 254));
    image_p.push_back(Point2f(405, 255));
    image_p.push_back(Point2f(417, 409));

    // image_p.push_back(Point2f(273, 347));
    // image_p.push_back(Point(274, 198));
    // image_p.push_back(Point2f(420, 198));
    // image_p.push_back(Point2f(434, 344));

    // image_p.push_back(Point2f(274, 353));
    // image_p.push_back(Point(280, 216));
    // image_p.push_back(Point2f(418, 220));
    // image_p.push_back(Point2f(434, 356));

    target_p.push_back(Point3f(-66, -66.5, 0));
    target_p.push_back(Point3f(-66, 66.5, 0));
    target_p.push_back(Point3f(66, 66.5, 0));
    target_p.push_back(Point3f(66, -66.5, 0));

    camera_matrix = (cv::Mat_<double>(3, 3) << 496.81, 0, 319.77, 0, 496.93, 235.55, 0, 0, 1);
    dist_coeffs = (cv::Mat_<double>(5, 1) << 0.1, -0.31, -0.008, -0.001, -1.248);
}

// 计算对应点的相机坐标
void Locate::pnp()
{
    Mat rvec;
    Mat tvec;

    solvePnP(target_p, image_p, camera_matrix, dist_coeffs, rvec, tvec, false, CV_P3P);

    Mat rotMat = Mat(3, 3, CV_64FC1, Scalar::all(0));
    cv::Rodrigues(rvec, rotMat); // 转换为旋转矩阵

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> R_n;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T_n;
    cv2eigen(rotMat, R_n);
    cv2eigen(tvec, T_n);

    Eigen::Vector3f v_3f;
    Eigen::Vector3f pw;
    for (int i = 0; i < target_p.size(); i++)
    {
        v_3f << target_p[i].x, target_p[i].y, target_p[i].z;
        pw = R_n * v_3f + T_n;
        target_p_cam.push_back(pw);
        // cout<<pw<<endl<<endl;
    }
}

void Locate::computeAngel()
{
    Eigen::Vector3f v1 = target_p_cam[2] - target_p_cam[1];
    Eigen::Vector3f v2 = target_p_cam[3] - target_p_cam[1];
    Eigen::Vector3f normal = v1.cross(v2);

    Eigen::Vector3f z(0, 0, 1);
    double angel = acos(z.dot(normal) / (z.norm() * normal.norm())) * 180 / M_PI;
    angel = angel > 90 ? 180 - angel : angel;

    cout << angel << endl;
}