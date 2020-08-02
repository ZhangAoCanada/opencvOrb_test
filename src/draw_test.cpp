#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include "pangolin_slamdraw.h"

using namespace std;
using namespace cv;

Mat getR(float alpha, float beta, float gamma)
{
    Mat R_x = Mat(3, 3, CV_32F);
    Mat R_y = Mat(3, 3, CV_32F);
    Mat R_z = Mat(3, 3, CV_32F);

    R_x.at<float>(0,0) = 1;
    R_x.at<float>(0,1) = 0;
    R_x.at<float>(0,2) = 0;

    R_x.at<float>(1,0) = 0;
    R_x.at<float>(1,1) = cos(alpha);
    R_x.at<float>(1,2) = -sin(alpha);

    R_x.at<float>(2,0) = 0;
    R_x.at<float>(2,1) = sin(alpha);
    R_x.at<float>(2,2) = cos(alpha);

    return R_x;
}

Mat getT(float x, float y, float z)
{
    Mat T = Mat(3, 1, CV_32F);

    T.at<float>(0) = x;
    T.at<float>(1) = y;
    T.at<float>(2) = z;

    return T;
}

int main()
{
    morb::MorbDraw monoDraw;
    vector<Point3f> random_pnts;
    vector<Point3f> pnts_colors;
    Point3f tmp_pnt;
    Point3f tmp_color;
    vector<Mat> R_vec;
    vector<Mat> T_vec;
    Mat R;
    Mat T;

    for (int i=0; i<10000; i++)
    {
        tmp_pnt.x = (float) rand() / (RAND_MAX) * 10;
        tmp_pnt.y = (float) rand() / (RAND_MAX) * 10;
        tmp_pnt.z = (float) rand() / (RAND_MAX) * 10;
        tmp_color.x = 1 - tmp_pnt.x / 10;
        tmp_color.y = 1 - tmp_pnt.y / 10;
        tmp_color.z = 1 - tmp_pnt.z / 10;
        random_pnts.push_back(tmp_pnt);
        pnts_colors.push_back(tmp_color);
    }

    R = Mat::eye(3, 3, CV_32F);
    T = Mat::zeros(3, 1, CV_32F);
    R_vec.push_back(R.clone());
    T_vec.push_back(T.clone());

    R = getR(M_PI_4, 0, 0) * R;
    T = getT(0.5, 0.5, 0.5) + R*T;
    R_vec.push_back(R.clone());
    T_vec.push_back(T.clone());

    R = getR(M_PI_4, 0, 0) * R;
    T = getT(0.5, 0.5, 0.5) + R*T;
    R_vec.push_back(R.clone());
    T_vec.push_back(T.clone());

    monoDraw(random_pnts, pnts_colors, R_vec, T_vec);

    return 0;
}
