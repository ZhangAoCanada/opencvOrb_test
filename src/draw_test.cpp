#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <random>
#include "pangolin_slamdraw.h"

using namespace std;
using namespace cv;

int main()
{
    morb::MorbDraw monoDraw;
    vector<Point3f> random_pnts;
    vector<Point3f> pnts_colors;
    Point3f tmp_pnt;
    Point3f tmp_color;
    vector<Mat> R_vec;
    vector<Mat> T_vec;
    Mat R = Mat::eye(3, 3, CV_64FC1);
    Mat T = Mat::zeros(3, 1, CV_64FC1);

    for (int i=0; i<10000; i++)
    {
        tmp_pnt.x = (double) rand() / (RAND_MAX) * 10;
        tmp_pnt.y = (double) rand() / (RAND_MAX) * 10;
        tmp_pnt.z = (double) rand() / (RAND_MAX) * 10;
        tmp_color.x = 1 - tmp_pnt.x / 10;
        tmp_color.y = 1 - tmp_pnt.y / 10;
        tmp_color.z = 1 - tmp_pnt.z / 10;
        random_pnts.push_back(tmp_pnt);
        pnts_colors.push_back(tmp_color);
    }

    R_vec.push_back(R);
    T_vec.push_back(T);
    monoDraw(random_pnts, pnts_colors, R_vec, T_vec);

    return 0;
}