#ifndef PANGOLIN_SLAMDRAW_H
#define PANGOLIN_SLAMDRAW_H

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "mono_orb_slam.h"

using namespace std;
using namespace cv;

namespace morb
{
class MorbCV;

class MorbDraw
{
public:
    MorbDraw(string v_path, bool follow_c):video_path(v_path), follow_camera(follow_c){};
    void operator()();
    void drawPoints(vector<Point3f> all_points, vector<Point3f> all_colors);
    void drawPath(vector<Mat> R_vec, vector<Mat> T_vec);
    void drawCam();
    void drawLine(Mat T);
    void getHormoMat(Mat R, Mat T);
    void transferToPangoMat(Mat R, Mat T);

protected:
    string video_path;
    bool follow_camera;
    pangolin::OpenGlMatrix draw_mat;
    int hm_frames;
    Mat R;
    Mat T;
    Mat H = Mat(4, 4, CV_32F);
    Mat prev_T;

    // for drawing the camera
    const float w = 0.05;
    const float h = w*0.75;
    const float z = w*0.6;

};

}

#endif
