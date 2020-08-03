#ifndef MONO_ORB_SLAM_H
#define MONO_ORB_SLAM_H

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

namespace morb
{
#define IMAGE_SCALEFACTOR 1.0
#define NUM_CORNERS_TO_DETECT 3000
#define KNN_MATCHER_DISTANCE_THRESHOLD 100

#define MONO_SCALE 0.1
#define MAX_ANGLE 3.141593/4
#define MAX_TRANSLATE 10
#define PCL_DISTANCE_UPPER 10000 
#define PCL_DISTANCE_LOWER 0

class MorbCV
{
    public:
        MorbCV();
        void operator() (Mat img);
        void imgPlot(Mat img);
        void orbKeyPointMatch(Mat img1, Mat img2);
        void getPosition();
        void getPointCloud(vector<Point2f> kp1, vector<Point2f> kp2, vector<Point3f> colors);

        vector<Mat> R_all, t_all;
        vector<Point3f> pcl_all, pcl_colors;
        Mat camera_matrix, E, R, t, R_world, t_world, this_R, this_T;

    protected:
        void readCameraIntrinsic();
        void RMatToMaxAngles(Mat R);
        void TToMaxDistance(Mat T);

        string camintrinsic_filename;
        BFMatcher matcher;
        Ptr<ORB> orb_cv = ORB::create();

        Mat prev_img, img_out;
        double max_angle, max_translate;
        float scale;
};
}

#endif