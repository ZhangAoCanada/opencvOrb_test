#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>
#include <string>

using namespace cv;
using namespace std;

namespace morb
{
class MorbCV
{
    public:
        MorbCV();
        void operator() (Mat img);
        void imgPlot(Mat img);
        void orbKeyPointMatch(Mat img1, Mat img2);

    protected:
        BFMatcher matcher;
        Ptr<ORB> orb_cv = ORB::create();
        vector<KeyPoint> kp1, kp2;
        Mat descriptor1, descriptor2;
        vector<DMatch> matches;
        Mat prev_img;
        Mat img_out;
};
}

