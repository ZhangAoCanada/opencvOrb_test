#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include "pangolin_slamdraw.h"

#include "mono_orb_slam.h"

using namespace std;
using namespace cv;


int main()
{
    string video_path = "/mnt/f/test_data/kitti_seq0.avi";
    // string video_path = "/mnt/f/test_data/seq.mov";

    morb::MorbDraw monoDraw(video_path);

    monoDraw();

    return 0;
}
