#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include "pangolin_slamdraw.h"

#include "mono_orb_slam.h"

using namespace std;
using namespace cv;


int main(int argc, char ** argv)
{
    if (argc != 3)
    {
        cerr << "Usage: ./morb path_to_video turn_on_camera_following" << endl;
    }

    string video_path = argv[1];
    stringstream ss(argv[2]);
    bool follow_camera;

    ss >> boolalpha >> follow_camera;
    
    //string video_path = "/Users/aozhang/Downloads/kitti_seq0.avi";
    // string video_path = "/mnt/f/test_data/seq.mov";
    //bool follow_camera = false;

    morb::MorbDraw monoDraw(video_path, follow_camera);

    monoDraw();

    return 0;
}
