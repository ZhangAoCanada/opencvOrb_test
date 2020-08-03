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
    morb::MorbDraw monoDraw;

    monoDraw();

    return 0;
}

// int main()
// {
//     string video_path = "/mnt/f/test_data/seq2.mov";
//     VideoCapture video(video_path.c_str());
//     Mat current_frame;
//     Size size;
//     morb::MorbCV morb_slam;
//     int count = 0;

//     if (!video.isOpened()){
//         cerr << "Please input the right video path" << endl;
//     }

//     while(1) {
//         video >> current_frame;

//         if (current_frame.empty())
//             break;

//         resize(current_frame, current_frame, Size(), 0.5, 0.5);


//         if (count % 1 == 0)
//         {
//             cout << "image size: " << current_frame.rows << "  " << current_frame.cols << endl;
//             morb_slam(current_frame);
//         }

//         count ++; 

//     }

//     return 0;
// }
