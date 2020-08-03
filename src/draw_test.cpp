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
