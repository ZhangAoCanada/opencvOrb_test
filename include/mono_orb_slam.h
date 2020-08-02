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

    protected:
        Ptr<ORB> orb_cv = ORB::create();
};
}

