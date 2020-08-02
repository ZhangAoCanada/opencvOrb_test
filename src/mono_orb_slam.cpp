#include "mono_orb_slam.h"

namespace morb
{

MorbCV::MorbCV()
{
    namedWindow("img", WINDOW_NORMAL); 
}

void MorbCV::operator() (Mat img)
{
    if (prev_img.empty()){
        prev_img = img.clone();
    } else {
        orbKeyPointMatch(prev_img, img);
        imgPlot(img_out);
        prev_img = img.clone();
    }
}

void MorbCV::imgPlot(Mat img)
{
    imshow("img", img);
    waitKey(1);
}

void MorbCV::orbKeyPointMatch(Mat img1, Mat img2)
{
    orb_cv->detect(img1, kp1);
    orb_cv->compute(img1, kp1, descriptor1);

    orb_cv->detect(img2, kp2);
    orb_cv->compute(img1, kp2, descriptor2);

    matcher.match(descriptor1, descriptor2, matches);
    drawMatches(img1, kp1, img2, kp2, matches, img_out);

    // drawKeypoints(img1, kp, img_out, Scalar(0, 255, 0));
}

}
