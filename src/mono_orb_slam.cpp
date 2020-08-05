#include "mono_orb_slam.h"

string CAMINTRIN = "../intrinsic.txt";

namespace morb
{

MorbCV::MorbCV():camintrinsic_filename(CAMINTRIN)
{
    namedWindow("img", WINDOW_NORMAL); 
    readCameraIntrinsic();
    cout << "intrinsic: " << camera_matrix << endl;
    R_world = Mat::eye(3, 3, CV_64FC1);
    t_world = Mat::zeros(3, 1, CV_64FC1); 
    this_R = R_world.clone();
    this_T = t_world.clone();
    R_all.push_back(this_R);
    t_all.push_back(this_T);
}

void MorbCV::operator() (Mat img)
{
    resize(img, img, Size(), IMAGE_SCALEFACTOR, IMAGE_SCALEFACTOR);
    img_out = img.clone();
    bool eq = false;

    if (!prev_img.empty()){
        cvtColor(img_out, current_gray, COLOR_BGR2GRAY);
        cvtColor(prev_img, prev_gray, COLOR_BGR2GRAY);
        eq = countNonZero(current_gray != prev_gray) == 0;
    }

    if (prev_img.empty() || eq){
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
    Mat img1_gray, img2_gray;
    vector<KeyPoint> kp1, kp2;
    vector<Point2f> kp1_pnts, kp2_pnts;
    vector<Point2f> kp_prev, kp_current;
    vector<Point2f> corners_1, corners_2;
    vector<Point3f> pnts_colors;
    Mat descriptor1, descriptor2, mask;

    cvtColor(img1, img1_gray, COLOR_BGR2GRAY);
    goodFeaturesToTrack(img1_gray, corners_1, NUM_CORNERS_TO_DETECT, 0.01, 7.0);
    KeyPoint::convert(corners_1, kp1);
    orb_cv->compute(img1, kp1, descriptor1);

    cvtColor(img2, img2_gray, COLOR_BGR2GRAY);
    goodFeaturesToTrack(img2_gray, corners_2, NUM_CORNERS_TO_DETECT, 0.01, 7.0);
    KeyPoint::convert(corners_2, kp2);
    orb_cv->compute(img2, kp2, descriptor2);

    KeyPoint::convert(kp1, kp1_pnts, std::vector<int>());
    KeyPoint::convert(kp2, kp2_pnts, std::vector<int>());

    vector<vector<DMatch>> matches;
    matcher.knnMatch(descriptor1, descriptor2, matches, 2);

    vector<Point2f> p1_vec, p2_vec;
    vector<double> p_dists, pixel_distances;
    Point2f p1_tmp, p2_tmp;
    Point pnt_1, pnt_2;
    double pixel_dist;

    if (matches.size() > 0)
    {
        for (int i=0; i<matches.size(); i++)
        {
            if (matches[i][0].distance < 0.75*matches[i][1].distance)
            {
                if (matches[i][0].distance < KNN_MATCHER_DISTANCE_THRESHOLD)
                {
                    p1_tmp = kp1_pnts[matches[i][0].queryIdx];
                    p2_tmp = kp2_pnts[matches[i][0].trainIdx];

                    double pixel_dist = norm(p1_tmp - p2_tmp);

                    p1_vec.push_back(p1_tmp);
                    p2_vec.push_back(p2_tmp);
                    p_dists.push_back(pixel_dist);
                    pixel_distances.push_back(pixel_dist);
                }
            }
        }
    }

    if (p_dists.size() > 0)
    {
        sort(p_dists.begin(), p_dists.end());
        int threshold_ind = (int) (PIXEL_DISTANCE_THRESHOLD_ORDER * p_dists.size());
        pixel_distance_threshold = p_dists[threshold_ind];
        for (int i=0; i<pixel_distances.size(); i++)
        {
            if (pixel_distances[i] < pixel_distance_threshold)
            {
                p1_tmp = p1_vec[i];
                p2_tmp = p2_vec[i];
                Point3f color_tmp(1.0, 1.0, 1.0);
                kp_prev.push_back(p1_tmp);
                kp_current.push_back(p2_tmp);
                pnts_colors.push_back(color_tmp);

                pnt_1 = Point((int)p1_tmp.x, (int)p1_tmp.y);
                pnt_2 = Point((int)p2_tmp.x, (int)p2_tmp.y);
                circle(img_out, pnt_1, 3, Scalar(255, 0, 0), -1);
                circle(img_out, pnt_2, 3, Scalar(0, 255, 0), -1);
                line(img_out, pnt_1, pnt_2, Scalar(255, 0, 0));
            }
        }
    }

    cout << "number of filtered points: " << kp_prev.size() << endl;

    if (kp_prev.size() > 10)
    {
        E = findEssentialMat(kp_prev, kp_current, camera_matrix, 
                                cv::RANSAC, 0.999, 1.0, mask);
        recoverPose(E, kp_prev, kp_current, camera_matrix, R, t, mask);
        t= -t;

        RMatToMaxAngles(R);
        TToMaxDistance(t);

        // if (max_angle <= MAX_ANGLE && max_translate <= MAX_TRANSLATE)
        if (max_angle <= MAX_ANGLE)
        {
            getPointCloud(kp_prev, kp_current, pnts_colors);
            getPosition();
            prev_img = img2.clone();
        }
    } 
}

void MorbCV::getPointCloud(vector<Point2f> kp1, vector<Point2f> kp2, vector<Point3f> colors)
{
    Mat P1, P2, transformed_pnts;
    Mat pnts3D(1, kp1.size(), CV_64FC4);

    hconcat(R, t, P2);
    hconcat(Mat::eye(3,3,CV_64FC1), Mat::ones(3,1,CV_64FC1), P1);
    triangulatePoints(P1, P2, kp1, kp2, pnts3D);

    for (int i=0; i<pnts3D.cols; i++)
    {
        pnts3D.at<double>(0, i) = pnts3D.at<double>(0, i) / pnts3D.at<double>(3, i);
        pnts3D.at<double>(1, i) = pnts3D.at<double>(1, i) / pnts3D.at<double>(3, i);
        pnts3D.at<double>(2, i) = pnts3D.at<double>(2, i) / pnts3D.at<double>(3, i);
    }
    // convert [x, y, z, w] to [x, y, z]
    pnts3D.rowRange(0, 3).convertTo(pnts3D, CV_64FC1);
    // don't know why, but it works.
    flip(pnts3D, pnts3D, 0);
    transformed_pnts = MONO_SCALE * (R_world * pnts3D);
    for (int i=0; i<transformed_pnts.cols; i++)
    {
        Point3f this_point;
        this_point.x = transformed_pnts.at<double>(0, i);
        this_point.y = transformed_pnts.at<double>(1, i);
        this_point.z = transformed_pnts.at<double>(2, i);
        // if (sqrt(pow(this_point.x, 2) + pow(this_point.z, 2)) <= PCL_DISTANCE_UPPER &&
        //     sqrt(pow(this_point.x, 2) + pow(this_point.z, 2)) >= PCL_DISTANCE_LOWER &&
        //     this_point.z > 0 && this_point.y > -3)
        if (this_point.y >= -PCL_Y_CONFIMENT && this_point.y <= PCL_Y_CONFIMENT)
        // if (1)
        {
            this_point.x = this_point.x + t_world.at<double>(0);
            this_point.y = this_point.y + t_world.at<double>(1);
            this_point.z = this_point.z + t_world.at<double>(2);

            pcl_all.push_back(this_point);
            pcl_colors.push_back(colors[i]);
        }
    }
}

void MorbCV::getPosition()
{
    t_world = t_world + MONO_SCALE * (R_world * t);
    R_world = R * R_world;
    this_R = R_world.clone();
    this_T = t_world.clone();
    R_all.push_back(this_R);
    t_all.push_back(this_T);
}

void MorbCV::readCameraIntrinsic()
{
    ifstream f;
    f.open(camintrinsic_filename.c_str());
    char value_str[100];
    vector<float> values;
    camera_matrix = cv::Mat::zeros(cv::Size(3,3), CV_32F);
    
    /* read camera matrix values */
    if (f.is_open()){
        while (!f.eof()){
            f >> value_str;
            values.push_back(std::stod(value_str));
        }
    }

    /* assign camear matrix values */
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            camera_matrix.at<float>(i,j) = values[i*3 + j] * IMAGE_SCALEFACTOR;
        }
    }
}

void MorbCV::RMatToMaxAngles(Mat R)
{    
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; 

    double x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    max_angle = max(abs(z), max(abs(x), abs(y)));
}

void MorbCV::TToMaxDistance(Mat T)
{
    double x, y, z;

    x = T.at<double>(0);
    y = T.at<double>(1);
    z = T.at<double>(2);

    max_translate = max(abs(x), max(abs(y),abs(z)));
}

}
