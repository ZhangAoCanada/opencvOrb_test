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

                    pixel_dist = norm(p1_tmp - p2_tmp);

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
        //E = findEssentialMat(kp_current, kp_prev, camera_matrix, 
                                //cv::RANSAC, 0.999, 1.0, mask);
        //recoverPose(E, kp_current, kp_prev, camera_matrix, R, t, mask);

        RMatToMaxAngles(R);
        TToMaxDistance(t);

        // if (max_angle <= MAX_ANGLE && max_translate <= MAX_TRANSLATE)
        if (max_angle <= MAX_ANGLE)
        {
            getPointCloud(kp_prev, kp_current, pnts_colors);
            //t= -t;
            getPosition();
            prev_img = img2.clone();
        }
    } 
}

vector<Point2f> MorbCV::normalizePnts(vector<Point2f> key_points)
{
    vector<Point2f> normalized_pnts;
    for(int i=0; i<key_points.size(); i++)
    {
        Mat pnt_addones = Mat::zeros(3, 1, CV_32F);
        pnt_addones.at<float>(0) = key_points[i].x;
        pnt_addones.at<float>(1) = key_points[i].y;
        pnt_addones.at<float>(2) = (float) 1;
        Mat normalize_one;
        normalize_one = camera_matrix.inv() * pnt_addones;
        Point2f new_point;
        new_point.x = normalize_one.at<float>(0);
        new_point.y = normalize_one.at<float>(1);
        //cout << new_point << endl;
        normalized_pnts.push_back(new_point);
    }

    return normalized_pnts;
}

vector<Point3f> MorbCV::triangulation(Mat P1, Mat P2, vector<Point2f> kp_norm1, 
                vector<Point2f> kp_norm2)
{
    vector<Point3f> pnts3d;
    Mat w, u, vt;
    Mat point_original = Mat::zeros(4, 1, CV_64FC1);
    Mat A = Mat::zeros(4, 4, CV_64FC1);
    for (int i=0; i<kp_norm1.size(); i++)
    {
        A.at<double>(0,0) = kp_norm1[i].x * P1.at<double>(2,0) - P1.at<double>(0,0);
        A.at<double>(0,1) = kp_norm1[i].x * P1.at<double>(2,1) - P1.at<double>(0,1);
        A.at<double>(0,2) = kp_norm1[i].x * P1.at<double>(2,2) - P1.at<double>(0,2);
        A.at<double>(0,3) = kp_norm1[i].x * P1.at<double>(2,3) - P1.at<double>(0,3);

        A.at<double>(1,0) = kp_norm1[i].y * P1.at<double>(2,0) - P1.at<double>(1,0);
        A.at<double>(1,1) = kp_norm1[i].y * P1.at<double>(2,1) - P1.at<double>(1,1);
        A.at<double>(1,2) = kp_norm1[i].y * P1.at<double>(2,2) - P1.at<double>(1,2);
        A.at<double>(1,3) = kp_norm1[i].y * P1.at<double>(2,3) - P1.at<double>(1,3);

        A.at<double>(2,0) = kp_norm2[i].x * P2.at<double>(2,0) - P2.at<double>(0,0);
        A.at<double>(2,1) = kp_norm2[i].x * P2.at<double>(2,1) - P2.at<double>(0,1);
        A.at<double>(2,2) = kp_norm2[i].x * P2.at<double>(2,2) - P2.at<double>(0,2);
        A.at<double>(2,3) = kp_norm2[i].x * P2.at<double>(2,3) - P2.at<double>(0,3);

        A.at<double>(3,0) = kp_norm2[i].y * P2.at<double>(2,0) - P2.at<double>(1,0);
        A.at<double>(3,1) = kp_norm2[i].y * P2.at<double>(2,1) - P2.at<double>(1,1);
        A.at<double>(3,2) = kp_norm2[i].y * P2.at<double>(2,2) - P2.at<double>(1,2);
        A.at<double>(3,3) = kp_norm2[i].y * P2.at<double>(2,3) - P2.at<double>(1,3);

        SVD::compute(A, w, u, vt);
        point_original.at<double>(0) = vt.at<double>(3, 0);
        point_original.at<double>(1) = vt.at<double>(3, 1);
        point_original.at<double>(2) = vt.at<double>(3, 2);
        point_original.at<double>(3) = vt.at<double>(3, 3);
        
        if (point_original.at<double>(3) != 0)
        {
            Point3f pnt;
            pnt.x = point_original.at<double>(0) / point_original.at<double>(3);
            pnt.y = point_original.at<double>(1) / point_original.at<double>(3);
            pnt.z = point_original.at<double>(2) / point_original.at<double>(3);
            pnts3d.push_back(pnt);
        }
    }

    return pnts3d;
}

void MorbCV::getPointCloud(vector<Point2f> kp1, vector<Point2f> kp2, vector<Point3f> colors)
{
    vector<Point2f> kp1_norm, kp2_norm;
    vector<Point3f> p3d;
    kp1_norm = normalizePnts(kp1);
    kp2_norm = normalizePnts(kp2);

    Mat P1, P2;
    Mat h_lastrow = Mat::zeros(1, 4, CV_64FC1);
    h_lastrow.at<double>(3) = 1;
    hconcat(R, t, P2);
    vconcat(P2, h_lastrow, P2);
    hconcat(Mat::eye(3,3,CV_64FC1), Mat::zeros(3,1,CV_64FC1), P1);
    vconcat(P1, h_lastrow, P1);

    p3d = triangulation(P1, P2, kp1_norm, kp2_norm);
    //transformed_pnts = MONO_SCALE * (R_world * pnts3D);
    for(int i=0; i<p3d.size(); i++)
    {
        Point3f this_point;
        Mat this_pnt_mat = Mat::zeros(3,1,CV_64FC1);
        this_pnt_mat.at<double>(0) = p3d[i].x;
        this_pnt_mat.at<double>(1) = p3d[i].y;
        this_pnt_mat.at<double>(2) = p3d[i].z;
        this_pnt_mat = MONO_SCALE * (R_world * this_pnt_mat);

        if (this_pnt_mat.at<double>(2) > 0)
        {
            this_point.x = this_pnt_mat.at<double>(0) + t_world.at<double>(0);
            this_point.y = this_pnt_mat.at<double>(1) + t_world.at<double>(1);
            this_point.z = this_pnt_mat.at<double>(2) + t_world.at<double>(2);
            
            cout << this_point << endl;

            pcl_all.push_back(this_point);
            pcl_colors.push_back(colors[i]);
        }
    }

    //Mat P1, P2, transformed_pnts;
    //Mat pnts3D(1, kp1.size(), CV_64FC4);

    //hconcat(R, t, P2);
    //hconcat(Mat::eye(3,3,CV_64FC1), Mat::ones(3,1,CV_64FC1), P1);
    //triangulatePoints(P1, P2, kp1, kp2, pnts3D);

    //for (int i=0; i<pnts3D.cols; i++)
    //{
        //pnts3D.at<double>(0, i) = pnts3D.at<double>(0, i) / pnts3D.at<double>(3, i);
        //pnts3D.at<double>(1, i) = pnts3D.at<double>(1, i) / pnts3D.at<double>(3, i);
        //pnts3D.at<double>(2, i) = pnts3D.at<double>(2, i) / pnts3D.at<double>(3, i);
    //}
    //// convert [x, y, z, w] to [x, y, z]
    //pnts3D.rowRange(0, 3).convertTo(pnts3D, CV_64FC1);
    //// don't know why, but it works.
    //flip(pnts3D, pnts3D, 0);
    //transformed_pnts = MONO_SCALE * (R_world * pnts3D);
    //for (int i=0; i<transformed_pnts.cols; i++)
    //{
        //Point3f this_point;
        //this_point.x = transformed_pnts.at<double>(0, i);
        //this_point.y = transformed_pnts.at<double>(1, i);
        //this_point.z = transformed_pnts.at<double>(2, i);
        //// if (sqrt(pow(this_point.x, 2) + pow(this_point.z, 2)) <= PCL_DISTANCE_UPPER &&
        ////     sqrt(pow(this_point.x, 2) + pow(this_point.z, 2)) >= PCL_DISTANCE_LOWER &&
        ////     this_point.z > 0 && this_point.y > -3)
        //if (this_point.y >= -PCL_Y_CONFIMENT && this_point.y <= PCL_Y_CONFIMENT && 
                //this_point.z > 0)
        //{
            //this_point.x = this_point.x + t_world.at<double>(0);
            //this_point.y = this_point.y + t_world.at<double>(1);
            //this_point.z = this_point.z + t_world.at<double>(2);

            //pcl_all.push_back(this_point);
            //pcl_colors.push_back(colors[i]);
        //}
    /*}*/
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
