#include "pangolin_slamdraw.h"

using namespace std;
using namespace cv;

namespace morb
{
			
    void MorbDraw::operator() ()
    {
        VideoCapture video(video_path.c_str());
        Mat current_frame;
        Size size;
        MorbCV morb_slam;
        int count = 0;

        if (!video.isOpened()){
            cerr << "Please input the right video path" << endl;
        }

        pangolin::CreateWindowAndBind("Main",640,480);
        glEnable(GL_DEPTH_TEST);

        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
            pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
        );

        // Create Interactive View in window
        pangolin::Handler3D handler(s_cam);
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
                .SetHandler(&handler);

        while( !pangolin::ShouldQuit() )
        {
            video >> current_frame;

            if (current_frame.empty())
                break;

            if (count % 2 == 0)
            {
                morb_slam(current_frame);
                // Clear screen and activate view to render into
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                // glClearColor(1.0, 1.0, 1.0, 1.0);
                d_cam.Activate(s_cam);

                // set the hormogenours as indentity for matmul
                draw_mat.SetIdentity();

                // Drawing things
                if (morb_slam.pcl_all.size() > 0)
                {
                    drawPoints(morb_slam.pcl_all, morb_slam.pcl_colors);
                    drawPath(morb_slam.R_all, morb_slam.t_all);
                }

                // Swap frames and Process Events
                pangolin::FinishFrame();
            }
            count ++;

        }
    }

    void MorbDraw::drawPoints(vector<Point3f> all_points, vector<Point3f> all_colors)
    {
        glPointSize(2);
        glBegin(GL_POINTS);

        for (int i=0; i<all_points.size(); i++)
        {
            glColor3f(all_colors[i].x, all_colors[i].y, all_colors[i].z);
            glVertex3f(all_points[i].x, all_points[i].y, all_points[i].z);
        }    

        glEnd();
    }

    void MorbDraw::drawPath(vector<Mat> R_vec, vector<Mat> T_vec)
    {
        for (int i=0; i<R_vec.size(); i++)
        {
            R = R_vec[i];
            T = T_vec[i];
            getHormoMat(R, T);

            drawCam();
            if (i == 0){
                prev_T = T.clone();
            } else {
                drawLine(T);
                prev_T = T.clone();
            }
        }
    }

    void MorbDraw::drawCam()
    {
        Mat H_transpose = H.t();
        // Mat H_transpose = H.clone();

        glPushMatrix();
        glMultMatrixf(H_transpose.ptr<GLfloat>(0));

        glLineWidth(2);
        glColor3f(0.0f,1.0f,0.0f);

        // line up every two points
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }

    void MorbDraw::drawLine(Mat T)
    {
        glLineWidth(2);
        glColor4f(0.0f, 1.0f, 0.0f, 0.6f); 

        glBegin(GL_LINES);
        glVertex3f((float)prev_T.at<double>(0), (float)prev_T.at<double>(1), (float)prev_T.at<double>(2));
        glVertex3f((float)T.at<double>(0), (float)T.at<double>(1), (float)T.at<double>(2));
        glEnd();
    }

    void MorbDraw::getHormoMat(Mat R, Mat T)
    {
        H.at<float>(0,0) = (float)R.at<double>(0,0);
        H.at<float>(0,1) = (float)R.at<double>(0,1);
        H.at<float>(0,2) = (float)R.at<double>(0,2);
        H.at<float>(0,3) = (float)T.at<double>(0);

        H.at<float>(1,0) = (float)R.at<double>(1,0);
        H.at<float>(1,1) = (float)R.at<double>(1,1);
        H.at<float>(1,2) = (float)R.at<double>(1,2);
        H.at<float>(1,3) = (float)T.at<double>(1);

        H.at<float>(2,0) = (float)R.at<double>(2,0);
        H.at<float>(2,1) = (float)R.at<double>(2,1);
        H.at<float>(2,2) = (float)R.at<double>(2,2);
        H.at<float>(2,3) = (float)T.at<double>(2);

        H.at<float>(3,0) = (float)0;
        H.at<float>(3,1) = (float)0;
        H.at<float>(3,2) = (float)0;
        H.at<float>(3,3) = (float)1;
    }

    void MorbDraw::transferToPangoMat(Mat R, Mat T)
    {
        if (!R.empty() && !T.empty())
        {
            draw_mat.m[0] = R.at<double>(0,0);
            draw_mat.m[1] = R.at<double>(1,0);
            draw_mat.m[2] = R.at<double>(2,0);
            draw_mat.m[3] = 0.0;

            draw_mat.m[4] = R.at<double>(0,1);
            draw_mat.m[5] = R.at<double>(1,1);
            draw_mat.m[6] = R.at<double>(2,1);
            draw_mat.m[7] = 0.0;

            draw_mat.m[8] = R.at<double>(0,2);
            draw_mat.m[9] = R.at<double>(1,2);
            draw_mat.m[10] = R.at<double>(2,2);
            draw_mat.m[11] = 0.0;

            draw_mat.m[12] = T.at<double>(0);
            draw_mat.m[13] = T.at<double>(1);
            draw_mat.m[14] = T.at<double>(2);
            draw_mat.m[15] = 1.0;
        } else {
            draw_mat.SetIdentity();
        }
    }

}

