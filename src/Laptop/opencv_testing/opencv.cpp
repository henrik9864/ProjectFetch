#include <iostream>
#include <vector>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>

using namespace sensor_msgs;
using namespace cv;
using namespace cv::ximgproc;

ros::Publisher disp_pub;

struct Server {
    bool has_left = false;
    bool has_right = false;

    Mat left;
    Mat right;
    int counter = 0;

    Server() {}

    void callback_left(Image left) {
        cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(left);
        this->left = cv_image_ptr->image;
        has_left = true;
        compute();
    }

    void callback_right(Image right) {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(right);
        this->right = cv_ptr->image;
        has_right = true;
        compute();
    }

    void compute() {
        if (!has_left || !has_right)
            return;

        Mat left_for_matcher, right_for_matcher;

        // Downsacle if preformance
        //resize(left ,left_for_matcher ,Size(),0.5,0.5, INTER_LINEAR);
        //resize(right,right_for_matcher,Size(),0.5,0.5, INTER_LINEAR);

        left_for_matcher  = left.clone();
        right_for_matcher = right.clone();

        Ptr<DisparityWLSFilter> wls_filter;
        Mat left_disp, right_disp;
        Mat filtered_disp;

        // BM
        Ptr<StereoBM> left_matcher = StereoBM::create(160,15);
        //left_matcher->setSpeckleWindowSize(9);
        left_matcher->setNumDisparities(112);
        left_matcher->setPreFilterSize(5);
        left_matcher->setPreFilterCap(61);
        left_matcher->setMinDisparity(0);
        left_matcher->setTextureThreshold(507);
        left_matcher->setUniquenessRatio(0);
        left_matcher->setSpeckleWindowSize(0);
        left_matcher->setSpeckleRange(8);
        left_matcher->setDisp12MaxDiff(1);

        wls_filter = createDisparityWLSFilter(left_matcher);
        Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

        cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
        cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);
        
        left_matcher->compute(left_for_matcher, right_for_matcher,left_disp);
        right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);

        // SGBM
        /*
        Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,160,3, 0, 0, 0, 5, 15, 100, 4, StereoSGBM::MODE_SGBM_3WAY);
        left_matcher->setP1(24*7*7);
        left_matcher->setP2(96*7*7);
        left_matcher->setPreFilterCap(63);
        left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
        wls_filter = createDisparityWLSFilter(left_matcher);
        Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

        left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
        right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
        */

        // Filter disparity map
        wls_filter->setLambda(8000);
        wls_filter->setSigmaColor(1.5);
        wls_filter->filter(left_disp,left,filtered_disp,right_disp);

        Mat raw_disp_vis;
        //getDisparityVis(left_disp,raw_disp_vis,1);
        //namedWindow("raw disparity", WINDOW_AUTOSIZE);
        //imshow("raw disparity", raw_disp_vis);

        Mat filtered_disp_vis;
        getDisparityVis(filtered_disp,filtered_disp_vis,1);
        //namedWindow("filtered disparity", WINDOW_AUTOSIZE);
        //imshow("filtered disparity", filtered_disp_vis);

        //Mat disp8;
        //normalize(left_disp, disp8, 0, 255, CV_MINMAX, CV_8U);

        //imshow("OPENCV_WINDOW", disp8);
        //waitKey(3);
        
        PublishDisparity(filtered_disp_vis);

        has_left = false;
        has_right = false;
    }

    void PublishDisparity(Mat &cv_image){
        stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();

        // you need to know only this params for a basic usage (0-64, st.img.proc default param here)
        disp_msg->min_disparity = 0;
        disp_msg->max_disparity = 63;

        // should be safe
        disp_msg->valid_window.x_offset = 0;
        disp_msg->valid_window.y_offset = 0;
        disp_msg->valid_window.width    = 0;
        disp_msg->valid_window.height   = 0;
        disp_msg->T                     = 0;
        disp_msg->f                     = 0;
        disp_msg->delta_d               = 0;
        disp_msg->header.stamp          = ros::Time::now();
        disp_msg->header.frame_id       = ros::this_node::getName(); 
        disp_msg->header.seq            = this->counter; //a counter, int type

        sensor_msgs::Image& dimage = disp_msg->image;
        dimage.width  = cv_image.size().width;
        dimage.height = cv_image.size().height;
        dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        dimage.step = dimage.width * sizeof(float);
        dimage.data.resize(dimage.step * dimage.height);
        cv::Mat_<float> dmat(dimage.height, dimage.width, reinterpret_cast<float*>(&dimage.data[0]), dimage.step);

        cv_image.convertTo(dmat,dmat.type());
        disp_pub.publish(disp_msg);

        counter++;
    }

} server;

void callback_left(Image image_left) {
    server.callback_left(image_left);
}

void callback_right(Image image_right) {
    server.callback_right(image_right);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "opencv");
    ros::NodeHandle nh;
    std::string nameSpace = ros::this_node::getNamespace();

    ros::Subscriber sub_image_left = nh.subscribe("/stereo/left/image_rect_color", 1, callback_left);
    ros::Subscriber sub_image_right = nh.subscribe("/stereo/right/image_rect_color", 1, callback_right);

    disp_pub = nh.advertise<stereo_msgs::DisparityImage>("/stereo/custom/disparity", 1);

    cv_bridge::CvImage test;

    ros::spin();
    return 0;
}