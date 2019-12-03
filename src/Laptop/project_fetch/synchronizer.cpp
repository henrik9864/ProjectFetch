#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <string.h>
#include <iostream>
#include <project_fetch/CameraImageInfo.h>
#include <ros/header.h>

using namespace project_fetch;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub_compressed_left;
ros::Publisher pub_compressed_right;

ros::Publisher pub_image_left;
ros::Publisher pub_info_left;

ros::Publisher pub_image_right;
ros::Publisher pub_info_right;

void callbackLeft(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
{
  CameraImageInfo imageInfo;
  imageInfo.header.stamp = ros::Time::now();
  imageInfo.header.frame_id = "/project_fetch";

  imageInfo.image = *image;
  imageInfo.camInfo = *cam_info;

  imageInfo.image.header.stamp = ros::Time::now();
  imageInfo.camInfo.header.stamp = ros::Time::now();

  pub_compressed_left.publish(imageInfo);
  //pub_image_left.publish(image);
  //pub_info_left.publish(cam_info);
}

void callbackRight(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
{
  CameraImageInfo imageInfo;
  imageInfo.header.stamp = ros::Time::now();
  imageInfo.header.frame_id = "/project_fetch";

  imageInfo.image = *image;
  imageInfo.camInfo = *cam_info;

  imageInfo.image.header.stamp = ros::Time::now();
  imageInfo.camInfo.header.stamp = ros::Time::now();

  pub_compressed_right.publish(imageInfo);
  //pub_image_right.publish(image);
  //pub_info_right.publish(cam_info);
}

void callbackImages(const CameraImageInfoConstPtr& left_image, const CameraImageInfoConstPtr& right_image) {
  pub_image_left.publish(left_image->image);
  pub_info_left.publish(left_image->camInfo);

  pub_image_right.publish(right_image->image);
  pub_info_right.publish(right_image->camInfo);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "synchronizer");
  ros::NodeHandle nh;
  std::string nameSpace = ros::this_node::getNamespace();

  pub_compressed_left = nh.advertise<CameraImageInfo>(nameSpace + "/left/async/compressed", 30);
  pub_compressed_right = nh.advertise<CameraImageInfo>(nameSpace + "/right/async/compressed", 30);

  pub_image_left = nh.advertise<Image>(nameSpace + "/left/image_raw", 30);
  pub_info_left = nh.advertise<CameraInfo>(nameSpace + "/left/camera_info", 30);

  pub_image_right = nh.advertise<Image>(nameSpace + "/right/image_raw", 30);
  pub_info_right = nh.advertise<CameraInfo>(nameSpace + "/right/camera_info", 30);

  //Synchronizes left image and camera_info, sends out compressed data
  message_filters::Subscriber<Image> image_sub_left(nh, nameSpace + "/left/async/image_raw", 1);
  message_filters::Subscriber<CameraInfo> info_sub_left(nh, nameSpace + "/left/async/camera_info", 1);
  TimeSynchronizer<Image, CameraInfo> sync_left(image_sub_left, info_sub_left, 10);
  sync_left.registerCallback(boost::bind(&callbackLeft, _1, _2));

  //Synchronizes right image and camera_info, sends out compressed data
  message_filters::Subscriber<Image> image_sub_right(nh, nameSpace + "/right/async/image_raw", 1);
  message_filters::Subscriber<CameraInfo> info_sub_right(nh, nameSpace + "/right/async/camera_info", 1);
  TimeSynchronizer<Image, CameraInfo> sync_right(image_sub_right, info_sub_right, 10);
  sync_right.registerCallback(boost::bind(&callbackRight, _1, _2));

  //Synchronizes left and right camera images and spits them out separetely again
  typedef message_filters::sync_policies::ApproximateTime<CameraImageInfo, CameraImageInfo> MySyncPolicy;

  message_filters::Subscriber<CameraImageInfo> sub_compressed_left(nh, nameSpace + "/left/async/compressed", 1);
  message_filters::Subscriber<CameraImageInfo> sub_compressed_right(nh, nameSpace + "/right/async/compressed", 1);
  //TimeSynchronizer<CameraImageInfo, CameraImageInfo> sync_images(sub_compressed_left, sub_compressed_right, 10, 0.1);
  message_filters::Synchronizer<MySyncPolicy> sync_images(MySyncPolicy(10), sub_compressed_left, sub_compressed_right);
  sync_images.registerCallback(boost::bind(&callbackImages, _1, _2));

  ros::spin();

  return 0;
}
