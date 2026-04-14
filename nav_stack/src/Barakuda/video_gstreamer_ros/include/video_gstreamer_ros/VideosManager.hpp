#ifndef VIDEOMANAGER_H_
#define VIDEOMANAGER_H_

#include "ros/ros.h"

// Basic
#include <iostream>

// ROS msgs
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>

// IT
#include <image_transport/image_transport.h>
//#include "compressed_image_transport/compressed_publisher.hpp"

// CV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>    // openCV's image processing headers
#include <opencv2/highgui/highgui.hpp>    // GUI modules

//test
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;

struct GStreamerConfig {
  std::string gst_out;
  std::string gst_pipeline_1;
  std::string gst_pipeline_2;
  std::string gst_framerate;
  std::string gst_image_width;
  std::string gst_image_height;
  std::string gst_pcc_host;
  std::string gst_pcc_port;
  std::string gst_enable_topic;
};

// a small class all the video need to inherit
class VideoConverter {
 public:
  VideoConverter() = default;
};

class VideosManager {
 private:
  XmlRpc::XmlRpcValue m_streams;
  std::vector<std::unique_ptr<VideoConverter>> m_videos;

 public:
  VideosManager(ros::NodeHandle &n);
};

#endif  // VIDEOMANAGER_H_
