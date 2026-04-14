#ifndef RTSP2ROS_H_
#define RTSP2ROS_H_

#include <video_gstreamer_ros/VideosManager.hpp>
#include <gst/gst.h>

class RTSP2ROS : public VideoConverter {
 private:
  // Camera
  std::string m_camera_topic;
  std::string m_camera_info_topic;
  Mat image;
  ros::Publisher m_pub_camera;
  ros::Publisher m_pub_camera_info;
  sensor_msgs::CameraInfo m_msg_cam_info;

  // Gst
  std::unique_ptr<VideoCapture> m_gst_reader;
  std::string gst_out;
  ros::Timer m_timer;

  // Debug
  bool m_debug;
  bool m_low_resources;
 public:
  const std::string name;

  RTSP2ROS(ros::NodeHandle &n, std::string stream);
  void timerCallback(const ros::TimerEvent&);
};

#endif  // RTSP2ROS_H_
