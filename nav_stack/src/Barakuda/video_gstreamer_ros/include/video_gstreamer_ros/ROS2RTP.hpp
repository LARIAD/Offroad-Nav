#ifndef ROS2RTP_H_
#define ROS2RTP_H_

#include <video_gstreamer_ros/VideosManager.hpp>

class ROS2RTP : public VideoConverter {
 private:
  std::string name;

  // ROS Topics
  std::string m_camera_topic;

  sensor_msgs::ImageConstPtr m_camera;
  sensor_msgs::CameraInfo m_camera_info;

  // Video meta
  int m_height = 0;
  int m_width = 0;

  // CV
  cv_bridge::CvImagePtr cv_ptr;
  Mat m_intrinsic_params;
  Mat m_distortion_matrix;

  // Debug
  bool m_debug;
  bool m_low_resources;
  bool m_get_image = false;

  // Subscribers
  image_transport::Subscriber m_cameraSubscriber;

  // Timer for gstramer
  ros::Timer m_timer;

  // Gstreamer gestion
  GStreamerConfig m_stream;
  VideoWriter m_gst_writer;

  // Enabling/Disabling the stream
  bool m_is_enable = false;
  ros::Subscriber m_gst_enable_sub;
 public:
  ROS2RTP(ros::NodeHandle &n, std::string stream);

  // Callbacks
  void cameraCallback(const sensor_msgs::ImageConstPtr &_img);
  void gstEnableCallback(const std_msgs::Bool::ConstPtr &msg);

  void publishGstreamer(const ros::TimerEvent &e);
};

#endif  // ROS2RTP_H_
