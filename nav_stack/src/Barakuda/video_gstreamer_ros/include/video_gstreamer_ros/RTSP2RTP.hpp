#ifndef RTSP2RTP_H_
#define RTSP2RTP_H_

#include <video_gstreamer_ros/VideosManager.hpp>
#include <gst/gst.h>

class RTSP2RTP : public VideoConverter {
 private:
  // Debug
  bool m_debug;
  bool m_low_resources;
  std::string gst_out;

  // stream
  GstElement *pipeline;
  GstBus *bus;

  // Enabling/Disabling the stream
  ros::Subscriber m_gst_enable_sub;
 public:
  const std::string name;

  RTSP2RTP(ros::NodeHandle &n, std::string stream);
  ~RTSP2RTP();

  // Callbacks
  gboolean onBusMessage(GstBus *bus, GstMessage *msg);
  void gstEnableCallback(const std_msgs::Bool::ConstPtr &msg);
};

#endif  // RTSP2RTP_H_
