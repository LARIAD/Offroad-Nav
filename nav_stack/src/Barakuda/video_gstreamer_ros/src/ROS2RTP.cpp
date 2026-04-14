#include <video_gstreamer_ros/ROS2RTP.hpp>

ROS2RTP::ROS2RTP(ros::NodeHandle &n, std::string stream) : VideoConverter() {
  ros::NodeHandle private_n("~");
  name = stream;
  // Params
  // Topics
  private_n.param<std::string>("streams/" + stream + "/camera_topic", m_camera_topic, "/image_rect_color");
  // Gst pipeline
  private_n.param<std::string>("streams/" + stream + "/gst_pipeline_1", m_stream.gst_pipeline_1,
                               "appsrc ! videoconvert ! videoscale ! video/x-raw,");
  private_n.param<std::string>("streams/" + stream + "/gst_pipeline_2", m_stream.gst_pipeline_2,
                               "/1, format=YUY2 ! videoconvert ! jpegenc ! rtpjpegpay !");
  private_n.param<std::string>("streams/" + stream + "/framerate", m_stream.gst_framerate, "30");
  private_n.param<std::string>("streams/" + stream + "/image_width", m_stream.gst_image_width, "640");
  private_n.param<std::string>("streams/" + stream + "/image_height", m_stream.gst_image_height, "360");
  private_n.param<std::string>("streams/" + stream + "/pcc_host", m_stream.gst_pcc_host, "127.0.0.1");
  private_n.param<std::string>("streams/" + stream + "/pcc_port", m_stream.gst_pcc_port, "5000");
  // Param
  private_n.param<bool>("streams/" + stream + "/debug", m_debug, true);
  private_n.param<bool>("streams/" + stream + "/low_resources", m_low_resources, true);
  private_n.param<std::string>("streams/" + stream + "/enable_stream_topic",
                               m_stream.gst_enable_topic, stream + "gst_enable");

  // Init Enabling/Disabling topic
  m_gst_enable_sub = n.subscribe(m_stream.gst_enable_topic, 1, &ROS2RTP::gstEnableCallback, this);

  // Init Gstreamer pipeline
  m_stream.gst_out =
      m_stream.gst_pipeline_1 + "width=" + m_stream.gst_image_width + ", height=" + m_stream.gst_image_height
          + ", framerate=" + m_stream.gst_framerate + m_stream.gst_pipeline_2 + " udpsink host=" + m_stream.gst_pcc_host
          + " port=" + m_stream.gst_pcc_port;
  ROS_INFO("[%s | %s] Pipeline is %s", name.c_str(), "ROS2RTP", m_stream.gst_out.c_str());
  m_gst_writer = VideoWriter();

  // IT Subscriber
  image_transport::ImageTransport it(n);
  // Subscribers
  m_cameraSubscriber = it.subscribe(m_camera_topic, 10, &ROS2RTP::cameraCallback, this);

  // Gstreamer loop
  m_timer = n.createTimer(ros::Duration(1. / std::stod(m_stream.gst_framerate)),
                          &ROS2RTP::publishGstreamer, this);
}

// Callbacks
void ROS2RTP::cameraCallback(const sensor_msgs::ImageConstPtr &_img) {
  if (m_width == 0 || m_height == 0 || !m_gst_writer.isOpened()) {
    m_width = _img->width;
    m_height = _img->height;
    m_gst_writer.open(m_stream.gst_out, 0,
                      std::stod(m_stream.gst_framerate),
                      Size(m_width, m_height), // use the input size of the image, gstream will resize
                      true);
    ROS_INFO("[%s | %s] Gstreamer started with %s", name.c_str(), "ROS2RTP", m_stream.gst_out.c_str());
  }
  ROS_DEBUG("[%s | %s] New image", name.c_str(), "ROS2RTP");

  if (!m_is_enable) return;

  try {
    /* Convert the ROS image message to a CvImage suitable for working with OpenCV
     * Note: if we plan to modify the image, we need a mutable copy of it, so we use toCvCopy()
     * Else, use toCvShare() to share without copy (more efficient)
     */
    cv_ptr = cv_bridge::toCvCopy(_img, sensor_msgs::image_encodings::BGR8);
    m_get_image = true;
  }
    /* catch conversion errors as toCvCopy() / toCvShared() will not check for the validity of the data */
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("[%s | %s] cv_bridge exception: %s", name.c_str(), "ROS2RTP",  e.what());
  }
}

void ROS2RTP::publishGstreamer(const ros::TimerEvent &e) {
  if (!m_is_enable) return;
  if (!m_get_image) return;

  if (!m_gst_writer.isOpened()) {
    ROS_FATAL("[%s | %s] The writer is not open!!", name.c_str(), "ROS2RTP");
    return;
  }

  Mat pub_img;
  if (cv_ptr) {
    pub_img = cv_ptr->image;
    if (pub_img.empty()) {
      ROS_WARN("[%s | %s] The image is empty...", name.c_str(), "ROS2RTP");
      return;
    }
  } else {
    ROS_WARN("[%s | %s] The pointer is not pointing...", name.c_str(), "ROS2RTP");
    return;
  }
  ROS_DEBUG("[%s | %s] Publish new image", name.c_str(), "ROS2RTP");

  m_gst_writer << pub_img;
  m_get_image = false;
}

void ROS2RTP::gstEnableCallback(const std_msgs::Bool::ConstPtr &msg) {
  m_is_enable = msg->data;
  ROS_INFO("[%s | %s] Now %s", name.c_str(), "ROS2RTP", m_is_enable ? "enable" : "disable");
}