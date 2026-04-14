#include <video_gstreamer_ros/RTSP2ROS.hpp>

RTSP2ROS::RTSP2ROS(ros::NodeHandle &n, std::string stream) :
    VideoConverter(), name(stream) {
  ros::NodeHandle private_n("~");
  // Params
  // Camera
  private_n.param<std::string>("streams/" + stream + "/camera_topic", m_camera_topic, "image_raw");
  private_n.param<std::string>("streams/" + stream + "/camera_info_topic", m_camera_info_topic, "image_raw/info");
  std::string frame_id;
  private_n.param<std::string>("streams/" + stream + "/frame_id", frame_id, "gst_camera_frame");
  int framerate;
  private_n.param<int>("streams/" + stream + "/framerate", framerate, 30);
  // Gst pipeline
  gst_out = "rtspsrc location=";
  std::string p;
  private_n.param<std::string>("streams/" + stream + "/gst_rtsp_url", p, "rtsp://192.168.0.90:554/cam");
  gst_out += p;
  private_n.param<std::string>("streams/" + stream + "/gst_pipeline_1", p,
                               " latency=0 ! decodebin max-size-time=30000000000 ! nvvidconv ! video/x-raw,");
  gst_out += p + "width=";
  private_n.param<std::string>("streams/" + stream + "/in_width", p, "1920");
  gst_out += p + ",height=";
  private_n.param<std::string>("streams/" + stream + "/in_height", p, "1080");
  gst_out += p;
  private_n.param<std::string>("streams/" + stream + "/gst_pipeline_2",
                               p,
                               ",format=BGRx ! videoconvert ! video/x-raw,format=BGR ! videoconvert ! videoscale ! video/x-raw,");
  std::string out_width;
  private_n.param<std::string>("streams/" + stream + "/out_width", out_width, "400");
  std::string out_height;
  private_n.param<std::string>("streams/" + stream + "/out_height", out_height, "200");
  gst_out += p + "width=" + out_width + ",height=" + out_height + " ! appsink drop=1";
  // Param
  private_n.param<bool>("streams/" + stream + "/debug", m_debug, true);
  private_n.param<bool>("streams/" + stream + "/low_resources", m_low_resources, true);

  // Camera topic
  m_pub_camera = n.advertise<sensor_msgs::CompressedImage>(m_camera_topic, 1);
  m_pub_camera_info = n.advertise<sensor_msgs::CameraInfo>(m_camera_info_topic, 1000);
  m_msg_cam_info.header.frame_id = frame_id;
  m_msg_cam_info.width = std::stoi(out_width);
  m_msg_cam_info.height = std::stoi(out_height);

  // Init Gstreamer pipeline
  ROS_INFO("[%s | %s] Pipeline is %s", name.c_str(), "RTSP2ROS", gst_out.c_str());
  m_gst_reader = std::make_unique<VideoCapture>(gst_out.c_str(), cv::CAP_GSTREAMER);
  ROS_DEBUG("[%s | %s] VideoCapture properly initiated", name.c_str(), "RTSP2ROS");

  // Publication loop
  m_timer = n.createTimer(ros::Duration(1.0 / framerate), &RTSP2ROS::timerCallback, this);
}

void RTSP2ROS::timerCallback(const ros::TimerEvent &) {
  if (!m_gst_reader->isOpened()) {
    ROS_FATAL("[%s | %s] Reader is not opened!!", name.c_str(), "RTSP2ROS");
    m_gst_reader = std::make_unique<VideoCapture>(gst_out.c_str(), cv::CAP_GSTREAMER);
    return;
  }
  ROS_DEBUG("[%s | %s] Publishing an image", name.c_str(), "RTSP2ROS");
  // get image and create pointer
  *m_gst_reader >> image;
  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  if (cv_ptr->image.empty()) return;
  // convert to compressed image
  std::vector<uchar> compressed_data;
  cv::imencode(".jpg", cv_ptr->image, compressed_data,
               {cv::IMWRITE_JPEG_QUALITY, 90}); // Qualité JPEG entre 0 et 100

  // create a compressed msg and publish it
  sensor_msgs::CompressedImage compressed_msg;
  compressed_msg.header = m_msg_cam_info.header;
  compressed_msg.format = "jpeg";
  compressed_msg.data = std::move(compressed_data);
  m_pub_camera.publish(compressed_msg);
  // publish camera info
  m_msg_cam_info.header.stamp = ros::Time::now();
  m_pub_camera_info.publish(m_msg_cam_info);
}
