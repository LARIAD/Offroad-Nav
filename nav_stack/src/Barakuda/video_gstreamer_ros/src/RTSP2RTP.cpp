#include <video_gstreamer_ros/RTSP2RTP.hpp>

static gboolean staticOnBusMessage(GstBus *bus, GstMessage *msg, gpointer data) {
  return (static_cast<RTSP2RTP *>(data))->onBusMessage(bus, msg);
}

RTSP2RTP::RTSP2RTP(ros::NodeHandle &n, std::string stream) : VideoConverter(), name(stream) {
  ros::NodeHandle private_n("~");
  // Params
  // Topics
  std::string gst_enable_topic;
  private_n.param<std::string>("streams/" + stream + "/enable_stream_topic", gst_enable_topic,
                               stream + "gst_enable");
  // Gst pipeline
  gst_out = "rtspsrc location=";
  std::string p;
  private_n.param<std::string>("streams/" + stream + "/gst_rtsp_url", p, "rtsp://192.168.0.90:554/cam");
  gst_out += p;
  private_n.param<std::string>("streams/" + stream + "/gst_pipeline_1", p,
                               " latency=0 ! decodebin max-size-time=30000000000 ! nvvidconv ! nvjpegenc ! imgjpeg,");
  gst_out += p + "width=";
  private_n.param<std::string>("streams/" + stream + "/out_width", p, "400");
  gst_out += p + ",height=";
  private_n.param<std::string>("streams/" + stream + "/out_height", p, "200");
  gst_out += p;
  private_n.param<std::string>("streams/" + stream + "/gst_pipeline_2", p,
                               " ! rtpjpegpay ! ");
  gst_out += p + " udpsink host=";
  private_n.param<std::string>("streams/" + stream + "/pcc_host", p, "127.0.0.1");
  gst_out += p + " port=";
  private_n.param<std::string>("streams/" + stream + "/pcc_port", p, "5000");
  gst_out += p;
  // Param
  private_n.param<bool>("streams/" + stream + "/debug", m_debug, true);
  private_n.param<bool>("streams/" + stream + "/low_resources", m_low_resources, true);

  // Init Enabling/Disabling topic
  m_gst_enable_sub = n.subscribe(gst_enable_topic, 1, &RTSP2RTP::gstEnableCallback, this);

  // Init Gstreamer pipeline
  ROS_INFO("[%s | %s] Pipeline is %s", this->name.c_str(), "RTSP2RTP", gst_out.c_str());
  gst_init(nullptr, nullptr);
  GError *error = nullptr;
  pipeline = gst_parse_launch(gst_out.c_str(), &error);
  if (!pipeline) {
    ROS_WARN("[%s | %s] Pipeline creation failed: %s", this->name.c_str(), "RTSP2RTP", error->message);
    g_clear_error(&error);
    return;
  }

  gst_element_set_state(pipeline, GST_STATE_PAUSED);
  bus = gst_element_get_bus(pipeline);
  gst_bus_add_watch(bus, &staticOnBusMessage, this);
}

gboolean RTSP2RTP::onBusMessage(GstBus *bus, GstMessage *msg) {
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_EOS:
      ROS_INFO("[%s | %s] EOS received", this->name.c_str(), "RTSP2RTP");
      break;
    case GST_MESSAGE_ERROR:
      ROS_ERROR("[%s | %s] Error received", this->name.c_str(), "RTSP2RTP");
      break;
  }
  return TRUE;
}

void RTSP2RTP::gstEnableCallback(const std_msgs::Bool::ConstPtr &msg) {
  const bool is_enable = msg->data;
  ROS_INFO("[%s | %s] Now %s", name.c_str(), "RTSP2RTP", is_enable ? "enable" : "disable");
  if (is_enable) {
    if (!pipeline) {
      GError *error = nullptr;
      pipeline = gst_parse_launch(gst_out.c_str(), &error);
      if (!pipeline) {
        ROS_ERROR("[%s | %s] Failed to recreate pipeline: %s", name.c_str(), "RTSP2RTP", error->message);
        g_clear_error(&error);
        return;
      }
      bus = gst_element_get_bus(pipeline);
      gst_bus_add_watch(bus, &staticOnBusMessage, this);
    } else {
      ROS_WARN("[%s | %s] Pipeline already exists, skipping recreation", name.c_str(), "RTSP2RTP");
    }
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
  } else {
    if (!pipeline)
      return;
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    gst_object_unref(bus);
    pipeline = nullptr;
    bus = nullptr;
  }
}

RTSP2RTP::~RTSP2RTP() {
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
  gst_object_unref(bus);
}
