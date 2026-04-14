#include <video_gstreamer_ros/VideosManager.hpp>
#include <video_gstreamer_ros/ROS2RTP.hpp>
#include <video_gstreamer_ros/RTSP2RTP.hpp>
#include <video_gstreamer_ros/RTSP2ROS.hpp>

VideosManager::VideosManager(ros::NodeHandle &nh) {
  ros::NodeHandle n("~");
  n.getParam("streams", m_streams);
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = m_streams.begin(); it != m_streams.end(); ++it) {
    const std::string name = ((std::string)(it->first));
    std::string type;
    n.param<std::string>("streams/" + name + "/type", type, "NONE");
    ROS_INFO("Initiating %s %s", type.c_str(), name.c_str());
    if (type == "NONE") {
      ROS_WARN("You must specified a type. ROS2RTP, RTSP2RTP or RTSP2ROS.");
    } else if (type == "ROS2RTP") {
      std::unique_ptr<ROS2RTP> my_stream = std::make_unique<ROS2RTP>(nh, name);
      m_videos.push_back(std::move(my_stream));
    } else if (type == "RTSP2RTP") {
      std::unique_ptr<RTSP2RTP> my_stream = std::make_unique<RTSP2RTP>(nh, name);
      m_videos.push_back(std::move(my_stream));
    } else if (type == "RTSP2ROS") {
      std::unique_ptr<RTSP2ROS> my_stream = std::make_unique<RTSP2ROS>(nh, name);
      m_videos.push_back(std::move(my_stream));
    } else {
      ROS_WARN("Unknown type %s", type.c_str());
    }
    ROS_INFO("Finish initiating %s", name.c_str());
  }
}