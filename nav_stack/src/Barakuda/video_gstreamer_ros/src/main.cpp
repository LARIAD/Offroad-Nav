#include "video_gstreamer_ros/VideosManager.hpp"

// TODO change gst pipeline arg and argparsing
//      to unique string with placeholder that the code finds with regex and then get them

int main(int argc, char **argv) {
  ros::init(argc, argv, "video_gstreamer");
  ros::NodeHandle n;
  VideosManager manager(n);
  ros::spin();
  return 0;
}
