#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/CameraInfo.h"

#include <sstream>

std::string imu_topic_name;
std::string image_topic_name;
std::string stereo_topic_name;
std::string cam_info_topic_name;
ros::Publisher pub_imu;
ros::Publisher pub_image;
ros::Publisher pub_stereo;
ros::Publisher pub_cam_info;
ros::Time now;

void callback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu new_msg = *msg;
    new_msg.header.stamp = ros::Time::now();

    pub_imu.publish(new_msg);
}

void callback_image(const sensor_msgs::Image::ConstPtr& msg)
{
    sensor_msgs::Image new_msg = *msg;
    new_msg.header.stamp = ros::Time::now();

    pub_image.publish(new_msg);
}

void callback_stereo(const sensor_msgs::Image::ConstPtr& msg)
{
    sensor_msgs::Image new_msg = *msg;
    new_msg.header.stamp = ros::Time::now();

    pub_stereo.publish(new_msg);
}

void callback_cam_info(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    sensor_msgs::CameraInfo new_msg = *msg;
    new_msg.header.stamp = ros::Time::now();

    pub_cam_info.publish(new_msg);
}

// void sync_callback(const sensor_msgs::CameraInfo::ConstPtr& msg)
// {
//     sensor_msgs::CameraInfo new_msg = *msg;
//     new_msg.header.stamp = ros::Time::now();

//     pub_cam_info.publish(new_msg);
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbag_sync");
    ros::NodeHandle n;

    imu_topic_name = "/zed2i/zed_node/imu/data";  // /imu/data
    image_topic_name = "/zed2i/zed_node/rgb/image_rect_color";
    stereo_topic_name = "/zed2i/zed_node/depth/depth_registered";
    cam_info_topic_name = "/zed2i/zed_node/rgb/camera_info";

    pub_imu = n.advertise<sensor_msgs::Imu>(imu_topic_name + "_new", 1000);
    pub_image = n.advertise<sensor_msgs::Image>(image_topic_name + "_new", 1000);
    pub_stereo = n.advertise<sensor_msgs::Image>(stereo_topic_name + "_new", 1000);
    pub_cam_info = n.advertise<sensor_msgs::CameraInfo>(cam_info_topic_name + "_new", 1000);

    ros::Subscriber sub_imu = n.subscribe(imu_topic_name, 2000, callback_imu);
    ros::Subscriber sub_image = n.subscribe(image_topic_name, 2000, callback_image);
    ros::Subscriber sub_stereo = n.subscribe(stereo_topic_name, 2000, callback_stereo);
    ros::Subscriber sub_cam_info = n.subscribe(cam_info_topic_name, 2000, callback_cam_info);



    ros::spin();

    return 0;
}