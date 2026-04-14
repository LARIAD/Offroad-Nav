#include"ros/ros.h"

// Basic
#include <iostream>

// ROS msgs
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <grid_map_msgs/GridMap.h>
#include <tf/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/Twist.h>

// IT
#include <image_transport/image_transport.h>
//#include "compressed_image_transport/compressed_publisher.hpp"

// CV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>	// openCV's image processing headers
#include <opencv2/highgui/highgui.hpp>	// GUI modules

using namespace cv;

class BarakudaVision
{
    private:

        // ROS Topics
        std::string m_camera_topic;
        std::string m_camera_info_topic;
        std::string m_imu_topic;
        std::string m_cmd_vel_topic;

        // Grid map layers;
        std::string m_gridmap_layer_topic;
        
        // TF
        std::string m_hight_info_frame;
        std::string m_robot_frame;
        std::string m_camera_frame;
        std::string m_optical_frame;

        sensor_msgs::ImageConstPtr m_camera;
        sensor_msgs::CameraInfo m_camera_info;
        sensor_msgs::Imu m_imu;
        geometry_msgs::Twist m_cmd_vel;
        
        // CV
        Mat m_tf_translation;
        Mat m_tf_rotation;
        Mat m_intrinsic_params;
        Mat m_distortion_matrix;
        Mat m_image;
        std::vector<Point3f> m_position_3d;
        std::vector<unsigned char> m_value_color;
        std::string m_filter_transparency;
        std::string m_erosion_size;
	

        // projection trajectory
	    std::vector<Point> m_traj;
        std::vector<Point3f> m_traj_3d;
        int m_num_points;
        float m_time_horizon;
        bool m_draw_traj,
	     m_init_traj;
        
        // Debug
        bool m_debug;
        bool m_low_ressources;
        bool m_get_image=false;

        // Gst
        std::string m_gst_out;
        std::string m_gst_pipeline_1;
        std::string m_gst_pipeline_2;
        std::string m_image_width;
        std::string m_image_height;
        std::string m_pcc_host;
        std::string m_pcc_port;

        VideoWriter m_writer;

        // TF
        tf::TransformListener m_listener;
        tf::StampedTransform m_transform_for_camera,
                        m_transform_robot_to_camera;

        // Subscribers
        image_transport::Subscriber m_cameraSubscriber;
        ros::Subscriber m_imuSubscriber;
        ros::Subscriber m_gridmaplayerSubscriber;
        ros::Subscriber m_CmdVelSubscriber;

        // Publisher
        ros::Publisher m_Pub_image;

    public:
        BarakudaVision(ros::NodeHandle& n);

        // Callbacks

        void cameraCallback(const sensor_msgs::ImageConstPtr& _img);

        void imuCallback(const sensor_msgs::Imu& _imu);

        void gridmaplayerCallback(const grid_map_msgs::GridMap& _gridmap);

        void cmd_vel_Callback(const geometry_msgs::Twist& cmd_vel);

        // Functions

        void cameraInfoInit(ros::NodeHandle nh);

        void tfInit(const std::string &target_frame, const std::string &source_frame, const std::string &optical_frame);

};
