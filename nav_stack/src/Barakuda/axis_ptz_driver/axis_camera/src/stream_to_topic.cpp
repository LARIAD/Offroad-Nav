#include <ros/ros.h>
#include <gst/gst.h>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>	
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


using namespace cv;

int main(int argc, char **argv)
{
    //Camera Input
    ros::init(argc, argv, "axis_stream_cap");
    ros::NodeHandle n;
    
    std::string user_name,password,frame_id,ip_adresse;
    
    n.param<std::string>("username", user_name, "root");
    n.param<std::string>("password", password, "admin");
    n.param<std::string>("frame_id", frame_id, "axis_camera_link");
    n.param<std::string>("hostname", ip_adresse, "192.168.0.90");


    int width,height,fps;
    n.param<int>("width", width , 1920);
    n.param<int>("height", height , 1080);
    n.param<int>("fps", fps , 50);
    
    ros::Publisher Pub_image = n.advertise<sensor_msgs::CompressedImage>("/axis/image_raw/compressed", 1);
    
    ros::Publisher cam_info_pub = n.advertise<sensor_msgs::CameraInfo>("/axis/camera_info", 1000);
    sensor_msgs::CameraInfo msg_cam_info;
    msg_cam_info.header.frame_id=frame_id;
    msg_cam_info.width=1920;
    msg_cam_info.height=1080;
    
    Mat image;

    //std::string Pipe="rtspsrc location=rtsp://"+user_name+":"+password+"@192.168.0.90:554/axis-media/media.amp latency=0 ! decodebin max-size-time=30000000000 ! nvvidconv ! video/x-raw,width=1920,height=1080,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"; //! videoconvert ! videoscale ! video/xraw,width="+std::to_string(width)+",height="+std::to_string(height)+ " ! nvvidconv ! videoconvert ! autovideosink";//appsink sync=0";
    std::string Pipe="rtspsrc location=rtsp://"+user_name+":"+password+"@"+ip_adresse+":554/axis-media/media.amp latency=0 ! decodebin ! nvvidconv ! video/x-raw,width=1920,height=1080,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! videoconvert ! videoscale ! video/x-raw,width="+std::to_string(width)+",height="+std::to_string(height)+" ! appsink drop=1"; 
    VideoCapture cap(Pipe,cv::CAP_GSTREAMER);
    
    
    if (!cap.isOpened()) {
        printf("Can't create capture\n");
        //return -1;
    }
    if (fps==0){
       fps=50;
    }
    ros::Rate loop_rate(fps);
    
    while (ros::ok() & cap.isOpened()){
        cap >> image;
        //if (image.empty()) continue;
        cv_bridge::CvImagePtr cv_ptr;
		    
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (cv_ptr->image.empty()) continue;
        std::vector<int> compression_params;
        
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY); 
        
        compression_params.push_back(90); 
// Qualité JPEG entre 0 et 100 std:
        std::vector<uchar> compressed_data; 
        
        cv::imencode(".jpg", cv_ptr->image, compressed_data, compression_params); 
// Création du message CompressedImage sens
        sensor_msgs::CompressedImage compressed_msg; 
        
        compressed_msg.header = msg_cam_info.header; 
// Copie de l'en-tête (timestamp, frame_id) comp
        compressed_msg.format = "jpeg"; // Format d'image compressée // Assignation des données compressées compressed_m
        compressed_msg.data = std::move(compressed_data); // Publication de l'image compressée compressed_image_pub_.publish(compressed_msg);
        Pub_image.publish(compressed_msg);
        
        msg_cam_info.header.stamp=ros::Time::now();
        cam_info_pub.publish(msg_cam_info);
    
        ros::spinOnce();
 
        loop_rate.sleep();
    }


    return 0;
}



