#include "barakuda_vision/barakuda_vision.h"

void draw_line_faster(Mat& img,Point begin,Point end,int size_line,Scalar color){
    
    int dif_y=begin.y-end.y,dif_x=begin.x-end.x;
    Point first=begin, second=end;
    if (abs(dif_x)>abs(dif_y)){
        
        if (first.x>second.x){
            first=end;
            second=begin;
        }
        float factor=(float)(second.y-first.y)/(float)(second.x-first.x);
        parallel_for_(Range(0,size_line ), [&](const Range& range1){
            for(std::size_t u = range1.start; u< range1.end; u++){
                //ROS_WARN("info :  u %d ",u);
                parallel_for_(Range(max(first.x,0), min(second.x,img.cols-1)), [&](const Range& range){
                    
                    for(std::size_t i = range.start; i< range.end; i++){
                //for(std::size_t i = first.x; i< second.x; i++){
                        int j=(i-first.x)*factor+first.y+(int)u-size_line/2;
                        /*if (first.y>img.rows*0.8){
                            color[0]=255;
                            color[1]=0;
                            color[2]=0;
                            ROS_WARN("info :  dif x %d ; y %d",dif_x,dif_y);
                            ROS_WARN("      first.x %d, first.y %d",first.x, first.y);
                            ROS_WARN("      second.x %d, second.y %d",second.x, second.y);
                            ROS_WARN("      factor %f",factor);
                            //line( img, first, second, Scalar(0,255,0),1, LINE_8 );
                            color[0]=255;
                            color[1]=0;
                            color[2]=0;
                        };*/
                        //if (factor<0) color[0]=255;
                        if ( j>=0 && j<img.rows){
                            img.at<Vec3b>(j, i)[0] = color[0];//Changing the value of first channel//
                            img.at<Vec3b>(j, i)[1] = color[1];//Changing the value of first channel//
                            img.at<Vec3b>(j, i)[2] = color[2];
                        }
                    }
                });
            };
        });
    }else{
        //Point first=begin, second=end;
        if (first.y>second.y){
            first=end;
            second=begin;
        }
            
        float factor=(float)(second.x-first.x)/(float)(second.y-first.y);
        parallel_for_(Range(0,size_line ), [&](const Range& range1){
            for(std::size_t u = range1.start; u< range1.end; u++){
                parallel_for_(Range(max(first.y,0), min(second.y,img.rows-1)), [&](const Range& range){
                    for(std::size_t j = range.start; j< range.end; j++){
                //for(std::size_t j = first.y; j< second.y; j++){
                        int i=(j-first.y)*factor+first.x+(int)u-size_line/2;
                        
                        if (i>=0 && i<img.cols){
                            img.at<Vec3b>(j, i)[0] = color[0];//Changing the value of first channel//
                            img.at<Vec3b>(j, i)[1] = color[1];//Changing the value of first channel//
                            img.at<Vec3b>(j, i)[2] = color[2];
                        }
                    }
                });
            }
        });
    }
}

BarakudaVision::BarakudaVision(ros::NodeHandle& n)
{
    ros::NodeHandle private_n("~");
    m_init_traj=false;
    // Params
        // Topics
    private_n.param<std::string>("camera_topic", m_camera_topic, "/camera");
    private_n.param<std::string>("camera_info_topic", m_camera_info_topic, "/camera_info");
    private_n.param<std::string>("imu_topic", m_imu_topic, "/imu/data");
    private_n.param<std::string>("gridmap_layer_topic", m_gridmap_layer_topic, "/layer");
    private_n.param<std::string>("cmd_vel", m_cmd_vel_topic , "/cmd_vel");

        // Gst pipeline
    private_n.param<std::string>("gst_pipeline_1", m_gst_pipeline_1, "appsrc ! videoconvert ! videoscale ! video/x-raw,");
    private_n.param<std::string>("gst_pipeline_2", m_gst_pipeline_2, ", framerate=30/1, format=YUY2 ! videoconvert ! jpegenc ! rtpjpegpay !");
    private_n.param<std::string>("image_width", m_image_width, "640");
    private_n.param<std::string>("image_height", m_image_height, "360");
    private_n.param<std::string>("pcc_host", m_pcc_host, "127.0.0.1");
    private_n.param<std::string>("pcc_port", m_pcc_port, "5000");

        // Visualisation
    private_n.param<std::string>("filter_transparency", m_filter_transparency, "150");
    private_n.param<std::string>("erosion_size", m_erosion_size, "4");
    
        // TF
    private_n.param<std::string>("hight_info_frame", m_hight_info_frame , "plane_tf");
    private_n.param<std::string>("robot_frame", m_robot_frame , "base_link");
    private_n.param<std::string>("camera_frame", m_camera_frame, "camera_frame");
    private_n.param<std::string>("optical_frame", m_optical_frame, "camera_frame");

        // Param
    private_n.param<bool>("debug", m_debug, true);
    private_n.param<bool>("low_ressources", m_low_ressources, true);
    private_n.param<float>("time_horizon", m_time_horizon , 5);
    private_n.param<int>("number_of_points", m_num_points , 10);
    private_n.param<bool>("draw_traj", m_draw_traj, true);
    
    // Wait for the TF between camera and lidar projected plan
    m_listener.waitForTransform(m_hight_info_frame, m_optical_frame, ros::Time::now(), ros::Duration(3.0));
    
    // Init camera infos
    cameraInfoInit(n);
    
    // Init sensor's tf
    tfInit(m_hight_info_frame, m_camera_frame,m_optical_frame);

    // IT Subsciber
    image_transport::ImageTransport it(n);

    // Subscribers
    m_cameraSubscriber = it.subscribe(m_camera_topic, 10, &BarakudaVision::cameraCallback, this);
    m_imuSubscriber = n.subscribe(m_imu_topic, 10, &BarakudaVision::imuCallback, this);
    m_CmdVelSubscriber = n.subscribe(m_cmd_vel_topic, 10, &BarakudaVision::cmd_vel_Callback, this);
    m_gridmaplayerSubscriber = n.subscribe(m_gridmap_layer_topic, 10, &BarakudaVision::gridmaplayerCallback, this);
    
    // Publish
    m_Pub_image = n.advertise<sensor_msgs::CompressedImage>("/super_image", 1);
    
    // Init Gstreamer pipeline
    m_gst_out = m_gst_pipeline_1 + "width=" + m_image_width + ", height=" + m_image_height + m_gst_pipeline_2 + " udpsink host=" + m_pcc_host +" port=" + m_pcc_port;
    ROS_WARN("%s", m_gst_out.c_str());

    // Create OpenCv writer
    m_writer.open(m_gst_out, 0, (double)30, Size(std::stoi(m_image_width), std::stoi(m_image_height)), true);

    if (!m_writer.isOpened()) {
        printf("Can't create writer\n");
    }

}

// Callbacks
void BarakudaVision::cameraCallback(const sensor_msgs::ImageConstPtr& _img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      /* Convert the ROS image message to a CvImage suitable for working with OpenCV
       * Note: if we plan to modify the image, we need a mutable copy of it, so we use toCvCopy()
       * Else, use toCvShare() to share without copy (more efficient)
       */
      cv_ptr = cv_bridge::toCvCopy(_img, sensor_msgs::image_encodings::BGR8); // = bgr8
    }
    /* catch conversion errors as toCvCopy() / toCvShared() will not check for the validity of the data */
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    m_image=(cv_ptr->image);
    m_get_image=true;
}
// TO DO 
void BarakudaVision::imuCallback(const sensor_msgs::Imu& _imu)
{
    m_imu = _imu;
}
void BarakudaVision::cmd_vel_Callback(const geometry_msgs::Twist& cmd_vel){
    /* compute trajectory from cmd_vel with time horizon
            it use : m_time_horizon
                     m_num_points number of point in traj
                     m_transform_robot_to_camera,m_tf_rotation,m_tf_translation,m_intrinsic_params for projection
                     m_image destination of projection    
    */

    m_cmd_vel=cmd_vel;
    /*if(m_cmd_vel.linear.x == 0 & m_cmd_vel.angular.z == 0){
        ROS_WARN("no velocity ");
        return;
    }
    int xsize = m_image.rows;
    int ysize = m_image.cols;
    if(xsize == 0){
        ROS_WARN("Wait for the image to be initialised ... ");
        return;
    }
    
    double velo_lin=m_cmd_vel.linear.x,
           velo_ang=m_cmd_vel.angular.z;
    tf::Vector3 point(0,0,0);
    m_traj_3d.push_back(Point3f(point.getX(),point.getY(),point.getZ()));
    float time_step=m_time_horizon/m_num_points;
    for (int i=0; i<m_num_points;i++ ){
        point.setX(velo_lin*time_step *cos(velo_ang*time_step));
        point.setY(velo_lin*time_step *sin(velo_ang*time_step));
        point =  m_transform_robot_to_camera * point;
	ROS_WARN("okk created  point : %lf, %lf",point.getX(),point.getY());
        m_traj_3d.push_back(Point3f(point.getX(),point.getY(),point.getZ()));
    }
    if (m_draw_traj){
        Mat tempo_mask_g = Mat::zeros(m_image.size(), CV_8UC3);
        std::vector<Point2f> position_2d;
        std::vector<Point> position;
        Mat tf_translation=m_tf_rotation.clone();
        Mat tf_rotation=m_tf_translation.clone();
        projectPoints(m_traj_3d, tf_rotation, tf_translation, m_intrinsic_params, m_distortion_matrix, position_2d);
        Mat(position_2d).convertTo(m_traj, Mat(m_traj).type());
        ROS_WARN("okkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk23");
        for (int i=0; i<m_traj.size()-1;i++){
	    ROS_WARN("okk point 1: %d, %d   point 2: %d, %d",m_traj[i].x,m_traj[i].y,m_traj[i+1].x,m_traj[i+1].y); 
            if (m_traj[i] != m_traj[i+1] && (m_traj[i].x>0 && m_traj[i+1].x>0 &&
                        m_traj[i].x<ysize && m_traj[i+1].x<ysize &&
                        m_traj[i].y>0 && m_traj[i+1].y>0 &&
                        m_traj[i].y<xsize && m_traj[i+1].y<xsize) ){
            ROS_WARN("priiiiiiiiiiiiiiiiiiiiiiiint");
            draw_line_faster(tempo_mask_g,m_traj[i], m_traj[i+1],1, Scalar(255,0,0));
            }
       }
        ROS_WARN("okkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk24");
        m_image+=tempo_mask_g;
    }
    m_traj_3d.clear();
    m_traj.clear();*/
}

void BarakudaVision::gridmaplayerCallback(const grid_map_msgs::GridMap& grid_map_msg)
{   
    int xsize = m_image.rows;
    int ysize = m_image.cols;

    ROS_WARN("xsize %d, ysize %d", xsize, ysize);
    
    // Don't call the callback as long as the image is not initialised 
    if(xsize == 0){
        //ROS_WARN("Wait for the image to be initialised ... ");
        return;
    }

    //wait new image
    if (!m_get_image) return;
    m_get_image=false;
    
    ros::Time time = ros::Time::now();
    ros::Duration duration;
    if(m_debug){
        //ROS_WARN(" ");
        //ROS_WARN("-----------------------------");
        //ROS_WARN(" ");
        duration = ros::Time::now() - time;
        //ROS_WARN("Gridmap frame received %lf", duration.toSec());
    }
    
    
    std::string layer = "elevation";

    grid_map::GridMap grid_map;
    grid_map::Position grid_map_position;
    grid_map::Position3 grid_map_position3;

    //std::vector<Point3f> position_3d_green;//,position_3d_yellow, position_3d_orange, position_3d_red;
    //std::vector<std::vector<Point3f>> position_red_3d_stack;
    std::vector<Point2f> position_2d_green;//,position_2d_yellow, position_2d_orange, position_2d_red;
    std::vector<Point> position_green;//,position_yellow, position_orange, position_red, green_toprint, yellow_toprint, orange_toprint, red_toprint;
    std::vector<std::vector<Point>> drawing_2d_vector;
    //std::vector<unsigned char> value_color;
    //Convert grid map in ROS standard
    grid_map::GridMapRosConverter::fromMessage(grid_map_msg, grid_map);

    // Get the grid map resolution 
    double resolution = grid_map.getResolution();
    resolution *= 1;
    
    //m_listener.waitForTransform(m_hight_info_frame, m_camera_frame, ros::Time::now(), ros::Duration(20.0));
    //ROS_WARN("... done !");
    m_listener.lookupTransform(m_camera_frame,m_hight_info_frame, ros::Time(0), m_transform_for_camera);

    grid_map::Size size = grid_map.getSize();
    
    m_position_3d.reserve(size(0)*size(1)*2);
    m_value_color.reserve(size(0)*size(1));

    Mat tf_translation=m_tf_translation.clone();
    Mat tf_rotation=m_tf_rotation.clone();

    // Sort the points depending on the grip map height
    bool wait_last_points=false;
    int number_element=0;
    
    std::vector<int> index_value;
    Mat tempo_mask_g = Mat::zeros(m_image.size(), CV_8UC3);
    int filter_transparency = std::stoi(m_filter_transparency);
    tf::Vector3 pointref(0,0,0);
    pointref =  m_transform_for_camera * pointref;
    
    for (int i = 0; i < size(0); i++) {
        index_value.push_back(m_position_3d.size()/2);
        wait_last_points=false;
        for (int j = 0; j < size(1); j++){
            grid_map::Index index(i,j);
            int value_color;
            grid_map.getPosition3(layer, index, grid_map_position3);
            //ROS_WARN("position x , y: %f , %f", -grid_map_position3(0) , -grid_map_position3(1));
            // See comments below
            tf::Vector3 point(grid_map_position3(0),grid_map_position3(1),grid_map_position3(2));
            point =  m_transform_for_camera * point;
            if (point.x()<=0) {
                //i=size(0);
                //break;
                continue;
            }
            if (grid_map_position3(2) < 0.05 ){
                //tf::Vector3 point(-grid_map_position3(0),-grid_map_position3(1)-resolution/2,-grid_map_position3(2)+resolution/2);
                //tf::Vector3 point(-grid_map_position3(0),-grid_map_position3(1),-grid_map_position3(2));
                //point =  m_transform_for_camera * point;
                if (!wait_last_points){
                    //m_value_color.push_back(0);
                    
                    
                    //m_position_3d.push_back(Point3f(point.getX(),point.getY(),point.getZ()));
                    point =  m_transform_for_camera * tf::Vector3(grid_map_position3(0),grid_map_position3(1),grid_map_position3(2));
                    //m_position_3d.push_back(Point3f(point.getX(),point.getY(),point.getZ()));
                    /*
                    position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                    point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)-resolution/2,-grid_map_position3(2)-resolution/2);
                    position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                    point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)+resolution/2,-grid_map_position3(2)-resolution/2);
                    position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                    point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)+resolution/2,-grid_map_position3(2)+resolution/2);
                    position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));*/
                    wait_last_points=true;
                }//else{
                    //point =  m_transform_for_camera * tf::Vector3(grid_map_position3(0),grid_map_position3(1),grid_map_position3(2));
                 //   m_position_3d[m_position_3d.size()-1]=Point3f(point.getX(),point.getY(),point.getZ());
                    
                    /*
                    point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)+resolution/2,-grid_map_position3(2)-resolution/2);
                    position_3d_green[position_3d_green.size()-2]=Point3d(point.getX(),point.getY(),point.getZ());
                    point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)+resolution/2,-grid_map_position3(2)+resolution/2);
                    position_3d_green[position_3d_green.size()-1]=Point3d(point.getX(),point.getY(),point.getZ());*/
                //}
            }else if (grid_map_position3(2) < 0.15){
                // Get the Gridmap point
                //tf::Vector3 point(-grid_map_position3(0),-grid_map_position3(1)+resolution/2,-grid_map_position3(2));
                //tf::Vector3 point(-grid_map_position3(0),-grid_map_position3(1),-grid_map_position3(2));
                // Tranform the point in the camera frame
                //point =  m_transform_for_camera * point;
                //Push back the projected element
                m_value_color.push_back(1);
                value_color=1;

                m_position_3d.push_back(Point3f(point.getX(),point.getY(),point.getZ()));
                point =  m_transform_for_camera * tf::Vector3(grid_map_position3(0),grid_map_position3(1),0);

                m_position_3d.push_back(Point3f(point.getX(),point.getY(),point.getZ()));
                /*position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));

                point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)+resolution/2,0);
                position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)-resolution/2,0);
                position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)-resolution/2,-grid_map_position3(2));
                position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));*/

                /*/Create n virtual point between the grid map point dans the ground plane
                double height=0.000;
                while(height<grid_map_position3(2)){
                    // Project virtual points
                    tf::Vector3 point2(-grid_map_position3(0),-grid_map_position3(1),-height);
                    point2 =  m_transform_for_camera * point2;
                    // Push back as well
                    position_3d_yellow.push_back(Point3d(point2.getX(),point2.getY(),point2.getZ()));
                    // Increment height
                    height+=resolution;
                }*/
                wait_last_points=false;
            // See comments above
            }else if (grid_map_position3(2) < 0.30){
                //tf::Vector3 point(-grid_map_position3(0),-grid_map_position3(1),-grid_map_position3(2));
                //point =  m_transform_for_camera * point;
                m_value_color.push_back(2);
                value_color=2;

                m_position_3d.push_back(Point3f(point.getX(),point.getY(),point.getZ()));
                point =  m_transform_for_camera * tf::Vector3(grid_map_position3(0),grid_map_position3(1),0);

                m_position_3d.push_back(Point3f(point.getX(),point.getY(),point.getZ()));
                /*
                position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                
                point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)+resolution/2,0);
                position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)-resolution/2,0);
                position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)-resolution/2,-grid_map_position3(2));
                position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));*/

                /*
                double height=0.000;

                while(height<grid_map_position3(2)){
                    tf::Vector3 point2(-grid_map_position3(0),-grid_map_position3(1),-height);
                    point2 =  m_transform_for_camera * point2;
                    position_3d_orange.push_back(Point3d(point2.getX(),point2.getY(),point2.getZ()));
                    height+=resolution;
                }*/
                wait_last_points=false;
            // See comments above
            }else{
                //tf::Vector3 point(-grid_map_position3(0),-grid_map_position3(1),-grid_map_position3(2));
                //point =  m_transform_for_camera * point;
                m_value_color.push_back(3);
                value_color=3;

                m_position_3d.push_back(Point3f(point.getX(),point.getY(),point.getZ()));
                point =  m_transform_for_camera * tf::Vector3(grid_map_position3(0),grid_map_position3(1),0);

                m_position_3d.push_back(Point3f(point.getX(),point.getY(),point.getZ()));/*

                position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                /*if(!m_low_ressources){
                    double height=0.000;
                    while(height<grid_map_position3(2)){
                        tf::Vector3 point2(-grid_map_position3(0),-grid_map_position3(1),-height);
                        point2 =  m_transform_for_camera * point2;
                        position_3d_red.push_back(Point3d(point2.getX(),point2.getY(),point2.getZ()));
                        height+=resolution;
                    }
                }else{*/
                /*
                point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)+resolution/2,0);
                position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)-resolution/2,0);
                position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                point =  m_transform_for_camera * tf::Vector3(-grid_map_position3(0),-grid_map_position3(1)-resolution/2,-grid_map_position3(2));
                position_3d_green.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                value_color[i+j]=3;
                number_element++;*/
                wait_last_points=false;
                /*position_3d_red.push_back(Point3d(point.getX(),point.getY(),point.getZ()));
                position_3d_red.push_back(Point3d(point.getX(),point.getY(),0));
                double height=0.000;
                while(height<grid_map_position3(2)){
                    tf::Vector3 point2(-grid_map_position3(0),-grid_map_position3(1),-height);
                    point2 =  m_transform_for_camera * point2;
                    position_3d_red.push_back(Point3d(point2.getX(),point2.getY(),point2.getZ()));
                    height+=resolution;
                }*/
            }

            // if (!wait_last_points & m_position_3d.size()>1){
            //     std::size_t indice=m_position_3d.size();
            //     position_2d_green.clear();
            //     std::vector<Point3f> tempo;
            //     tempo.push_back(m_position_3d[indice-2]);
            //     tempo.push_back(m_position_3d[indice-1]);
                
            //     projectPoints(tempo,  m_tf_rotation, m_tf_translation, m_intrinsic_params, m_distortion_matrix, position_2d_green);
            //     Mat(position_2d_green).convertTo(position_green, Mat(position_green).type());
            //     Scalar color(0,0,0);
            
            //     switch (value_color)
            //     {
            //     case 0:
            //         color= Scalar(0,filter_transparency,0);
                    
            //         break;
            //     case 1:
            //         color= Scalar(0,filter_transparency,filter_transparency);
            //         break;
            //     case 2:
            //         color= Scalar(0,filter_transparency*0.5,filter_transparency);
            //         break;
            //     case 3:
            //         color= Scalar(0,0,filter_transparency);
            //         break;
            //     default:
            //         break;
            //     }
            //     line( tempo_mask_g, position_green[i], position_green[i+1], color, 4, LINE_8 );
            // }
        }
    }
    
    if(m_debug){
        duration = ros::Time::now() - time;
        //ROS_WARN("End of point sorting %lf", duration.toSec());
    }
    

    // Check if the close plan array is not empty
    if(m_position_3d.size() != 0){
        //Project all the 3D points (projected from the lidar frame to the camera frame) into 2D

        //ROS_WARN("Vector position_3d_green size: %ld", position_3d_green.size());
        projectPoints(m_position_3d, m_tf_rotation, m_tf_translation, m_intrinsic_params, m_distortion_matrix, position_2d_green);
        m_position_3d.clear();
        //ROS_WARN("Vector position_3d_yellow size: %ld", position_3d_yellow.size());
        //projectPoints(position_3d_yellow, m_tf_rotation, m_tf_translation, m_intrinsic_params, m_distortion_matrix, position_2d_yellow);

        //ROS_WARN("Vector position_3d_orange size: %ld", position_3d_orange.size());
        //projectPoints(position_3d_orange, m_tf_rotation, m_tf_translation, m_intrinsic_params, m_distortion_matrix, position_2d_orange);

        //ROS_WARN("Vector position_3d_red size: %ld", position_3d_red.size());
        //projectPoints(position_3d_red, m_tf_rotation, m_tf_translation, m_intrinsic_params, m_distortion_matrix, position_2d_red);

        if(m_debug){
            duration = ros::Time::now() - time;
            //ROS_WARN("End of point projection %f", duration.toSec());
        }

        // Convert point from Points2f to Points
        Mat(position_2d_green).convertTo(position_green, Mat(position_green).type());
        //Mat(position_2d_yellow).convertTo(position_yellow, Mat(position_yellow).type());
        //Mat(position_2d_orange).convertTo(position_orange, Mat(position_orange).type());
        //Mat(position_2d_red).convertTo(position_red, Mat(position_red).type());

        if(m_debug){
            duration = ros::Time::now() - time;
            //ROS_WARN("End of matrix convert %lf", duration.toSec());
        }

        
        // Split the image into 3 channels
        std::vector<Mat> channels(3);

        split(m_image, channels);
        int filter_transparency = std::stoi(m_filter_transparency);
        int erosion_size = std::stoi(m_erosion_size);

        //Mat element = getStructuringElement( MORPH_RECT,
        //Size( (2*erosion_size + 1), (2*erosion_size+1)),
        //Point( erosion_size, erosion_size ));

        // Create a temporary mask
        Mat tempo_mask_g = Mat::zeros(m_image.size(), CV_8UC3);
        //Mat tempo_mask_g = Mat::zeros(m_image.size(), CV_8UC1);
        //Mat tempo_mask_r=Mat::zeros(m_image.size(), CV_8UC1);
        std::size_t u=0;
        int size_max_line=3;
        int lineType = LINE_8;
        float line_size_factor=1./(float) index_value.size();
        for(std::size_t t=1; t<index_value.size();t++){
            //int line_size=(size_max_line*line_size_factor*t)+1;
            //ROS_WARN("line_size %d",line_size);
	        parallel_for_(Range(index_value[t-1], index_value[t]), [&](const Range& range){
                for(std::size_t i = range.start; i< range.end; i++){
            //for(std::size_t i = 0; i< position_green.size(); i+=2){
                    //std::vector<Point> tempo(position_green.begin()+i,position_green.begin()+i+4);
                    /*tempo.insert
                    tempo.push_back(position_red[i]);
                    tempo.push_back(position_red[i+1]);
                    tempo.push_back(position_red[i+2]);
                    tempo.push_back(position_red[i+3]);*/
                    //drawing_2d_vector.push_back(tempo);
                    
                    if (position_green[i*2].x<0 && position_green[i*2+1].x<0 ||
                        position_green[i*2].x>ysize && position_green[i*2+1].x>ysize ||
                        position_green[i*2].y<0 && position_green[i*2+1].y<0 ||
                        position_green[i*2].y>xsize && position_green[i*2+1].y>xsize) continue;
                    Scalar color(0,0,0);
                    
                    switch (m_value_color[i])
                    {
                    case 0:
                        color= Scalar(0,filter_transparency,0);
                        //fillPoly(tempo_mask_g, drawing_2d_vector, Scalar(filter_transparency));
                        //fillPoly(tempo_mask_r, drawing_2d_vector, Scalar(0));
                        break;
                    case 1:
                        //fillPoly(tempo_mask_g, drawing_2d_vector, Scalar(filter_transparency));
                        //fillPoly(tempo_mask_r, drawing_2d_vector, Scalar(filter_transparency));
                        color= Scalar(0,filter_transparency,filter_transparency);
                        break;
                    case 2:
                        //fillPoly(tempo_mask_g, drawing_2d_vector, Scalar(filter_transparency*0.5));
                        //fillPoly(tempo_mask_r, drawing_2d_vector, Scalar(filter_transparency));
                        color= Scalar(0,filter_transparency*0.5,filter_transparency);
                        break;
                    case 3:
                        //fillPoly(tempo_mask_g, drawing_2d_vector, Scalar(0));
                        //fillPoly(tempo_mask_r, drawing_2d_vector, Scalar(filter_transparency));
                        color= Scalar(0,0,filter_transparency);
                        break;
                    default:
                        break;
                    }
                    //u++;
                    draw_line_faster(tempo_mask_g,position_green[i*2], position_green[i*2+1],1, color);
                    //line( tempo_mask_g, position_green[i*2], position_green[i*2+1], color,1, lineType );
                    //fillPoly(tempo_mask, drawing_2d_vector, Scalar(1));(3*line_size_factor*t)+1
                    //fillPoly(tempo_mask, drawing_2d_vector, color);
                    //drawing_2d_vector.clear();
                }
	        });
        }
        //channels[1]+=tempo_mask_g;
        //channels[2]+=tempo_mask_r;
        //merge(channels, m_image);

        //draw_line_faster(tempo_mask_g, Point(0,xsize-4),Point(ysize,xsize+1),1, Scalar(255,0,0));
	if(m_draw_traj && !(m_cmd_vel.linear.x == 0 && m_cmd_vel.angular.z == 0) && m_init_traj){
	  
          ROS_WARN("okk cmd_vel : %lf, %lf",m_cmd_vel.linear.x,m_cmd_vel.angular.z);
    	  float velo_lin=m_cmd_vel.linear.x;
          float velo_ang=m_cmd_vel.angular.z;
          ROS_WARN("okk cmd_vel : %f, %f",velo_lin,velo_ang);
          tf::Vector3 new_point(0,0,0);
          m_traj_3d.push_back(Point3f(new_point.getX(),new_point.getY(),new_point.getZ()));
          float time_step=m_time_horizon/(float)m_num_points;
          for (int i=1; i<m_num_points+1;i++ ){
             new_point.setX(velo_lin*time_step*(float)i *cos(velo_ang*time_step*(float)i));
             new_point.setY(velo_lin*time_step*(float)i *sin(velo_ang*time_step*(float)i));
             new_point.setZ(0);
             ROS_WARN("okk created base_link lin cos sin : %lf, %lf, %lf",velo_lin*time_step*i,cos(velo_ang*time_step*i),sin(velo_ang*time_step*i));
             ROS_WARN("okk created base_link point : %lf, %lf",new_point.getX(),new_point.getY());
             new_point =  m_transform_robot_to_camera * new_point;
             ROS_WARN("okk created  point : %lf, %lf, %lf",new_point.getX(),new_point.getY(),new_point.getZ());
             if (new_point.getX()>0)
             m_traj_3d.push_back(Point3f(new_point.getX(),new_point.getY(),new_point.getZ()));
          }
          if (m_draw_traj){
            
            std::vector<Point2f> new_position_2d;
            
            // Mat tf_translation=m_tf_translation.clone();
            // Mat tf_rotation=m_tf_rotation.clone();
            projectPoints(m_traj_3d, tf_rotation, tf_translation, m_intrinsic_params, m_distortion_matrix, new_position_2d);
            Mat(new_position_2d).convertTo(m_traj, Mat(m_traj).type());
            ROS_WARN("okkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk23");
            for (int i=0; i<m_traj.size()-1;i++){
	      if (m_traj[i+1].y>ysize && m_traj[i].y<ysize)
                  m_traj[i+1]=Point((m_traj[i].x-m_traj[i+1].x)*m_traj[i].y/(m_traj[i].y-m_traj[i+1].y),ysize);
              if (m_traj[i+1].y<0 && m_traj[i].y>0)
                  m_traj[i+1]=Point((m_traj[i].x-m_traj[i+1].x)*m_traj[i].y/(m_traj[i].y-m_traj[i+1].y),0);
              ROS_WARN("okk point 1: %d, %d   point 2: %d, %d",m_traj[i].x,m_traj[i].y,m_traj[i+1].x,m_traj[i+1].y); 
              if (m_traj[i] != m_traj[i+1] && (m_traj[i].x>0 && m_traj[i+1].x>0 &&
                  m_traj[i].x<ysize && m_traj[i+1].x<ysize &&
                  m_traj[i].y>0 && m_traj[i+1].y>0 &&
                  m_traj[i].y<xsize && m_traj[i+1].y<xsize) ){
                  ROS_WARN("priiiiiiiiiiiiiiiiiiiiiiiint");
                  draw_line_faster(tempo_mask_g,m_traj[i], m_traj[i+1],3, Scalar(255,0,0));
               }
            }
            //draw_line_faster(tempo_mask_g,Point(), m_traj[i+1],1, Scalar(255,0,0));
            
            m_traj_3d.clear();
            m_traj.clear();
            

          }
        }
        m_image+=tempo_mask_g;
        m_init_traj=true;
        /*
        for(int i = 0; i< position_green.size(); i++){
            //Add all the green points to this temporary mask
            circle(tempo_mask, position_green[i], 3, Scalar(filter_transparency),1);
        }
        // Erode and dilate the mask
        if(!m_low_ressources){
            dilate( tempo_mask, tempo_mask, element );
            dilate( tempo_mask, tempo_mask, element );
            erode( tempo_mask, tempo_mask, element );
            erode( tempo_mask, tempo_mask, element );
        }
        
        // Add the temporart mask to the green channel
        channels[1]+=tempo_mask*filter_transparency;
        // Merge the mask and the channels
        merge(channels, m_image);
        */
        //m_image+=tempo_mask;

        if(m_debug){
            duration = ros::Time::now() - time;
            //ROS_WARN("End of green display %lf", duration.toSec());
        }

        m_position_3d.clear();
        m_value_color.clear();

        



        // tempo_mask1=tempo_mask.clone();
        // // Same as above
        // tempo_mask=Mat::zeros(m_image.size(), CV_8UC1);
        

        // for(std::size_t i = 0; i< position_yellow.size(); i+=4){
        //     std::vector<Point> tempo(position_yellow.begin()+i,position_yellow.begin()+i+4);
        //     /*tempo.insert
        //     tempo.push_back(position_red[i]);
        //     tempo.push_back(position_red[i+1]);
        //     tempo.push_back(position_red[i+2]);
        //     tempo.push_back(position_red[i+3]);*/
        //     drawing_2d_vector.push_back(tempo);
        //     fillPoly(tempo_mask, drawing_2d_vector, Scalar(1));
        //     drawing_2d_vector.clear();
        // }
        // /*
        // for(int i = 0; i< position_yellow.size(); i++){
        //     circle(tempo_mask, position_yellow[i], 3, Scalar(filter_transparency),1);
        // }*/

        // if(!m_low_ressources){
        //     dilate( tempo_mask, tempo_mask, element );
        //     dilate( tempo_mask, tempo_mask, element );
        //     erode( tempo_mask, tempo_mask, element );
        //     erode( tempo_mask, tempo_mask, element );
        // }
        // tempo_mask-=tempo_mask1;
        // split(m_image, channels);
        // channels[1]+=tempo_mask*filter_transparency;
        // channels[2]+=tempo_mask*filter_transparency;
        // merge(channels, m_image);

        // if(m_debug){
        //     duration = ros::Time::now() - time;
        //     ROS_WARN("End of yellow display %lf", duration.toSec());
        // }
        
        // tempo_mask = Mat::zeros(m_image.size(), CV_8UC1);

        // for(std::size_t i = 0; i< position_orange.size(); i+=4){
        //     std::vector<Point> tempo(position_orange.begin()+i,position_orange.begin()+i+4);
        //     /*tempo.insert
        //     tempo.push_back(position_red[i]);
        //     tempo.push_back(position_red[i+1]);
        //     tempo.push_back(position_red[i+2]);
        //     tempo.push_back(position_red[i+3]);*/
        //     drawing_2d_vector.push_back(tempo);
        //     fillPoly(tempo_mask, drawing_2d_vector, Scalar(1));
        //     drawing_2d_vector.clear();
        //     circle(tempo_mask, position_orange[i], 3, Scalar(filter_transparency),1);
        // }*/

        // if(!m_low_ressources){
        //     dilate( tempo_mask, tempo_mask, element );
        //     dilate( tempo_mask, tempo_mask, element );
        //     erode( tempo_mask, tempo_mask, element );
        //     erode( tempo_mask, tempo_mask, element );
        // }
        // tempo_mask-=tempo_mask1;
        // split(m_image, channels);
        // channels[1]+=tempo_mask*0.5*filter_transparency;
        // channels[2]+=tempo_mask*filter_transparency;
        // merge(channels, m_image);

        // if(m_debug){
        //     duration = ros::Time::now() - time;
        //     ROS_WARN("End of orange display %lf", duration.toSec());
        // }
        

        // // Same as above
        // tempo_mask1+=tempo_mask;
        // tempo_mask = Mat::zeros(m_image.size(), CV_8UC1);

        // if(m_debug){
        //     duration = ros::Time::now() - time;
        //     ROS_WARN("Before dilate/erode %lf", duration.toSec());
        // }
        
        // if(!m_low_ressources){
        //     for(int i = 0; i< position_red.size(); i++){
        //         circle(tempo_mask, position_red[i], 3, Scalar(filter_transparency),1);
        //         //line(tempo_mask,position_red[i],position_red[i++],Scalar(filter_transparency),3);
        //     }
        //     /*dilate( tempo_mask, tempo_mask, element );
        //     dilate( tempo_mask, tempo_mask, element );
        //     erode( tempo_mask, tempo_mask, element );
        //     erode( tempo_mask, tempo_mask, element );*/
        // }else{
        //     for(std::size_t i = 0; i< position_red.size(); i+=4){
        //     std::vector<Point> tempo(position_red.begin()+i,position_red.begin()+i+4);
        //     /*tempo.insert
        //     tempo.push_back(position_red[i]);
        //     tempo.push_back(position_red[i+1]);
        //     tempo.push_back(position_red[i+2]);
        //     tempo.push_back(position_red[i+3]);*/
        //     drawing_2d_vector.push_back(tempo);
        //     fillPoly(tempo_mask, drawing_2d_vector, Scalar(1));
        //     drawing_2d_vector.clear();
        // }
        // }

        // if(m_debug){
        //     duration = ros::Time::now() - time;
        //     ROS_WARN("After dilate/erode %lf", duration.toSec());
        // }
        // tempo_mask-=tempo_mask1;
        // split(m_image, channels);
        // channels[2]+=tempo_mask*filter_transparency;
        
        // merge(channels, m_image);
        
        if(m_debug){
            duration = ros::Time::now() - time;
            //ROS_WARN("End of red display %lf", duration.toSec());
        }
        
        
        
        /*
        //green
        drawing_2d_vector.push_back(green_toprint);
        polylines(m_image, drawing_2d_vector, false, Scalar(0,255,0),3);
        //fillPoly(m_image, drawing_2d_vector, Scalar(0,255,0));
        drawing_2d_vector.clear();
        ROS_WARN("Green printed");

        /*
        drawing_2d_vector.push_back(position_yellow);
        fillPoly(m_image, drawing_2d_vector, Scalar(0,255,255));
        drawing_2d_vector.clear();
        ROS_WARN("Yellow printed");

        //orange
        drawing_2d_vector.push_back(position_orange);
        fillPoly(m_image, drawing_2d_vector, Scalar(0,128,255));
        drawing_2d_vector.clear();
        ROS_WARN("Orange printed");
        
        //red
        drawing_2d_vector.push_back(red_toprint);
        polylines(m_image, drawing_2d_vector, false, Scalar(0,255,0),3);
        //fillPoly(m_image, drawing_2d_vector, Scalar(0,0,255));
        ROS_WARN("Red printed");
        */
        // Put the image as a source for the OpenCV writer
        if (!m_image.empty()){
                    cv_bridge::CvImagePtr cv_ptr;
		    
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_image).toImageMsg();
                    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		    std::vector<int> compression_params; compression_params.push_back(cv::IMWRITE_JPEG_QUALITY); compression_params.push_back(90); 
// Qualité JPEG entre 0 et 100 std:
                    std::vector<uchar> compressed_data; cv::imencode(".jpg", cv_ptr->image, compressed_data, compression_params); 
// Création du message CompressedImage sens
                    sensor_msgs::CompressedImage compressed_msg; compressed_msg.header = msg->header; 
// Copie de l'en-tête (timestamp, frame_id) comp
                    compressed_msg.format = "jpeg"; // Format d'image compressée // Assignation des données compressées compressed_m
                    compressed_msg.data = std::move(compressed_data); // Publication de l'image compressée compressed_image_pub_.publish(compressed_msg);
                    m_Pub_image.publish(compressed_msg);
		    m_writer << m_image;
        }
       
        // image display for local debug
        //if (m_image.size.dims()>0){
           //imshow("Image",m_image);
           //waitKey(1);
        //}
    }
}

//Functions

// Camera initialisation function
void BarakudaVision::cameraInfoInit(ros::NodeHandle nh)
{   
    // Wait for camera info message
    //ROS_WARN("Wait for image message ...");
    m_camera_info = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(m_camera_info_topic, nh);
    //ROS_WARN("... done");

    //ROS_WARN("Width: %d, Height: %d", m_camera_info.width, m_camera_info.height);

    // Update camera infos (intrasec parameters, distorition matrix)
    m_intrinsic_params = Mat(3, 3, CV_64FC1, m_camera_info.K.data());
    m_distortion_matrix = Mat(5,1,CV_64FC1, m_camera_info.D.data() );
}

//!\\ TO DO clarifier le nom des TF

// Update the TF between the lidar and the camera
void BarakudaVision::tfInit(const std::string &lidar_frame, const std::string &camera_frame, const std::string &optical_frame){

    tf::StampedTransform transform;

    //Lidar Frame to Camera Frame
    //ROS_WARN("Wait for the /tf between %s and %s ...", lidar_frame.c_str(), camera_frame.c_str());
    m_listener.waitForTransform(lidar_frame, camera_frame, ros::Time::now(), ros::Duration(20.0));
    //ROS_WARN("... done !");
    m_listener.lookupTransform(camera_frame,lidar_frame, ros::Time(0), m_transform_for_camera);
    // tf::Quaternion q1,q;
    // q=m_transform_for_camera.getRotation ();
    // q1.setRPY(0, 0, -M_PI/2);
    // q=q*q1;
    // m_transform_for_camera.setRotation(q);

    m_listener.waitForTransform(m_robot_frame, camera_frame, ros::Time::now(), ros::Duration(20.0));
    //ROS_WARN("... done !");
    m_listener.lookupTransform(camera_frame,m_robot_frame, ros::Time(0), m_transform_robot_to_camera);

    //Camera Frame to Optical Frame
    //ROS_WARN("Wait for the /tf between %s and %s ...", optical_frame.c_str(), camera_frame.c_str());
    m_listener.waitForTransform(optical_frame, camera_frame, ros::Time::now(), ros::Duration(20.0));
    //ROS_WARN("... done !");
    m_listener.lookupTransform(optical_frame, camera_frame, ros::Time(0), transform);
    // float dist;
    // dist=transform.getOrigin().getX();
    // dist +2
    // Rotation between tf_child and tf_parent in Rodrigues convention
    double rot_mat[9] = {
        transform.getBasis().getColumn(0).getX(), transform.getBasis().getColumn(1).getX(), transform.getBasis().getColumn(2).getX(),
        transform.getBasis().getColumn(0).getY(), transform.getBasis().getColumn(1).getY(), transform.getBasis().getColumn(2).getY(),
        transform.getBasis().getColumn(0).getZ(), transform.getBasis().getColumn(1).getZ(), transform.getBasis().getColumn(2).getZ()};

    // Convert Rotation part into Mat(3,3) Rodrigues convention 
    Rodrigues((Mat(3,3,CV_64FC1,rot_mat)), m_tf_rotation);

    //ROS_WARN("m_tf_translation [0,0]: %f , %f , %f", transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());

    // Convert Translation part into Mat(1,3)
    m_tf_translation = Mat(1,3,CV_64FC1,{transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ()});
} 
