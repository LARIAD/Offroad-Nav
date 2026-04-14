#include <ros/ros.h>

// Basic
#include <iostream>
#include <string>
#include <math.h> 
#include <vector>
#include <algorithm>
#include <map>

// Custom
#include <mirador_msgs/Mission.h>
#include <barakuda_manager/SetIntRequest.h>
#include <barakuda_manager/SetIntResponse.h>
#include <barakuda_manager/SetInt.h>
#include <mirador_msgs/GeoPose.h>
#include <mirador_msgs/Status.h>

// Axis
#include <axis_msgs/Axis.h>

// ROS msgs
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <std_srvs/SetBool.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Quaternion.h>

#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPath.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <barakuda_manager/GeoPolygon.h>
#include <barakuda_manager/GeoPolygonArray.h>
#include <barakuda_manager/Polygon2.h>
#include <barakuda_manager/Polygon2Array.h>

// Action
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <barakuda_manager/WaypointAction.h>

#include <waypoint_manager/WaypointActionGoal.h>

#include <sbg_driver/SbgGpsPos.h>
#include <sbg_driver/SbgEkfNav.h>
#include <sbg_driver/SbgEkfQuat.h>
#include <sbg_driver/SbgEkfEuler.h>


// Geographic Lib
#include <GeographicLib/UTMUPS.hpp>


class BarakudaManager
{
    private:
        // node handler
        ros::NodeHandle m_node_handler;

        // ROS Topics
        std::string m_listening_topic;
        std::string m_alt_stop_topic;
        std::string m_exclusion_geopoly_topic;
        std::string m_imu_topic;
        std::string m_nav_sat_fix_topic;
        std::string m_mission_topic;
        std::string m_joy_topic;

        std::string m_UTM_frame;
        std::string m_map_frame;

        std::string m_camera_orientation_topic;

        std::string m_mission_context_topic;
        std::string m_red_detect_topic;
        
        // Variables
        std::string m_utm_frame_id;
        int m_utm_zone;
        bool m_is_north;
        bool m_is_NED;
        int m_max_record_length;
        float m_dist_to_previous_record;
        float m_dist_record_tolerance;

        bool m_gnss_record; 

        actionlib_msgs::GoalID m_curent_move_base_goal_id;
        actionlib_msgs::GoalID m_curent_waypoint_goal_id;

        // Enum and array witht the different stats of the robot
        enum m_robot_state_enum {NONE = 0, TELEOPERATED = 1, AUTONOMOUS = 2, PATH = 3, RETURN = 4, REPLAY = 5, BORDER = 6};
        std::string robot_state_array[7] = {"NONE", "TELEOPERATED","AUTONOMOUS", "PATH", "RETURN", "REPLAY", "BORDER"};

        std_msgs::UInt8 m_robot_state;
        sensor_msgs::Imu m_imu;
        sensor_msgs::NavSatFix m_nav_sat_fix;
        barakuda_manager::GeoPolygonArray m_exclusion_geopoly;
        barakuda_manager::Polygon2Array m_exclusion_poly;
        mirador_msgs::Status m_status;

        std_msgs::UInt8 m_gnssStatus;

        std_msgs::Float32 m_barakudaBattery;
        std_msgs::Float32 m_taurusBattery;

        std_msgs::Float32MultiArray m_cmd_motors;

        // Published path messages 
        nav_msgs::Path m_pose_record_array;
        nav_msgs::Path m_path;
        nav_msgs::Path m_current_path;

        // Array to store localy the gnss positions and the UTM projection
        std::vector<geographic_msgs::GeoPoint> m_gnss_record_vector;
        std::vector<geometry_msgs::PoseStamped> m_pose_record_vector;
        
        // Array to store localy the UTM position of the mission 
        std::vector<geometry_msgs::PoseStamped> m_pose_mission_vector;

        // Camera Stream messages
        std::vector<std::string> m_stream_port;
        std::vector<std::string> m_stream_enable;

        // CoHoMa mission infos
        mirador_msgs::MissionContext m_mission_context;
        std_msgs::Bool m_red_detect;

        // Camera Info
        axis_msgs::Axis m_camera_orientation;
        int m_camera_fov;

        // Safety variables
        std_msgs::Bool m_miradorStop;
        std_msgs::Bool m_joyStop;
        std_msgs::Bool m_closeRangeStop;
        std_msgs::Bool m_sharkStationStop;
        std_msgs::Bool m_sharkStationListening;
        std_msgs::Bool m_managerStop;

        std_msgs::String m_stopPccDisplay;
        std_msgs::String m_navPccDisplay;
        
        // Service variables
        barakuda_manager::SetInt m_set_int_srv;
        std_srvs::SetBool m_set_bool_srv;

        // TF
        tf::StampedTransform m_map_in_UTM;
        tf::StampedTransform m_goal_in_UTM;
        tf::StampedTransform m_goal_in_map;

        tf2_ros::Buffer m_tf2_buffer;
        tf2_ros::TransformListener m_tf2_listener;
        tf::TransformListener m_tf_listener;
        tf::TransformBroadcaster br;

        // Subscribers
        ros::Subscriber m_exclusionGeopolySubscriber;
        ros::Subscriber m_imuSubscriber;
        ros::Subscriber m_locationSubscriber;
        ros::Subscriber m_locationSelectorSubscriber;
        ros::Subscriber m_missionSubscriber;
        ros::Subscriber m_joySubscriber;
        ros::Subscriber m_moveBaseGoalIdSubscriber;
        ros::Subscriber m_waypointGoalIdSubscriber;
        ros::Subscriber m_gnssStatusSubscriber;
        ros::Subscriber m_ekfEulerSubscriber;
        ros::Subscriber m_imuOdometrySubscriber;
        ros::Subscriber m_miradorModeSwapSubscriber;
        ros::Subscriber m_barakudaBatterySubscriber;
        ros::Subscriber m_taurusBatterySubscriber;
        ros::Subscriber m_recordSubscriber;
        ros::Subscriber m_replaySubscriber;
        ros::Subscriber m_returnSubscriber;

        ros::Subscriber m_cameraOrientationSubscriber;
        ros::Subscriber m_mission_contextSubscriber;
        ros::Subscriber m_red_detectSubscriber;

        ros::Subscriber m_moveBaseStatusSubscriber;
        ros::Subscriber m_emergencyStopSubscriber;
        ros::Subscriber m_miradorStopSubscriber;
        ros::Subscriber m_miradorLaunchSubscriber;
        ros::Subscriber m_joyStopSubscriber;
        ros::Subscriber m_closeRangeStopSubscriber;
        ros::Subscriber m_sharkStationStopSubscriber;
        ros::Subscriber m_sharkStationListeningSubscriber;

        
        // Publisher
        ros::Publisher m_exclusionPolyPublisher;
        ros::Publisher m_pathPublisher;
        ros::Publisher m_posePublisher;
        ros::Publisher m_joyPublisher;
        ros::Publisher m_miradorStatusPublisher;
        ros::Publisher m_moveBaseCancelPublisher;
        ros::Publisher m_waypointCancelPublisher;

        ros::Publisher m_robotStatePublisher;
        ros::Publisher m_recordStatePublisher;
        ros::Publisher m_robotPosePublisher;

        ros::Publisher m_managerStopPublisher;
        ros::Publisher m_stopPccDisplayPublisher;
        ros::Publisher m_navPccDisplayPublisher;
        ros::Publisher m_gnssStatusPccDisplayPublisher;
        
        // Service Clients
        ros::ServiceClient m_robotStateUpdateClient;
        ros::ServiceClient m_emergencyStopClient;

        // Service Servers
        ros::ServiceServer m_robotStateUpdateServer ;
        ros::ServiceServer m_poseRecordUpdateServer;
        
        // Action Clients 
        actionlib::SimpleActionClient<barakuda_manager::WaypointAction> wp_client;
        //actionlib::SimpleClientGoalState wp_state;

    public:
        BarakudaManager(ros::NodeHandle& n);

        // ============= Callbacks
        void exclusionGeopolyCallback(const barakuda_manager::GeoPolygonArray& _exclusion_geopoly);
        void imuCallback(const sensor_msgs::Imu& _imu);

        // Multiple source of lat/lng for mirador
        void inline updateHeadingUsingIMU();
        void initLocationSelector();
        void locationSelectorCallback(const std_msgs::String& _msg);
        void navSatFixCallback(const sensor_msgs::NavSatFix& _nav_sat_fix);
        void sbgEkfNavCallback(const sbg_driver::SbgEkfNav& _nav_sat_fix);
        void odometryCallback(const nav_msgs::Odometry& _odom);

        void missionCallback(const mirador_msgs::Mission& _mission);
        void joyCallback(const sensor_msgs::Joy& _joy);
        void moveBaseGoalIdCallback(const move_base_msgs::MoveBaseActionGoal& _goal);
        void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray& _status);
        void waypointGoalIdCallback(const waypoint_manager::WaypointActionGoal& _goal);
        void gnssStatusCallback(const sbg_driver::SbgGpsPos& _status);
        void ekfEulerCallback(const sbg_driver::SbgEkfEuler& ekf_euler);
        void imuOdometryCallback(const nav_msgs::Odometry& _odom);
        void miradorModeSwapCallback(const std_msgs::Empty& _empty);
        void barakudaBatteryCallback(const std_msgs::Float32& _batt);
        void recordCallback(const std_msgs::Empty& _empty);
        void replayCallback(const std_msgs::Empty& _empty);
        void returnCallback(const std_msgs::Empty& _empty);

        void miradorStopCallback(const std_msgs::Empty& _empty);
        void miradorLaunchCallback(const std_msgs::Empty& _empty);
        void joyStopCallback(const std_msgs::Bool& _bool);
        void closeRangeStopCallback(const std_msgs::Bool& _bool);
        void sharkStationStopCallback(const std_msgs::Bool& _bool);
        void sharkStationListeningCallback(const std_msgs::Bool& _bool);


        void cameraOrientationCallback(const axis_msgs::Axis& _camera_orientation);
        void missionContextCallback(const mirador_msgs::MissionContext& _mission_context);
        void redDetectCallback(const std_msgs::Bool& _red_detect);
        
        // Service Servers
        bool robotStateUpdateServer(barakuda_manager::SetInt::Request&  req, barakuda_manager::SetInt::Response& res);
        bool poseRecordUpdateServer(std_srvs::SetBool::Request&  req, std_srvs::SetBool::Response& res);

        // Service Clients
        void robotStateUpdateClient(long int state);
        void emergencyStopClient(const bool stop);
        void updateCurrentMission(nav_msgs::Path &path);

        // Functions
        void latLongToUtm(const geographic_msgs::GeoPoint& _geo_point, geometry_msgs::Point& utm_point);
        void latLongToUtm(const double& _latitude, const double& _longitude, const double& _altitude, geometry_msgs::Point& utm_point);
        void utmToLatLong(const geometry_msgs::Point& _utm_point, geographic_msgs::GeoPoint& geo_point) const;

        void tfInit(const std::string& UTM_frame, const std::string& map_frame);
        
        bool getGnssRecord();
        void gnssRecord();
        void resetRecord();
        void inverseRecord(const std::vector<geometry_msgs::PoseStamped>& m_pose_vector, std::vector<geometry_msgs::PoseStamped>& m_inverted_pose_vector);
        
        // Convert a GeoPoint (LL in UTM) to PointStamped (XY in target frame)
        geometry_msgs::PointStamped geoPointToPoint(geographic_msgs::GeoPoint geo_point, std::string &target_frame);

        /**
         * Convert a pose (XY in source frame) to a GeoPoint (LL) via UTM
         *
         * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
         * tf2::ExtrapolationException, tf2::InvalidArgumentException
         *
         * @param point source point
         * @return the corresponding latlong
         */
        geographic_msgs::GeoPoint pointToGeoPoint(geometry_msgs::PointStamped point);

        // Publish string that contains informations for the PCC to display
        void updatePccDisplay();

        // Send Path to the Waypoint Action Server that call the move_base Action Server
        void sendPath(nav_msgs::Path &path);

        // Called once when the goal completes
        void donePath(const actionlib::SimpleClientGoalState& state, const barakuda_manager::WaypointResultConstPtr& result);

        // Called once when the goal becomes active
        void activePath();

        // Called every time feedback is received for the goal
        void feedbackPath(const barakuda_manager::WaypointFeedbackConstPtr& feedback);

        // Cancel the Path given to the Action Server
        void cancelPath();;

        // Send Current path to the Action Server in case of iterruption
        void restartPath();

        // Set the emergency state to true and cancel the path
        void stopNavigation();
        
        // Send a new path minus the reached ones
        void resumeNavigation();

        // Update manager stop
        void updateManagerStop();

        // Publish Status
        void miradorStatusPublisher();
};
