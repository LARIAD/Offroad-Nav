#include "barakuda_manager.h"
#include <numeric>

BarakudaManager::BarakudaManager(ros::NodeHandle &n)
: wp_client("waypoint", true), m_tf2_listener(m_tf2_buffer)
{
    ros::NodeHandle private_n("~");
    m_node_handler=n;
    // ----- Params  ----- //
    private_n.param<std::string>("listening_topic", m_listening_topic, "/listening");
    private_n.param<std::string>("exclusion_geopoly_topic", m_exclusion_geopoly_topic, "/exclusion_geopoly");
    private_n.param<std::string>("imu_topic", m_imu_topic, "/imu/data");

    private_n.param<std::string>("mission_topic", m_mission_topic, "/mirador/mission");
    private_n.param<std::string>("joy_topic", m_joy_topic, "/mirador/joy");

    private_n.param<std::string>("alt_stop_topic", m_alt_stop_topic, "/alt_stop");

    private_n.param<int>("utm_zone", m_utm_zone, 31);
    private_n.param<bool>("is_north", m_is_north, true);

    private_n.param<std::string>("UTM_frame", m_UTM_frame, "UTM");
    private_n.param<std::string>("map_frame", m_map_frame, "odom");
    private_n.param<bool>("is_NED", m_is_NED, true);

    private_n.param<int>("max_record_length", m_max_record_length, 30);

    private_n.param<float>("m_dist_record_tolerance", m_dist_record_tolerance, 25.0);

    private_n.param<std::string>("camera_orientation_topic", m_camera_orientation_topic, "/axis/status");
    private_n.param<int>("camera_fov", m_camera_fov, 70);

    private_n.param<std::string>("mission_context_topic", m_mission_context_topic, "mission/mission_context");
    private_n.param<std::string>("red_detect_topic", m_red_detect_topic, "mission/red_detect");

    private_n.param<std::vector<std::string>>("stream_port", m_stream_port, std::vector<std::string>());
    // extract the enable topics from the rtp config
    XmlRpc::XmlRpcValue streams;
    if (private_n.getParam("streams", streams)) {
        for (XmlRpc::XmlRpcValue::iterator it = streams.begin(); it != streams.end(); ++it) {
            XmlRpc::XmlRpcValue& stream = it->second;
            if (stream.hasMember("enable_stream_topic")) {
                std::string topic = static_cast<std::string>(stream["enable_stream_topic"]);
                m_stream_enable.push_back(topic);
            }
        }
    }


    // ----- Subscribers ----- //
    m_exclusionGeopolySubscriber = n.subscribe(m_exclusion_geopoly_topic, 10, &BarakudaManager::exclusionGeopolyCallback, this);
    m_imuSubscriber = n.subscribe(m_imu_topic, 10, &BarakudaManager::imuCallback, this);
    m_ekfEulerSubscriber = n.subscribe("/sbg/ekf_euler", 10, &BarakudaManager::ekfEulerCallback, this);
    m_imuOdometrySubscriber = n.subscribe("/imu/odometry", 10, &BarakudaManager::imuOdometryCallback, this);
    m_missionSubscriber = n.subscribe(m_mission_topic, 10, &BarakudaManager::missionCallback, this);
    m_joySubscriber = n.subscribe(m_joy_topic, 10, &BarakudaManager::joyCallback, this);
    m_moveBaseGoalIdSubscriber = n.subscribe("/move_base/goal", 10, &BarakudaManager::moveBaseGoalIdCallback, this);
    m_moveBaseStatusSubscriber = n.subscribe("/move_base/status", 10, &BarakudaManager::moveBaseStatusCallback, this);
    m_waypointGoalIdSubscriber = n.subscribe("/waypoint/goal", 10, &BarakudaManager::waypointGoalIdCallback, this);
    m_gnssStatusSubscriber = n.subscribe("/sbg/gps_pos", 10, &BarakudaManager::gnssStatusCallback, this);
    m_barakudaBatterySubscriber = n.subscribe("/barakuda/battery", 10, &BarakudaManager::barakudaBatteryCallback, this);
    m_locationSelectorSubscriber = n.subscribe("/location/selector", 10, &BarakudaManager::locationSelectorCallback, this);
    initLocationSelector();

    // ----- Safety Subscribers ----- //
    m_joyStopSubscriber = n.subscribe("/safety/joy_stop", 10, &BarakudaManager::joyStopCallback, this);
    m_closeRangeStopSubscriber = n.subscribe("/safety/close_range_stop", 10, &BarakudaManager::closeRangeStopCallback, this);
    m_sharkStationStopSubscriber = n.subscribe("/safety/shark_station_stop", 10, &BarakudaManager::sharkStationStopCallback, this);
    m_sharkStationListeningSubscriber = n.subscribe("/safety/shark_station_listening", 10, &BarakudaManager::sharkStationListeningCallback, this);

    // ----- Mirador Subscribers ----- //
    m_miradorStopSubscriber = n.subscribe("/mirador/abort", 10, &BarakudaManager::miradorStopCallback, this);
    m_miradorLaunchSubscriber = n.subscribe("/mirador/launch", 10, &BarakudaManager::miradorLaunchCallback, this);
    m_miradorModeSwapSubscriber = n.subscribe("/mirador/swap_mode", 10, &BarakudaManager::miradorModeSwapCallback, this);
    m_recordSubscriber = n.subscribe("/mirador/record", 10, &BarakudaManager::recordCallback, this);
    m_replaySubscriber = n.subscribe("/mirador/replay", 10, &BarakudaManager::replayCallback, this);
    m_returnSubscriber = n.subscribe("/mirador/return", 10, &BarakudaManager::returnCallback, this);

    // -----  CoHoMa Subscribers ----- //

    m_mission_contextSubscriber = n.subscribe(m_mission_context_topic, 10, &BarakudaManager::missionContextCallback, this);
    m_red_detectSubscriber = n.subscribe(m_red_detect_topic, 10, &BarakudaManager::redDetectCallback, this);
    m_cameraOrientationSubscriber = n.subscribe(m_camera_orientation_topic, 10, &BarakudaManager::cameraOrientationCallback, this);

    // ----- Publisher ----- //
    m_exclusionPolyPublisher = n.advertise<barakuda_manager::Polygon2Array>("/manager/exclusion_poly", 10);
    m_pathPublisher = n.advertise<nav_msgs::Path>("/move_base_simple/trajectory_to_do", 10);
    m_posePublisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    m_joyPublisher = n.advertise<sensor_msgs::Joy>("/manager/joy", 10);
    m_miradorStatusPublisher = n.advertise<mirador_msgs::Status>("/mirador/status", 10);
    m_robotStatePublisher = n.advertise<std_msgs::UInt8>("/manager/robot_state", 10);
    m_recordStatePublisher = n.advertise<std_msgs::Bool>("/manager/record_state", 10);
    m_robotPosePublisher = n.advertise<geometry_msgs::PoseStamped>("/manager/robot_pose", 10);

    m_moveBaseCancelPublisher = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);
    m_waypointCancelPublisher = n.advertise<actionlib_msgs::GoalID>("/waypoint/cancel", 10);

    m_managerStopPublisher = n.advertise<std_msgs::Bool>("/safety/manager_stop", 10);
    m_stopPccDisplayPublisher = n.advertise<std_msgs::String>("/safety/stop_pcc_display", 10);
    m_navPccDisplayPublisher = n.advertise<std_msgs::String>("/safety/nav_pcc_display", 10);
    m_gnssStatusPccDisplayPublisher = n.advertise<std_msgs::UInt8>("/safety/gnss_status", 10);


    // ----- Services Server ----- //
    m_robotStateUpdateServer = n.advertiseService("/manager/robots_state_update", &BarakudaManager::robotStateUpdateServer, this);
    m_poseRecordUpdateServer = n.advertiseService("/manager/pose_record_update", &BarakudaManager::poseRecordUpdateServer, this);

    // ----- Services Client ----- //
    m_robotStateUpdateClient = n.serviceClient<barakuda_manager::SetInt>("/manager/robots_state_update");
    m_emergencyStopClient = n.serviceClient<std_srvs::SetBool>("/barakuda/stop");

    // Wait for the action server to start
    ROS_INFO("Waiting for waypoint action server to start ...");
    wp_client.waitForServer(ros::Duration(10));

    // ----- Variables init ----- //
    m_gnss_record = false;
    m_nav_sat_fix.latitude = 0;
    m_robot_state.data = 1;
    m_imu.orientation.w = 1;

    m_miradorStop.data = false;
    m_joyStop.data = false;
    m_closeRangeStop.data = false;
    m_sharkStationStop.data = false;
    m_sharkStationListening.data = false;
    m_managerStop.data = true;

    m_status = mirador_msgs::Status();

    m_status.camera_pitch = 0;
    m_status.camera_yaw = 0;
    m_status.camera_zoom = 1;

    m_status.camera_fov = m_camera_fov;
    m_status.mission_context = mirador_msgs::MissionContext();
    m_status.red_detect = false;

    m_status.stream_port=m_stream_port;
    m_status.stream_enable=m_stream_enable;

    // Publish a first message to update stop model state
    m_managerStopPublisher.publish(m_managerStop);

    // Wait for tf between UTM and Map
    tfInit(m_UTM_frame, m_map_frame);
}

// ----- Callbacks ----- //

void BarakudaManager::imuCallback(const sensor_msgs::Imu &_imu)
{
    m_imu = _imu;
}

void BarakudaManager::initLocationSelector() {
    ros::NodeHandle private_n("~");
    XmlRpc::XmlRpcValue location_topics;
    private_n.getParam("location_topics", location_topics);

    if (location_topics.getType() == XmlRpc::XmlRpcValue::TypeArray && location_topics.size() > 0) {
        ROS_INFO("Configured location sources are:");
        for (int i = 0; i < location_topics.size(); ++i) {
            if (location_topics[i].hasMember("topic") && location_topics[i].hasMember("type")) {
                std::string topic = static_cast<std::string>(location_topics[i]["topic"]);
                std::string type  = static_cast<std::string>(location_topics[i]["type"]);
                ROS_INFO("'%s' (%s)", topic.c_str(), type.c_str());
                if (i == 0) {
                    std_msgs::String msg;
                    msg.data = topic;
                    locationSelectorCallback(msg);
                }
            }
        }
    } else {
        // default behaviour if no config specified
        m_locationSubscriber = m_node_handler.subscribe("/imu/nav_sat_fix", 10, &BarakudaManager::navSatFixCallback, this);
    }
}

void BarakudaManager::locationSelectorCallback(const std_msgs::String &_msg) {
    const std::string topic = _msg.data;
    bool found = false;

    ros::NodeHandle private_n("~");
    XmlRpc::XmlRpcValue location_topics;
    private_n.getParam("location_topics", location_topics);
    for (int i = 0; i < location_topics.size(); ++i) {
        if (location_topics[i].hasMember("topic") && location_topics[i].hasMember("type")) {
            std::string _topic = static_cast<std::string>(location_topics[i]["topic"]);
            if (topic == _topic) {
                std::string type  = static_cast<std::string>(location_topics[i]["type"]);
                ROS_INFO("Switching to %s as location source.", topic.c_str());
                found = true;
                // unsubscribe from previous topic
                // (this should be unnecessary as changing the value of m_locationSubscriber should unsub)
                if (type == "sensor_msgs::NavSatFix") {
                    m_locationSubscriber.shutdown();
                    m_locationSubscriber = m_node_handler.subscribe(topic, 10, &BarakudaManager::navSatFixCallback, this);
                } else if (type == "sbg_driver::SbgEkfNav") {
                    m_locationSubscriber.shutdown();
                    m_locationSubscriber = m_node_handler.subscribe(topic, 10, &BarakudaManager::sbgEkfNavCallback, this);
                } else if (type == "nav_msgs::Odometry") {
                    m_locationSubscriber.shutdown();
                    m_locationSubscriber = m_node_handler.subscribe(topic, 10, &BarakudaManager::odometryCallback, this);
                } else {
                    ROS_INFO("Unknown message type source.");
                }
                break;
            }
        }
    }

    if (!found) {
        ROS_WARN("You must specified a configured topic.");
    }
}

void inline BarakudaManager::updateHeadingUsingIMU() {
    double siny_cosp = 2 * (m_imu.orientation.w * m_imu.orientation.z + m_imu.orientation.x * m_imu.orientation.y);
    double cosy_cosp = 1 - 2 * (m_imu.orientation.y * m_imu.orientation.y + m_imu.orientation.z * m_imu.orientation.z);
    m_status.pose.heading = atan2(siny_cosp, cosy_cosp) * 180 / M_PI; // yaw in deg
}

void BarakudaManager::navSatFixCallback(const sensor_msgs::NavSatFix& _nav_sat_fix)
{
    m_nav_sat_fix = _nav_sat_fix;

    // update mirador Status
    m_status.pose.longitude = _nav_sat_fix.longitude;
    m_status.pose.latitude = _nav_sat_fix.latitude;
    m_status.pose.altitude = _nav_sat_fix.altitude;

    updateHeadingUsingIMU();
}

void BarakudaManager::sbgEkfNavCallback(const sbg_driver::SbgEkfNav& _nav_sat_fix)
{
    m_nav_sat_fix.longitude = _nav_sat_fix.longitude;
    m_nav_sat_fix.latitude = _nav_sat_fix.latitude;
    m_nav_sat_fix.altitude = _nav_sat_fix.altitude;


    // update mirador Status
    m_status.pose.longitude = _nav_sat_fix.longitude;
    m_status.pose.latitude = _nav_sat_fix.latitude;
    m_status.pose.altitude = _nav_sat_fix.altitude;

    updateHeadingUsingIMU();
}

void BarakudaManager::odometryCallback(const nav_msgs::Odometry &_odom) {
    try {
        geometry_msgs::PointStamped point;
        point.header = _odom.header;
        point.point = _odom.pose.pose.position;

        const geographic_msgs::GeoPoint latLong = pointToGeoPoint(point);

        m_nav_sat_fix.longitude = latLong.longitude;
        m_nav_sat_fix.latitude = latLong.latitude;
        m_nav_sat_fix.altitude = latLong.altitude;

        // update mirador Status
        m_status.pose.longitude = latLong.longitude;
        m_status.pose.latitude = latLong.latitude;
        m_status.pose.altitude = latLong.altitude;

        updateHeadingUsingIMU();
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Exception while reprojecting location odometry msg:%s",ex.what());
    }
}


// Convert a LL_geopath vector into a XY_polygon vector
void BarakudaManager::exclusionGeopolyCallback(const barakuda_manager::GeoPolygonArray &_exclusion_geopoly)
{
    // m_exclusion_geopoly = _exclusion_geopoly;

    // Clear the previous data from the exclusion area
    m_exclusion_poly.zones.clear();

    // No need to define the the XY vector size depending on the LL vector because the new poly are psuhed back
    // m_exclusion_poly.zones.resize(_exclusion_geopoly.zones.size());

    ROS_INFO("Nb Exclusion Area: %ld", _exclusion_geopoly.zones.size());
    for (int i = 0; i < _exclusion_geopoly.zones.size(); i++)
    {

        // Create a new polygon and define its Points32 size
        barakuda_manager::Polygon2 poly;
        geometry_msgs::PointStamped point_stamped;
        geographic_msgs::GeoPoint geopoint;

        // Define the polygon size depending on the GeoPath size
        poly.zone.points.resize(_exclusion_geopoly.zones[i].zone.size());

        ROS_INFO("Exclusion area nb %d \n Number of points: %ld", i + 1, poly.zone.points.size());

        // Fill the poly with convert LL Geopoints
        for (int j = 0; j < poly.zone.points.size(); j++)
        {
            // Altitude
            //latLongToUtm(_exclusion_geopoly.zones[i].zone[j].latitude, _exclusion_geopoly.zones[i].zone[j].longitude, _exclusion_geopoly.zones[i].zone[j].altitude, poly.zone.points[j]);

            // Altitude forced to be 0
            geopoint.latitude = _exclusion_geopoly.zones[i].zone[j].latitude;
            geopoint.longitude = _exclusion_geopoly.zones[i].zone[j].longitude;
            geopoint.altitude = 0;

            point_stamped = geoPointToPoint(geopoint, m_map_frame);

            poly.zone.points[j].x = point_stamped.point.x;
            poly.zone.points[j].y = point_stamped.point.y;


            ROS_INFO("-- Point %d:", j + 1);
            ROS_INFO("Lat: %f; Long: %f", geopoint.latitude, geopoint.longitude);
            ROS_INFO("Norting: %f; Easting: %f", poly.zone.points[j].y, poly.zone.points[j].x);
        }

        // Assign zone id and zone type
        poly.zone_id = _exclusion_geopoly.zones[i].zone_id;
        poly.zone_type = _exclusion_geopoly.zones[i].zone_type;

        // Push back the polygon in the vector
        m_exclusion_poly.zones.push_back(poly);
    }

    m_exclusionPolyPublisher.publish(m_exclusion_poly);
}

// Convert Mirador mission into a mission understandable for the local planner
void BarakudaManager::missionCallback(const mirador_msgs::Mission &_mission)
{
    // Path callback
    //if (m_robot_state.data == 3 && _mission.type == 2) // Path mode
    if (m_robot_state.data == 2 && (_mission.type == 2 && _mission.points.size() > 1)) // Autonomous Mode
    {
        ROS_INFO("Mission call back with: %ld points", _mission.points.size());

        // Clear previous mission array
        m_pose_mission_vector.clear();

        geometry_msgs::PointStamped pointStamped_mission;
        geometry_msgs::PoseStamped pose_mission;

        // For each point translate LL to XY in map frame
        for (int i = 0; i < _mission.points.size(); i++)
        {
            pointStamped_mission = geoPointToPoint(_mission.points[i], m_map_frame);

            pose_mission.header.stamp = pointStamped_mission.header.stamp;
            pose_mission.header.frame_id = pointStamped_mission.header.frame_id;
            pose_mission.pose.position = pointStamped_mission.point;

            m_pose_mission_vector.push_back(pose_mission);
        }

        // For each point calculate the orientation

        // if (_mission.points.size() == 1)
        // { // Deal with the size = 1 case => Just give a correct quaternion

        //     // !!! -- TO DO: faire le même calcul de cap du point que pour l'envoie de point simple (orientation = vecteur(cap/robot)) -- !!!

        //     m_pose_mission_vector[0].pose.orientation = m_imu.orientation;
        // }
        // else
        // {
        for (int i = 0; i < _mission.points.size(); i++)
        {
            // Calculate the z axis angle between point n and n-1 to define the pose orientation
            if (i < _mission.points.size() - 1)
            {
                // Output in NED configuration with Z rotation increasing clockwise
                double yaw = atan2((m_pose_mission_vector[i + 1].pose.position.y - m_pose_mission_vector[i].pose.position.y),
                                   (m_pose_mission_vector[i + 1].pose.position.x - m_pose_mission_vector[i].pose.position.x));
                // ROS_INFO("Angle betwenn point %d and %d: %lf°", i, i+1 , 180*yaw/M_PI);

            // Quaternion translation with the pitch
                double cr = cos(0 * 0.5);
                double sr = 0; // sin(0 * 0.5);
                double cp = cos(0 * 0.5);
                double sp = 0; // sin(0 * 0.5);
                double cy = cos(yaw * 0.5);
                double sy = sin(yaw * 0.5);

                geometry_msgs::Quaternion q;
                q.w = cr * cp * cy + sr * sp * sy;
                q.x = sr * cp * cy - cr * sp * sy;
                q.y = cr * sp * cy + sr * cp * sy;
                q.z = cr * cp * sy - sr * sp * cy;

                m_pose_mission_vector[i].pose.orientation = q;

                // Set the orientation of the last point like the one before
                if (i == _mission.points.size() - 2)
                    m_pose_mission_vector[_mission.points.size() - 1].pose.orientation = m_pose_mission_vector[_mission.points.size() - 2].pose.orientation;
            }
        }
        // }

        m_path.header.stamp = ros::Time::now();
        m_path.poses = m_pose_mission_vector;

        // Use tropic to publish path
        //m_pathPublisher.publish(m_path);

        // Use Action Client to publish
        sendPath(m_path);
        ROS_INFO("Path published");
    }
    // Navgoal callback
    else if (m_robot_state.data == 2 && (_mission.type == 1 || (_mission.type == 2 && _mission.points.size() == 1)))
    {
        if (m_nav_sat_fix.latitude != 0)
        {
            // Get LL values for the goal and the robot
            geographic_msgs::GeoPoint robot_geoPoint, goal_geoPoint;
            goal_geoPoint = _mission.points[0];
            robot_geoPoint.latitude = m_nav_sat_fix.latitude;
            robot_geoPoint.longitude = m_nav_sat_fix.longitude;

            geometry_msgs::PointStamped goal_point, robot_point;

            goal_point = geoPointToPoint(goal_geoPoint, m_map_frame);
            robot_point = geoPointToPoint(robot_geoPoint, m_map_frame);

            // ENU yaw from robot to goal
            double yaw = atan2((goal_point.point.y - robot_point.point.y),
                               (goal_point.point.x - robot_point.point.x));

            ROS_INFO("-----------------------> Goal yaw MAP: %f", 180*yaw/M_PI);

            // Quaternion translation with the pitch
            double cr = cos(0 * 0.5);
            double sr = 0; // sin(0 * 0.5);
            double cp = cos(0 * 0.5);
            double sp = 0; // sin(0 * 0.5);
            double cy = cos(yaw * 0.5);
            double sy = sin(yaw * 0.5);

            geometry_msgs::Quaternion q;
            q.w = cr * cp * cy + sr * sp * sy;
            q.x = sr * cp * cy - cr * sp * sy;
            q.y = cr * sp * cy + sr * cp * sy;
            q.z = cr * cp * sy - sr * sp * cy;

            // Create the Goal PoseStamped
            geometry_msgs::PoseStamped goal_pose;
            goal_pose.header.stamp = ros::Time::now();
            goal_pose.header.frame_id = m_map_frame;
            goal_pose.pose.orientation = q;
            goal_pose.pose.position = goal_point.point;

            // Create a path with one PoseStamped
            m_path.poses.clear();
            m_path.header.stamp = ros::Time::now();
            m_path.header.frame_id = m_map_frame;
            m_path.poses.push_back(goal_pose);

            // Use tropic to publish path
            //m_posePublisher.publish(m_path);

            // Use Action Client to publish path
            sendPath(m_path);
            ROS_INFO("New pose send");
        }
        else
        {
            ROS_WARN("Waiting for the IMU to be published in order to publish pose ...");
        }
    }
    else if (_mission.type == 3)
    {
        // Create Geopoly
        barakuda_manager::GeoPolygon geopoly;
        geopoly.zone = _mission.points;
        geopoly.zone_id = 0;
        geopoly.zone_type = 1;

        // Create an array of a single GeoPoly
        barakuda_manager::GeoPolygonArray geopoly_array;
        geopoly_array.zones.clear();
        geopoly_array.zones.push_back(geopoly);

        // Call the callback to create the new layer
        exclusionGeopolyCallback(geopoly_array);
    }
    // Else Warn the user
    else
    {
        ROS_WARN("Can't send the mission, Robot is not in the correct mode !");
    }
}

// Joy callback and cmd_vel conversion
void BarakudaManager::joyCallback(const sensor_msgs::Joy &_joy)
{
    if (m_robot_state.data == 1)
    {
        m_joyPublisher.publish(_joy);
    }
}

// Get the last move_base goal ID
void BarakudaManager::moveBaseGoalIdCallback(const move_base_msgs::MoveBaseActionGoal& _goal)
{
    m_curent_move_base_goal_id = _goal.goal_id;
    ROS_INFO("Current Move Base Goal ID %s", (m_curent_move_base_goal_id.id).c_str());
}

void BarakudaManager::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray& _status)
{
    // Concatenante strings

    for(int i=0; i<_status.status_list.size(); i++)
    {
        m_navPccDisplay.data += _status.status_list[i].text;
        if(i != (_status.status_list.size()-1)) m_navPccDisplay.data += " | ";
    }

    //m_navPccDisplayPublisher.publish(m_navPccDisplay);
}

// Get the last waypoint goal ID
void BarakudaManager::waypointGoalIdCallback(const waypoint_manager::WaypointActionGoal& _goal)
{
    m_curent_waypoint_goal_id = _goal.goal_id;
    ROS_INFO("Current Waypoint Goal ID %s", (m_curent_waypoint_goal_id.id).c_str());
}

// Get the Gnss/RTK status
void BarakudaManager::gnssStatusCallback(const sbg_driver::SbgGpsPos& _status)
{
    // 0 NO_SOLUTION No valid solution available.
    // 1 UNKNOWN_TYPE An unknown solution type has been computed.
    // 2 SINGLE Single point solution position.
    // 3 PSRDIFF Standard Pseudorange Differential Solution (DGPS).
    // 4 SBAS SBAS satellite used for differential corrections.
    // 5 OMNISTAR Omnistar VBS Position (L1 sub-meter).
    // 6 RTK_FLOAT Floating RTK ambiguity solution (20 cms RTK).
    // 7 RTK_INT Integer RTK ambiguity solution (2 cms RTK).

    m_gnssStatus.data = _status.status.type;
    m_gnssStatusPccDisplayPublisher.publish(m_gnssStatus);

}

void BarakudaManager::ekfEulerCallback(const sbg_driver::SbgEkfEuler &ekf_euler) {
    m_status.robot_angle = ekf_euler.angle;
}

void BarakudaManager::imuOdometryCallback(const nav_msgs::Odometry &odom) {
    m_status.robot_twist = odom.twist.twist;
}

void BarakudaManager::miradorModeSwapCallback(const std_msgs::Empty& _empty)
{
    if(m_robot_state.data == 1)
    {
        m_robot_state.data = 2;
        m_status.flight_status = 2;
    }
    else if(m_robot_state.data == 2)
    {
        m_robot_state.data = 1;
        m_status.flight_status = 1;
    }
    else
    {
        m_robot_state.data = 1;
        m_status.flight_status = 1;
    }

    ROS_INFO("Barakuda mode is set to : %s", (robot_state_array[m_robot_state.data]).c_str());
}

// ----- Safety Callbacks ----- //

// Call the Mirador Emergency Stop function
void BarakudaManager::miradorStopCallback(const std_msgs::Empty& _empty)
{
    ROS_INFO("c'est la");
    m_miradorStop.data = true;
    stopNavigation();
    updateManagerStop();
}

// Call the Mirador launch function
void BarakudaManager::miradorLaunchCallback(const std_msgs::Empty& _empty)
{
    m_miradorStop.data = false;
    resumeNavigation();
    updateManagerStop();
}

// Joy Stop Callback
void BarakudaManager::joyStopCallback(const std_msgs::Bool& _bool)
{
   m_joyStop = _bool;
   updateManagerStop();
}

// Close range scanner Stop Callback
void BarakudaManager::closeRangeStopCallback(const std_msgs::Bool& _bool)
{
    m_closeRangeStop = _bool;
    updateManagerStop();
}

// Shark Station Stop Callback
void BarakudaManager::sharkStationStopCallback(const std_msgs::Bool& _bool)
{
    m_sharkStationStop = _bool;
    updateManagerStop();
}

// Shark Station Listening Callback
void BarakudaManager::sharkStationListeningCallback(const std_msgs::Bool& _bool)
{
    m_sharkStationListening = _bool;
    updateManagerStop();
}

// Barakuda battery Callback
void BarakudaManager::barakudaBatteryCallback(const std_msgs::Float32& _batt)
{
    m_barakudaBattery = _batt;
    m_status.batteries = {m_barakudaBattery.data, -1., -1.};
}

// Taurus battery Callback
void BarakudaManager::recordCallback(const std_msgs::Empty& _batt)
{
    if(m_gnss_record == true) m_gnss_record = false; else m_gnss_record = true;
}

void BarakudaManager::replayCallback(const std_msgs::Empty& _batt)
{
    m_robot_state.data = 5;
    m_status.flight_status = 5;
    // Publish the recorded poses
    if (m_pose_record_array.poses.size() > 0)
    {
        // Set the gnss recording to false when REPLAY so it doesn't update the array
        m_gnss_record = false;
        // Use Action to publish path
        sendPath(m_pose_record_array);

        // Use Topic to publish path
        m_pathPublisher.publish(m_pose_record_array);
        ROS_INFO("Following the recorded path... ");
    }
    // Do nothing and go back to NONE state
    else
    {
        ROS_WARN("Can't REPLAY, record array is empty");
        m_robot_state.data = 0;
    }
}


void BarakudaManager::returnCallback(const std_msgs::Empty& _batt)
{
    m_robot_state.data = 4;
    m_status.flight_status = 4;
    // The record array is inverted and the orientations are updated
    if (m_pose_record_array.poses.size() >= 2)
    {
        // Set the gnss recording to false when RETURN
        m_gnss_record = false;

        // Invert the recorded path
        nav_msgs::Path return_path;
        inverseRecord(m_pose_record_array.poses, return_path.poses);

        // Use Action to publish return path
        sendPath(return_path);

        // Use Topic to publish return path
        //m_pathPublisher.publish(return_path);

        ROS_INFO("Following the inverted recorded path... ");
    }
    // If the record array has a size of one, it is not inverted
    else if (m_pose_record_array.poses.size() == 1)
    {
        // Use Action to publish path
        sendPath(m_pose_record_array);

        // Use Topic to publish path
        m_pathPublisher.publish(m_pose_record_array);
        ROS_INFO("Following the recorded position... ");
    }
    // Do nothing and go back to NONE state
    else
    {
        ROS_WARN("Can't RETURN, record array is empty");
        m_robot_state.data = 0;
    }
}

// CoHoMa Callback

void BarakudaManager::cameraOrientationCallback(const axis_msgs::Axis& _camera_orientation)
{
    m_camera_orientation =  _camera_orientation;
    m_status.camera_pitch = m_camera_orientation.tilt;
    m_status.camera_yaw = m_camera_orientation.pan;
    m_status.camera_zoom = m_camera_orientation.zoom;
    m_status.camera_fov = m_camera_fov;
}

void BarakudaManager::missionContextCallback(const mirador_msgs::MissionContext& _mission_context)
{
    m_mission_context = _mission_context;
    m_status.mission_context = m_mission_context;
}

void BarakudaManager::redDetectCallback(const std_msgs::Bool& _red_detect)
{
    m_red_detect = _red_detect;
    m_status.red_detect = _red_detect.data;
}


// ----- Services Server ----- //

// Update the robot state
bool BarakudaManager::robotStateUpdateServer(barakuda_manager::SetInt::Request &req, barakuda_manager::SetInt::Response &res)
{
    if (req.data < sizeof(robot_state_array) / sizeof(robot_state_array[0]))
    {
        m_robot_state.data = req.data;
        // Update mirador status
        m_status.flight_status = m_robot_state.data;

        ROS_INFO("Barakuda mode is set to : %s", (robot_state_array[m_robot_state.data]).c_str());

        res.success = true;
        res.message = "Barakuda mode is set to: " + robot_state_array[m_robot_state.data];
    }
    else
    {
        ROS_WARN("Barakuda mode invalid");
        res.success = false;
        res.message = "Barakuda mode invalid";
    }

    // Call functions depending on the Robot state
    switch (m_robot_state.data)
    {
    // TELEOPERATED
    case 1:
    {
        // Cancel any published path or goal before the switch
        nav_msgs::Path empty_path;

        // Send Empty path using topic
        //m_pathPublisher.publish(empty_path);

        // Send Empty path using Action
        sendPath(empty_path);

        ROS_INFO("Switch to teleoperated mode... ");
    }

    break;

    // RETURN
    case 4:
        // The record array is inverted and the orientations are updated
        if (m_pose_record_array.poses.size() >= 2)
        {
            // Set the gnss recording to false when RETURN
            m_gnss_record = false;

            // Invert the recorded path
            nav_msgs::Path return_path;
            inverseRecord(m_pose_record_array.poses, return_path.poses);

            // Use Action to publish return path
            sendPath(return_path);

            // Use Topic to publish return path
            //m_pathPublisher.publish(return_path);

            ROS_INFO("Following the inverted recorded path... ");
        }
        // If the record array has a size of one, it is not inverted
        else if (m_pose_record_array.poses.size() == 1)
        {
            // Use Action to publish path
            sendPath(m_pose_record_array);

            // Use Topic to publish path
            m_pathPublisher.publish(m_pose_record_array);
            ROS_INFO("Following the recorded position... ");
        }
        // Do nothing and go back to NONE state
        else
        {
            ROS_WARN("Can't RETURN, record array is empty");
            m_robot_state.data = 0;
        }
        break;

    // REPLAY
    case 5:
        // Publish the recorded poses
        if (m_pose_record_array.poses.size() > 0)
        {
            // Set the gnss recording to false when REPLAY so it doesn't update the array
            m_gnss_record = false;
            // Use Action to publish path
            sendPath(m_pose_record_array);

            // Use Topic to publish path
            m_pathPublisher.publish(m_pose_record_array);
            ROS_INFO("Following the recorded path... ");
        }
        // Do nothing and go back to NONE state
        else
        {
            ROS_WARN("Can't REPLAY, record array is empty");
            m_robot_state.data = 0;
        }
        break;

    default:
        break;
    }

    // Publish robot state
    m_robotStatePublisher.publish(m_robot_state);

    return true;
}

// Update the record state
bool BarakudaManager::poseRecordUpdateServer(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    m_gnss_record = req.data;
    ROS_INFO("Record mode is set to : %s", req.data ? "true" : "false");

    res.success = true;
    res.message = "Record mode is set to: " + std::string(req.data ? "true" : "false");

    // Clear the previous local record array when enabling record
    if (m_pose_record_vector.size() != 0)
    {
        m_gnss_record_vector.clear();
        m_pose_record_vector.clear();
        ROS_INFO("Previous record cleared");
    }

    // Publish record state
    m_recordStatePublisher.publish(req);

    return true;
}

// Services Clients

void BarakudaManager::robotStateUpdateClient(long int state)
{
    // Call the service
    std_msgs::Int8 plint8;
    plint8.data = state;
    m_set_int_srv.request.data = state;
    if (m_robotStateUpdateClient.call(m_set_int_srv))
        ROS_INFO("State update call success");
    else
        ROS_ERROR("State update fail");
}

void BarakudaManager::emergencyStopClient(const bool stop)
{
    // Call the service
    m_set_bool_srv.request.data = stop;
    if (m_emergencyStopClient.call(m_set_bool_srv) == false) ROS_ERROR("Emergency update failed");
}

void BarakudaManager::updateCurrentMission(nav_msgs::Path &path)
{
    if (path.poses.empty()) {
        m_status.mission.id = "";
    } else {
        m_status.mission.header = path.header;
        m_status.mission.id = "current_mission";
        m_status.mission.type = 2;
        m_status.mission.points = std::vector<geographic_msgs::GeoPoint>(path.poses.size());

        for (int i = 0; i < path.poses.size(); i++) {
            geometry_msgs::PointStamped in;
            in.header = path.poses[i].header;
            in.point.x = path.poses[i].pose.position.x;
            in.point.y = path.poses[i].pose.position.y;
            m_status.mission.points[i] = pointToGeoPoint(in);
        }
    }
}

// ----- Functions ----- //

// GeoPoint => UTM Pointint(
void BarakudaManager::latLongToUtm(const geographic_msgs::GeoPoint &_geo_point, geometry_msgs::Point &utm_point)
{
    double x, y;
    GeographicLib::UTMUPS::Forward(_geo_point.latitude, _geo_point.longitude, m_utm_zone, m_is_north, x, y);
    utm_point.x = x;
    utm_point.y = y;
    utm_point.z = _geo_point.altitude;
}

// LLA => UTM Point
void BarakudaManager::latLongToUtm(const double &_latitude, const double &_longitude, const double &_altitude, geometry_msgs::Point &utm_point)
{
    double x, y;
    GeographicLib::UTMUPS::Forward(_latitude, _longitude, m_utm_zone, m_is_north, x, y);
    utm_point.x = x;
    utm_point.y = y;
    utm_point.z = _altitude;
}

// Point UTM => GeoPoint
void BarakudaManager::utmToLatLong(const geometry_msgs::Point &_utm_point, geographic_msgs::GeoPoint &geo_point) const
{
    GeographicLib::UTMUPS::Reverse(m_utm_zone, m_is_north, _utm_point.x, _utm_point.y, geo_point.latitude, geo_point.longitude);
    geo_point.altitude = _utm_point.z;
}

// Init the tf between UTM and odom/map
void BarakudaManager::tfInit(const std::string &UTM_frame, const std::string &map_frame)
{
    // UTM frame to odom frame
    while(m_tf_listener.waitForTransform(UTM_frame, map_frame, ros::Time::now(), ros::Duration(1.0)) == false)
	{
		ROS_WARN("Wait for the /tf between %s and %s ...", UTM_frame.c_str(), map_frame.c_str());
	}

    m_tf_listener.lookupTransform(UTM_frame, map_frame, ros::Time(0), m_map_in_UTM);

    ROS_WARN("... done !");
    ROS_INFO("TF translation between %s and %s is: x: %f | y: %f", UTM_frame.c_str(), map_frame.c_str(), m_map_in_UTM.getOrigin().getX() , m_map_in_UTM.getOrigin().getY());
}

// GNSS Record getter
bool BarakudaManager::getGnssRecord()
{
    return m_gnss_record;
}

// Record in an array the n last robot position
void BarakudaManager::gnssRecord()
{
    // If the record is on, we will create a list of GeoPoints which is increased each seconds
    if (m_gnss_record)
    {
        // record only if the IMU is on
        if (m_nav_sat_fix.latitude != 0)
        {
            if(true ) { //m_gnss_record_vector.size() == 0){
                try
                {
                    geographic_msgs::GeoPoint geopoint_record;
                    geopoint_record.latitude = m_nav_sat_fix.latitude;
                    geopoint_record.longitude = m_nav_sat_fix.longitude;
                    geopoint_record.altitude = m_nav_sat_fix.altitude;

                    // If the record time is bigger than the limit, then the first item of the list is removed
                    if (m_gnss_record_vector.size() > m_max_record_length)
                        m_gnss_record_vector.erase(m_gnss_record_vector.begin());

                    // Convert LL GeoPoint into XY map point
                    geometry_msgs::PointStamped pointStamped_record = geoPointToPoint(geopoint_record, m_map_frame);
                    geometry_msgs::PoseStamped pose_record;
                    // Header
                    pose_record.header.seq = m_pose_record_vector.size();
                    pose_record.header.stamp = ros::Time::now();
                    pose_record.header.frame_id = m_map_frame;
                    // Pose
                    pose_record.pose.position = pointStamped_record.point;
                    pose_record.pose.orientation = m_imu.orientation;

                    // If the record time is bigger than the limit, then the first item of the list is removed
                    if (m_pose_record_vector.size() > m_max_record_length)
                        m_pose_record_vector.erase(m_pose_record_vector.begin());

                    // Calculate the distance difference between the previous recorded position and th actual one
                    if(m_pose_record_vector.size() != 0) m_dist_to_previous_record = std::sqrt(std::pow(m_pose_record_vector[m_pose_record_vector.size() -1].pose.position.x - pointStamped_record.point.x, 2) + std::pow(m_pose_record_vector[m_pose_record_vector.size() -1].pose.position.x  -  pointStamped_record.point.y, 2));

                    // If the distance if bigger than the tolerance then we update the new one
                    if(m_pose_record_vector.size() == 0 || m_dist_to_previous_record > m_dist_record_tolerance)
                    {
                        // Push back the last LL in UTM geopoint recorded
                        m_gnss_record_vector.push_back(geopoint_record);

                        // Push back the last point recorded and converted in XY map
                        m_pose_record_vector.push_back(pose_record);
                        m_pose_record_array.header.stamp = ros::Time::now();
                        m_pose_record_array.poses = m_pose_record_vector;
                        ROS_INFO("New position recorded");
                    }
                }
                catch (ros::Exception &e)
                {
                    ROS_ERROR("Error occured: %s ", e.what());
                }


            }
        }
        else
        {
            ROS_INFO("Waiting for the IMU to be published in order to record ...");
        }
    }
}

// Reset the record variables
void BarakudaManager::resetRecord(){
    m_gnss_record_vector.empty();
    m_pose_record_vector.empty();
    m_pose_record_array = nav_msgs::Path();
}
// Inverse an array the n last robot poses
void BarakudaManager::inverseRecord(const std::vector<geometry_msgs::PoseStamped> &m_pose_vector, std::vector<geometry_msgs::PoseStamped> &m_inverted_pose_vector)
{
    // Invert only if the vector has at least 2 values
    if (m_pose_vector.size() > 1)
    {
        // Inverse the pose vector using the magical properties of **vector**
        m_inverted_pose_vector = m_pose_vector;
        std::reverse(m_inverted_pose_vector.begin(), m_inverted_pose_vector.end());

        if (m_inverted_pose_vector.size() == 1)
        { // Deal with the size = 1 case => Just give a correct quaternion
            m_inverted_pose_vector[0].pose.orientation.w = 1;
        }
        else
        {
            for (int i = 0; i < m_inverted_pose_vector.size(); i++)
            {
                // Calculate the z axis angle between point n and n-1 to define the pose orientation
                if (i < m_inverted_pose_vector.size() - 1)
                {
                    // Output in NED configuration with Z rotation increasing clockwise
                    double yaw = atan2((m_inverted_pose_vector[i + 1].pose.position.y - m_inverted_pose_vector[i].pose.position.y),
                                       (m_inverted_pose_vector[i + 1].pose.position.x - m_inverted_pose_vector[i].pose.position.x));
                    // ROS_INFO("Angle betwenn point %d and %d: %lf°", i, i+1 , 180*yaw/M_PI);

                    // Quaternion translation with the pitch
                    double cr = cos(0 * 0.5);
                    double sr = 0; // sin(0 * 0.5);
                    double cp = cos(0 * 0.5);
                    double sp = 0; // sin(0 * 0.5);
                    double cy = cos(yaw * 0.5);
                    double sy = sin(yaw * 0.5);

                    geometry_msgs::Quaternion q;
                    q.w = cr * cp * cy + sr * sp * sy;
                    q.x = sr * cp * cy - cr * sp * sy;
                    q.y = cr * sp * cy + sr * cp * sy;
                    q.z = cr * cp * sy - sr * sp * cy;

                    m_inverted_pose_vector[i].pose.orientation = q;
                    // ROS_INFO("Quaternion:");
                    // ROS_INFO("-->  x: %lf  y: %lf  z: %lf  w: %lf ", q.x, q.y, q.z, q.w);

                    // Set the orientation of the last point like the one before
                    if (i == m_inverted_pose_vector.size() - 2)
                        m_inverted_pose_vector[m_inverted_pose_vector.size() - 1].pose.orientation = m_inverted_pose_vector[m_inverted_pose_vector.size() - 2].pose.orientation;
                }
            }
        }
        ROS_INFO("Pose vector inverted");
    }
    else
    {
        ROS_INFO("Can't inverse this array, need to have at least 2 values (here %ld)...", m_pose_vector.size());
    }
}

// Convert a GeoPoint (LL in UTM) to PointStamped (XY in target frame)
geometry_msgs::PointStamped BarakudaManager::geoPointToPoint(geographic_msgs::GeoPoint geo_point, std::string &target_frame)
{
    //ROS_INFO("UTM frame: %s, map frame %s", m_UTM_frame.c_str(), target_frame.c_str());

    geometry_msgs::Point point;
    geometry_msgs::PoseStamped pose;

    // Convert LL to XY UTM
    latLongToUtm(geo_point, point);

    // Convert Point32 into Point
    // for the robot
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;


    // Convert pose from XY UTM frame to XY target_frame
    geometry_msgs::PointStamped in, out;
    in.header.stamp = ros::Time::now();
    in.header.frame_id = m_UTM_frame;
    in.point = pose.pose.position;
    //ROS_INFO("In x: %f, y: %f", in.point.x, in.point.y);
    m_tf_listener.transformPoint(target_frame, in, out);
    //ROS_INFO("Out x: %f, y: %f", out.point.x, out.point.y);
    out.point.z = 0;
    return out;
}

geographic_msgs::GeoPoint BarakudaManager::pointToGeoPoint(geometry_msgs::PointStamped point) {
    const geometry_msgs::TransformStamped transformStamped = m_tf2_buffer.lookupTransform(
       m_UTM_frame,
       point.header.frame_id,
       ros::Time::now(),
       ros::Duration(.2));
    geometry_msgs::PointStamped out;
    tf2::doTransform(point, out, transformStamped);
    geographic_msgs::GeoPoint latLong;
    utmToLatLong(out.point, latLong);
    return latLong;
}

// ----- Navigation Functions ----- //

// Send Path to the Waypoint Action Server that call the move_base Action Server
void BarakudaManager::sendPath(nav_msgs::Path &path)
{
    // Cancelling Goal if one is currently going
    actionlib::SimpleClientGoalState wp_state = wp_client.getState();
    ROS_INFO("Action Status: %s", wp_state.toString().c_str());

    if(wp_state == actionlib::SimpleClientGoalState::ACTIVE)
    {
        ROS_INFO("Action currently running ... ");
        cancelPath();
        ROS_INFO("... Action canceled !!!!");
    }

    barakuda_manager::WaypointGoal goal;
    goal.path = path;
    m_current_path = path;
    updateCurrentMission(m_current_path);

    // Send a goal to the waypoint action
    ROS_INFO("Sending path...");

    wp_client.sendGoal(goal,
                boost::bind(&BarakudaManager::donePath, this, _1, _2),  // Callback lors de la fin de l'action
                boost::bind(&BarakudaManager::activePath, this),        // Callback lorsque l'action devient active
                boost::bind(&BarakudaManager::feedbackPath, this, _1)); // Callback à chaque feedback
}

// Called once when the goal completes
void BarakudaManager::donePath(const actionlib::SimpleClientGoalState& state, const barakuda_manager::WaypointResultConstPtr& result)
{
    ROS_INFO("Waypoint Client Done");
}

// Called once when the goal becomes active
void BarakudaManager::activePath()
{
    ROS_INFO("Waypoint Client Active");
}

// Called every time feedback is received for the goal
void BarakudaManager::feedbackPath(const barakuda_manager::WaypointFeedbackConstPtr& feedback)
{
  ROS_INFO("Update current path, removing a pose");

  m_current_path.poses.erase(m_current_path.poses.begin());
  ROS_INFO("Current path new size %d", int(m_current_path.poses.size()));
  updateCurrentMission(m_current_path);

  m_navPccDisplay.data = "Waypoint " + std::to_string((feedback->current_wp)+1) + " reached !";
  m_navPccDisplayPublisher.publish(m_navPccDisplay);
}

// Cancel the Path given to the Action Server
void BarakudaManager::cancelPath()
{
    // cancelMoveBase();
    wp_client.cancelGoal();
    ROS_WARN("Waypoint canceled");

    m_moveBaseCancelPublisher.publish(m_curent_move_base_goal_id);
    ROS_WARN("Move Base Goal canceled");
}

// Send Current path to the Action Server in case of iterruption
void BarakudaManager::restartPath()
{
    sendPath(m_current_path);
}

// ----- Safety Functions ----- //

// Navigation stop
void BarakudaManager::stopNavigation()
{
    ROS_WARN("-- Navigation Pause --");

    //Set Alt Stop to true
    emergencyStopClient(true);

    m_miradorStop.data == true;
    updateManagerStop();

    // Cancelling current path
    cancelPath();

    bool v_not_null = true;
    do{
        m_cmd_motors = *ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/barakuda/cmd_motors", m_node_handler);
        // ROS_WARN("cmd motors: [ %f , %f , %f , %f]",m_cmd_motors.data[0],m_cmd_motors.data[1],m_cmd_motors.data[2],m_cmd_motors.data[3]);
        float cur_sum_vel=0.;
        for (int i=0; i<4 ;i++){
            cur_sum_vel+=std::abs(m_cmd_motors.data[i]);
        }
        // ROS_INFO("sum: %f",cur_sum_vel);
        if (cur_sum_vel<=0.1){
            v_not_null=false;
        }
    }while (v_not_null);
    ROS_INFO("Release alt_stop");
    emergencyStopClient(false);
}

// Send a new path minus the reached ones
void BarakudaManager::resumeNavigation()
{
    m_miradorStop.data == false;
    updateManagerStop();

    restartPath();
}

// Update manager stop
void BarakudaManager::updateManagerStop()
{
    if((m_miradorStop.data == false) && (m_joyStop.data == false) && (m_closeRangeStop.data == false) && (m_sharkStationStop.data == false) && (m_sharkStationListening.data == true))
    {
        // Remove alt Stop from the robot
        emergencyStopClient(false);
        m_managerStop.data = false;
    }
    else
    {
        m_managerStop.data = true;
    }

    m_managerStopPublisher.publish(m_managerStop);

    updatePccDisplay();
}

// Publish string that contains informations for the PCC to display
void BarakudaManager::updatePccDisplay()
{
    std::vector<std::string> data;
    if(m_miradorStop.data == true) data.push_back("Mirador STOP");
    if (m_joyStop.data == true) data.push_back("Joy STOP");
    if (m_closeRangeStop.data == true) data.push_back("Close Range STOP (Press Y to disable)");
    if (m_sharkStationStop.data == true) data.push_back("Shark Station STOP");
    if(m_sharkStationListening.data == false) data.push_back("Shark Station NO LISTENING");
    if (!data.empty()) {
        m_stopPccDisplay.data = std::accumulate(std::next(data.begin()), data.end(), data[0],
                                                [](const std::string& a, const std::string& b) { return a + "," + b; });
    } else {
        m_stopPccDisplay.data = "";
    }
    m_stopPccDisplayPublisher.publish(m_stopPccDisplay);
}

void BarakudaManager::miradorStatusPublisher()
{
    m_miradorStatusPublisher.publish(m_status);
}
