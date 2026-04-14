#include "barakuda_logger.h"


BarakudaLogger::BarakudaLogger(ros::NodeHandle& n)
{
    ros::NodeHandle private_n("~");
    
    // Params

    // Init the log count at -1 so it can create a new log file
    m_log_num = -1;
    m_logfile_name = "none.txt";
    m_logfile_path = "/catkin_ws/src/barakuda_manager/log/";
    m_log_data = "";
    m_etat_wp = "";

    // Log init
    m_date = "";
    m_hour = "";
    m_mode_active = "";
    m_etat_dep = "";
    m_evitement = "";
    m_suivi_bas_cote = "";
    m_rejeu  = "";
    m_enregistrement  = "";
    m_retour = "";

    m_sens_long = "";
    m_sens_ang = "";
    m_cmd_long = "";
    m_cmd_ang = "";
    m_northing_robot = "";
    m_easting_robot = "";

    // Topics
   

    // Subscribers
    m_robotStateSubscriber = n.subscribe("/manager/robot_state", 10, &BarakudaLogger::robotStateCallback, this);
    m_recordStateSubscriber = n.subscribe("/manager/record_state", 10, &BarakudaLogger::recordStateCallback, this);
    m_altStopSubscriber = n.subscribe("/altStop", 10, &BarakudaLogger::altStopCallback, this);
    //m_ekfNavSubscriber = n.subscribe("/sbg/ekf_nav", 10, &BarakudaLogger::ekfNavCallback, this);
    //m_ekfEulerSubscriber = n.subscribe("/sbg/ekf_euler", 10, &BarakudaLogger::ekfEulerCallback, this);
    m_odomglobSUB = n.subscribe("odometry/filtered/global", 10, &BarakudaLogger::odomCallback, this);
    m_cmdVelSubscriber = n.subscribe("/cmd_vel", 10, &BarakudaLogger::cmdVelCallback, this);
    m_robotPoseSubscriber = n.subscribe("/imu/odometry", 10, &BarakudaLogger::robotPoseCallback, this);
    m_GoalReachedSubscriber = n.subscribe("/safety/nav_pcc_display", 10, &BarakudaLogger::goalReachedCallback, this);
    m_currentGoal = n.subscribe("/current_goal", 10, &BarakudaLogger::currentGoalCallback, this);

    // Publisher 

}

// Callbacks

void BarakudaLogger::robotStateCallback(const std_msgs::UInt8& _state)
{
    m_robot_state = _state;

    // Reset all states
    m_mode_active = "";
    m_evitement = "";
    m_suivi_bas_cote = "";
    m_rejeu = "";
    m_retour = "";

    switch (m_robot_state.data)
    {
    case 0:
        m_mode_active = "NONE";
        break;
    case 1:
        m_mode_active = "TELEOP";
        break;
    case 2:
        m_mode_active = "AUTO";
        m_evitement = "TRUE";
        break;
    case 3:
        m_mode_active = "AUTO";
        m_evitement = "TRUE";
        break;
    case 4:
        m_mode_active = "AUTO";
        m_evitement = "TRUE";
        m_retour = "BACKTRACK";
        break;
    case 5:
        m_mode_active = "AUTO";
        m_evitement = "TRUE";
        m_rejeu = "REPLAY";
        break;
    case 6:
        m_mode_active = "AUTO";
        m_evitement = "TRUE";
        m_suivi_bas_cote = "BORDER";
        break;
    default:
        break;
    }
}


void BarakudaLogger::recordStateCallback(const std_msgs::Bool& _state)
{
    m_record_state = _state;
    if(m_record_state.data) {m_enregistrement = "RECORD";}else{m_enregistrement = "";}
}

void BarakudaLogger::altStopCallback(const std_msgs::Bool& _stop)
{
    m_alt_stop = _stop;
    if(m_alt_stop.data) {m_etat_dep = "ALT_STOP";}
    //else{m_etat_dep = "RUN";}
}

void BarakudaLogger::ekfNavCallback(const sbg_driver::SbgEkfNav& _ekf_nav)
{
    m_ekf_nav = _ekf_nav;
    m_sens_long = std::to_string(m_ekf_nav.velocity.x);
    
}

void BarakudaLogger::ekfEulerCallback(const sbg_driver::SbgEkfEuler& _ekf_euler)
{
    m_ekf_euler = _ekf_euler;
    m_sens_ang = std::to_string(m_ekf_euler.angle.z);
    m_devers = std::to_string(m_ekf_euler.angle.y);
}

void BarakudaLogger::cmdVelCallback(const geometry_msgs::Twist& _cmd)
{
    m_cmd_vel = _cmd;

    m_cmd_long = std::to_string(_cmd.linear.x);
    m_cmd_ang = std::to_string(_cmd.angular.z);
    if ((_cmd.linear.x == 0.0) && (_cmd.angular.z == 0.0))
    {m_etat_dep = "ALT_STOP";}
    else {m_etat_dep = "RUN";}

}

void BarakudaLogger::odomCallback(const nav_msgs::Odometry& _odom){
    m_robot_pose_global = _odom;
    m_sens_long = std::to_string(m_robot_pose_global.twist.twist.linear.x);
    m_sens_ang = std::to_string(m_robot_pose_global.twist.twist.angular.z);
    m_devers = std::to_string(m_robot_pose_global.twist.twist.angular.y);
}


void BarakudaLogger::robotPoseCallback(const nav_msgs::Odometry& _odom){
    m_robot_pose = _odom;
    m_easting_robot = std::to_string(m_robot_pose.pose.pose.position.x);
    m_northing_robot = std::to_string(m_robot_pose.pose.pose.position.y);
}

void BarakudaLogger::goalReachedCallback(const std_msgs::String& _msg)
{
    if (_msg.data == "Waypoint reached, waiting goal")
    {
        m_etat_wp = "REACHED";
    }
}

void BarakudaLogger::currentGoalCallback(const geometry_msgs::Pose & _msg)
{
    m_etat_wp = std::to_string(_msg.position.x) + "," + std::to_string(_msg.position.y);
}

// Functions 

void BarakudaLogger::sendLog()
{
    // Chronologie
    time_t now = time(0);
    tm *ltm = localtime(&now);
    m_date = std::to_string(ltm->tm_mday) + "-" + std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(1900 + ltm->tm_year);
    m_hour = std::to_string(ltm->tm_hour) + ":" + std::to_string(ltm->tm_min) + ":" + std::to_string(ltm->tm_sec);
    // ROS_INFO("Date %s, Time %s", m_date.c_str() , m_hour.c_str());

    // TO DO MAX in VAR
    if(m_log_num >= 600 || m_log_num == -1)
    {
        m_log_num = 0;
        // TO DO PATH in VAR
        m_logfile_name = m_logfile_path + "taurus_logfile_" + m_date + "_" + m_hour + ".txt";
    }


	m_logfile.open(m_logfile_name, std::ios::out | std::ios::app);
	if (m_logfile) {

        // Concatenate strinf to have a CSV
        m_log_data = m_date + ";" + m_hour + ";" + m_mode_active + ";" + m_etat_dep + ";" + 
        m_evitement + ";" + m_suivi_bas_cote + ";" + 
        m_rejeu + ";" + 
        ";" + m_etat_wp + // ralliement WP
        ";" + "" + // zone exclusion
        ";" + "oui" + // Obs detect
        ";" + "" + // Statut Obs
        ";" + "" + // Distance OBS X
        ";" + "" + // Distance OBS Y
        ";" + "" + // bas-coté detect
        ";" + "" + // Distance bas-coté X
        ";" + "" + // Distance bas-coté Y
        ";" + "" + // Devers detect !!!!!!!!!!!!!!!!!!!!!!!
        m_easting_robot + ";" +  m_northing_robot + ";" + 
        //";" + "" + // Demande assistance !!!!!!!!!!!!!!!!!!!
        m_sens_long + ";" + m_sens_ang +
        ";" + "" + // Demande assistance !!!!!!!!!!!!!!!!!!!
        m_cmd_long + ";" + m_cmd_ang +
        ";" + // Commande operateur
        ";" +  ";" + // Commande type coordoénne XY
        ";" +m_etat_wp + // Coordonnées point de passage X 
        ";" +m_easting_robot + ";" +  m_northing_robot; // Position absolue ou relative (x, y) du point de départ actuel pour la fonction retour sur trace
        
        

        // Appen and lose
        m_logfile << m_log_data << std::endl;
		m_logfile.close(); 
        m_log_num++;
	}
	else {
        ROS_WARN("Can't Open the logfile !");
	}
}
