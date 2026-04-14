#include"ros/ros.h"

// Basic
#include <iostream>
#include <string>
#include <ctime>
#include <fstream>

// ROS msgs
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <sbg_driver/SbgEkfNav.h>
#include <sbg_driver/SbgEkfEuler.h>

#include <waypoint_manager/WaypointAction.h>




class BarakudaLogger
{
    private:

        // ROS Topics

        // Subscribers
        ros::Subscriber m_robotStateSubscriber;
        ros::Subscriber m_recordStateSubscriber;
        ros::Subscriber m_altStopSubscriber;
        ros::Subscriber m_ekfNavSubscriber;
        ros::Subscriber m_ekfEulerSubscriber;
        ros::Subscriber m_odomglobSUB;
        ros::Subscriber m_cmdVelSubscriber;
        ros::Subscriber m_robotPoseSubscriber;
        ros::Subscriber m_GoalReachedSubscriber;
        ros::Subscriber m_currentGoal;

        // Log Variables

        // Number of log line in the file
        int m_log_num;
        std::string m_logfile_name, m_logfile_path, m_log_data;
        std::fstream m_logfile;

        // Date, Heure
        std::string m_date, m_hour;
        // Etat robot, manuel, auto, route, rejeu, retour
        // NONE = 0, TELEOPERATED = 1, AUTONOMOUS = 2, PATH = 3, RETURN = 4, REPLAY = 5, BORDER = 6
        std_msgs::UInt8 m_robot_state;
        std::string m_mode_active, m_evitement, m_suivi_bas_cote, m_rejeu, m_retour;
        // Etat points de pasage
        std::string m_etat_wp;

        // Etat enregistrement
        std_msgs::Bool m_record_state;
        std::string m_enregistrement;
        // Etat déplacement
        std_msgs::Bool m_alt_stop;
        std::string m_etat_dep;
        // Position, Vitesse linéaire
        sbg_driver::SbgEkfNav m_ekf_nav;
        nav_msgs::Odometry m_robot_pose;
	nav_msgs::Odometry m_robot_pose_global;
        std::string m_sens_long, m_northing_robot, m_easting_robot;
        // Cap, Vitesse angulaire
        sbg_driver::SbgEkfEuler m_ekf_euler;
        std::string m_sens_ang;
        std::string m_devers;
        // Twist généré par brique
        geometry_msgs::Twist m_cmd_vel;
        std::string m_cmd_long, m_cmd_ang;

    public:
        BarakudaLogger(ros::NodeHandle& n);

        // Callbacks
        void robotStateCallback(const std_msgs::UInt8& _state);
        void recordStateCallback(const std_msgs::Bool& _state);
        void altStopCallback(const std_msgs::Bool& _stop);
        void ekfNavCallback(const sbg_driver::SbgEkfNav& _ekf_nav);
        void ekfEulerCallback(const sbg_driver::SbgEkfEuler& _ekf_euler);
	void odomCallback(const nav_msgs::Odometry& _odom);
        void cmdVelCallback(const geometry_msgs::Twist& _cmd);
        void robotPoseCallback(const nav_msgs::Odometry& _odom);
        void goalReachedCallback(const std_msgs::String& _msg);
        void currentGoalCallback(const geometry_msgs::Pose& _msg);


        // Function
        void sendLog();
};
