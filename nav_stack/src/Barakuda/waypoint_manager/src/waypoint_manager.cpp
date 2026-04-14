#include <ros/ros.h>
#include <math.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <waypoint_manager/WaypointAction.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>


class WaypointAction
{
protected:

    // Waypoint Action Server
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<waypoint_manager::WaypointAction> wp_server; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // Create messages that are used to published feedback/result
    waypoint_manager::WaypointFeedback feedback_;
    waypoint_manager::WaypointResult result_;

    // Move base Action Client 
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient mb_client;
    
    // Variables
    int current_wp;
    geometry_msgs::PoseStamped waypoint;
    bool success;
    float distance_tolerance;   // Max euclidian distance to WP 
    float distance_to_wp;       // Actual euclidian distance to WP 
    ros::Duration wp_timeout;
    ros::Duration wp_reach_sleep;

    std::string path_frame_id;
    std::string base_frame_id;

    tf::TransformListener listener;
    tf::StampedTransform base_in_path;

    // MB Client definition
    move_base_msgs::MoveBaseGoal mb_goal;

public:

    WaypointAction(std::string name) :
      wp_server(nh_, name, boost::bind(&WaypointAction::executeCB, this, _1), false),
      action_name_(name),
      mb_client("move_base", true) 
    { 
        // Get parameters
        ros::NodeHandle private_n("~");
        private_n.param<float>("distance_tolerance", distance_tolerance, 1);
        private_n.param<std::string>("path_frame_id", path_frame_id, "map");
        private_n.param<std::string>("base_frame_id", base_frame_id, "base_link");

        // Define some variables
        wp_timeout = ros::Duration(5.0);
        wp_reach_sleep = ros::Duration(0.5);

        // Start Action Server
        wp_server.start();

        // Create move_base Action Client object  
        ROS_INFO("Waiting for move_base action server to start ...");
        mb_client.waitForServer(ros::Duration(30));
    }

    ~WaypointAction(void)
    {
      // Cancel the current move_base Goal when the object is destroyed
      mb_client.cancelGoal();
      ROS_INFO("Action ended, current move_base goal canceled");
    }

    void executeCB(const waypoint_manager::WaypointGoalConstPtr &goal)
    {
      current_wp = 0;
      success = true;
      
      // Cancel move_base action if one is already running
      actionlib::SimpleClientGoalState mb_state = mb_client.getState();
      if(mb_state == actionlib::SimpleClientGoalState::ACTIVE) mb_client.cancelAllGoals();
      
      // Publish info to the console for the user
      ROS_INFO("%i waypoints received, starting %s action ...", int((goal->path).poses.size()), action_name_.c_str());

      // Wait for TF between Path frame and Base
      while(listener.waitForTransform(path_frame_id, base_frame_id, ros::Time::now(), ros::Duration(1.0)) == false)
      {
        ROS_WARN("Wait for the /tf between %s and %s ...", path_frame_id.c_str(), base_frame_id.c_str());
      }

      // Start executing the action while the path array is not empty
      while (current_wp != (goal->path).poses.size())
      {
        waypoint = (goal->path).poses[current_wp];

        // Check that preempt has not been requested by the client
        if (wp_server.isPreemptRequested() || !ros::ok())
        {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // Set the action state to preempted
        wp_server.setPreempted();

        // Cancel the current move_base Goal 
        ROS_INFO("%s: Cancelling the current move_base goal", action_name_.c_str());
        mb_client.cancelGoal();
        
        success = false;
        break;
        }
        
        // Fill the move_base Goal
        mb_goal.target_pose.header.frame_id = path_frame_id;
        mb_goal.target_pose.header.stamp = ros::Time::now();
        mb_goal.target_pose = waypoint;

        // Send move_base goal
        ROS_INFO("Sending move_base goal");
        mb_client.sendGoal(mb_goal);

        // Wait for precise goal reach is the tolerance is negative or it it's the last WP of the list 
        if((distance_tolerance <= 0) || ((current_wp + 1) == int((goal->path).poses.size()))){
            // Client wait for result
            ROS_INFO("Wait the robot to reach the precise Move Base Goal...");
            mb_client.waitForResult();
            ROS_INFO("...Goal Reached");
            //ROS_INFO("Wait %f sec to reach the precise Move Base Goal", wp_timeout.toSec());
            //wp_timeout.sleep();
        }
        else
        // Otherwise we check for the Euclidean distance between the robot and it's goal. 
        {
            distance_to_wp = distance_tolerance;
            // If the distance of is smaller than the tolerance, we can move to the next goal
            while(distance_to_wp >= distance_tolerance)
            {
                listener.lookupTransform(path_frame_id, base_frame_id, ros::Time(0), base_in_path);
                distance_to_wp = std::sqrt(std::pow(waypoint.pose.position.x - base_in_path.getOrigin().getX(), 2) + std::pow(waypoint.pose.position.y -  base_in_path.getOrigin().getY(), 2));
                
                // Debug to check is the robot is in the correct sequence
                ROS_INFO("%s: Distance to wp %i: %f", action_name_.c_str(), feedback_.current_wp + 1 , distance_to_wp);
                wp_reach_sleep.sleep();
            }
        }

        // Update and Publish the feedback
        feedback_.current_wp = current_wp;
        wp_server.publishFeedback(feedback_);
        ROS_INFO("%s: Feedback: %i / %i Waypoint done", action_name_.c_str(), feedback_.current_wp + 1 , int((goal->path).poses.size()));

        // Increment Waypoint
        current_wp++;
    }

    if(success)
        {
            result_.result = true;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            wp_server.setSucceeded(result_);
        }
    }  
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint");
    WaypointAction server("waypoint");
    ros::spin();
    return 0;
}

/*

rostopic pub -r 10  /imu/nav_sat_fix sensor_msgs/NavSatFix "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
status: {status: 0, service: 0}
latitude: 48.711
longitude: 2.218
altitude: 0.0
position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
position_covariance_type: 0"

*/