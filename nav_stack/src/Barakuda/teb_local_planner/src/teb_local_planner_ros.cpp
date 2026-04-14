/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/algorithm/string.hpp>

// MBF return codes
#include <mbf_msgs/ExePathResult.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"


// register this planner both as a BaseLocalPlanner and as a MBF's CostmapController plugin
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, mbf_costmap_core::CostmapController)

namespace teb_local_planner
{


TebLocalPlannerROS::TebLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL),
                                           costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                           dynamic_recfg_(NULL), custom_via_points_active_(false), goal_reached_(false), nbr_infeasible_plans_(0),
                                           last_preferred_rotdir_(RotType::none), initialized_(false)
{
}


TebLocalPlannerROS::~TebLocalPlannerROS()
{
}

void TebLocalPlannerROS::reconfigureCB(TebLocalPlannerReconfigureConfig& config, uint32_t level)
{
  cfg_.reconfigure(config);
  ros::NodeHandle nh("~/" + name_);
  // lock the config mutex externally
  boost::mutex::scoped_lock lock(cfg_.configMutex());

  // create robot footprint/contour model for optimization
  cfg_.robot_model = getRobotFootprintFromParamServer(nh, cfg_);

  for (std::vector<TebOptimalPlannerPtr>::iterator it_planner = planners_.begin() ; it_planner != planners_.end(); ++it_planner)
  {
    (*it_planner)->updateRobotModel(cfg_.robot_model);
  }
}

void TebLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // check if the plugin is already initialized
  if(!initialized_)
  {
    name_ = name;
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle nh("~/" + name);

    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_.loadRosParamFromNodeHandle(nh);

    // Register topic to publish log (string) messages to the GUI
    log_pub_ = nh.advertise<std_msgs::String>("/safety/nav_pcc_display", 100);

    /* DEPRECATED
    // reserve some memory for obstacles
    //obstacles_.reserve(500);
    */

    // create visualization instance
    // TODO: How does it behaves if all instances of planners have the same ?
    //       If I use a instance per planner, do we have conflits on topics ?
    //visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_));

    // create robot footprint/contour model for optimization
    cfg_.robot_model = getRobotFootprintFromParamServer(nh, cfg_);

    if (cfg_.hcp.enable_homotopy_class_planning) ROS_INFO("Parallel planning in distinctive topologies enabled.");
    else ROS_INFO("Parallel planning in distinctive topologies disabled.");

    // Before creating planners on each layer with different velocities, ensure layers names and layers max velocities have same size
    if ((cfg_.obstacle_layers_names).size() != (cfg_.layers_max_vel).size())
    {
      ROS_ERROR("Please configure as many max velocities as layers to use.");
      // TODO: choose default values for velocities if not enough? Truncate if too many?
    }

    // Iterate on the vector of max velocities, as they will be used for configuring each planner (and as there must be as many max vel as layers)
    for (std::vector<double>::iterator it_vel = (cfg_.layers_max_vel).begin() ; it_vel != (cfg_.layers_max_vel).end(); ++it_vel)
    {
      // Create a new obstacles container and reserve memory for this layer
      //ObstContainer new_obst_container;
      //new_obst_container.reserve(500);
      boost::shared_ptr<ObstContainer> new_obstacle_container_ptr(new ObstContainer);
      new_obstacle_container_ptr->reserve(500);
      obstacles_.push_back(new_obstacle_container_ptr);

      // Adapt cfg_ max vel parameters according to the one mentioned for the current layer
      cfg_.robot.max_vel_x = *it_vel;

      // Create a visualization instance for this planner
      TebVisualizationPtr new_visualization_ptr(new TebVisualization(nh, cfg_));
      visualizations_.push_back(new_visualization_ptr); 
      // Create a planner instance for this layer. For easier development, only TebOptimalPlanner is implemented yet
      planners_.push_back(TebOptimalPlannerPtr(new TebOptimalPlanner(cfg_, &(*(obstacles_.back())), visualizations_.back(), &via_points_)));
    }

    // init other variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    /*
    // DEPRECATED
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

    // Instead of calling getCostmap() method to get the stacked costmap, select just specific layer(s) and store it/them
    for (std::vector<std::string>::iterator it = (cfg_.obstacle_layers_names).begin() ; it != (cfg_.obstacle_layers_names).end(); ++it)
    {
      costmap_2d::Costmap2D* new_layer_ptr = extract_layer(*it);
      costmaps_.push_back(new_layer_ptr);
      costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*new_layer_ptr);
    }
    */
    costmaps_.push_back(costmap_ros_->getCostmap());
    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*(costmap_ros_->getCostmap()));
    global_frame_ = costmap_ros_->getGlobalFrameID();
    cfg_.map_frame = global_frame_; // TODO
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    //Initialize as many costmap-to-polygon converters as layers considered
    if (!cfg_.obstacles.costmap_converter_plugin.empty())
    {
      // If one of the converters fails to load, come back to point obstacles solution
      try
      {
        // Increment both costmaps vector and name vector iterators (using std::pair) to access them parallelly
        /* NOT WORKING
        for (std::vector<costmap_2d::Costmap2D*>::iterator it_map = costmaps_.begin(), std::vector<std::string>::iterator it_name = (cfg_.obstacle_layers_names).begin() ; 
        	it_map != costmaps_.end(); ++it_map, ++it_name)
        */
        for (std::pair<std::vector<costmap_2d::Costmap2D*>::iterator, std::vector<std::string>::iterator> it(costmaps_.begin(), (cfg_.obstacle_layers_names).begin());
             it.first != costmaps_.end() /* && it.second != (cfg_.obstacle_layers_names).end() */ ;
             ++it.first, ++it.second)
        {
	  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> new_costmap_converter = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
	  std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin) + "/" + it.second->c_str();
	  // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
          boost::replace_all(converter_name, "::", "/");  
          new_costmap_converter->setOdomTopic(cfg_.odom_topic);
          new_costmap_converter->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
          new_costmap_converter->setCostmap2D(*(it.first));
          new_costmap_converter->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), *(it.first), cfg_.obstacles.costmap_converter_spin_thread);
          ROS_INFO_STREAM("Costmap conversion plugin " << converter_name.c_str() << " loaded.");  
          costmap_converters_.push_back(new_costmap_converter);         
        }      
      }
      catch(pluginlib::PluginlibException& ex)
      {
        // In case of error, reset all converters
        ROS_WARN("One of the specified costmap converter plugins cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        for (std::vector<boost::shared_ptr<costmap_converter::BaseCostmapToPolygons>>::iterator it = costmap_converters_.begin() ; it != costmap_converters_.end(); ++it)
        {
          it->reset();
        }
      }
    }
    else 
      ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");


    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);    
    
    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // setup dynamic reconfigure
    dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(nh);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&TebLocalPlannerROS::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);
    
    // validate optimization footprint and costmap footprint
    validateFootprints(cfg_.robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);
        
    // setup callback for custom obstacles
    custom_obst_sub_ = nh.subscribe("obstacles", 1, &TebLocalPlannerROS::customObstacleCB, this);

    // setup callback for custom via-points
    via_points_sub_ = nh.subscribe("via_points", 1, &TebLocalPlannerROS::customViaPointsCB, this);
    
    // initialize failure detector
    ros::NodeHandle nh_move_base("~");
    double controller_frequency = 5;
    nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
    failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration*controller_frequency));
    
    // set initialized flag
    initialized_ = true;

    ROS_DEBUG("teb_local_planner plugin initialized.");
  }
  else
  {
    ROS_WARN("teb_local_planner has already been initialized, doing nothing.");
  }
}



bool TebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.  

  // reset goal_reached_ flag
  goal_reached_ = false;

  return true;
}


bool TebLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
  uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;
  return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t TebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                     const geometry_msgs::TwistStamped& velocity,
                                                     geometry_msgs::TwistStamped &cmd_vel,
                                                     std::string &message)
{
  // check if plugin initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    message = "teb_local_planner has not been initialized";
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  static uint32_t seq = 0;
  cmd_vel.header.seq = seq++;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;  
  
  // Get robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  robot_pose_ = PoseSE2(robot_pose.pose);
    
  // Get robot velocity
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);

  // prune global plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);
  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::TransformStamped tf_plan_to_global;
  
  // Note: costmap argument is used only to access window size for transformation, so we can use any layer in it
  if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *(costmaps_.back()), global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist, 
                           transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    message = "Could not transform the global plan to the frame of the controller";
    return mbf_msgs::ExePathResult::INTERNAL_ERROR;
  }

  // update via-points container
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);

  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);

  // check if global goal is reached
  geometry_msgs::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
  double dx = global_goal.pose.position.x - robot_pose_.x();
  double dy = global_goal.pose.position.y - robot_pose_.y();
  double delta_orient = g2o::normalize_theta( tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta() );
  if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance
    && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
    && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0)
    && (base_local_planner::stopped(base_odom, cfg_.goal_tolerance.theta_stopped_vel, cfg_.goal_tolerance.trans_stopped_vel)
        || cfg_.goal_tolerance.free_goal_vel))
  {
    goal_reached_ = true;
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);
  
    
  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
    message = "Transformed plan is empty";
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }

  // Get current goal point (last point of the transformed plan)
  robot_goal_.x() = transformed_plan.back().pose.position.x;
  robot_goal_.y() = transformed_plan.back().pose.position.y;
  // Overwrite goal orientation if needed
  
  if (cfg_.trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    tf2::convert(q, transformed_plan.back().pose.orientation);
  }  
  else
  {
    robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
  }
  
  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
  }
  transformed_plan.front() = robot_pose; // update start
  
  // clear currently existing obstacles    
  for (std::vector<boost::shared_ptr<ObstContainer>>::iterator it_obs = obstacles_.begin() ; it_obs != obstacles_.end() ; ++it_obs)
  {
    (*it_obs)->clear();
  } 
  //obstacles_.clear();
  
  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
  if (!costmap_converters_.empty())
    updateObstacleContainerWithCostmapConverter();
  else
    updateObstacleContainerWithCostmap();

  // also consider custom obstacles (must be called after other updates, since the container is not cleared)
  updateObstacleContainerWithCustomObstacles();
  
    
  // Do not allow config changes during the following optimization step
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());
    
  // Now perform the actual planning
//   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel); // straight line init

  bool success = false;
  for (std::vector<TebOptimalPlannerPtr>::iterator it_planner = planners_.begin() ; it_planner != planners_.end() ; ++it_planner)
  {
    success |= (*it_planner)->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
  }


  if (!success)
  {
    // force reinitialization for next time
    for (std::vector<TebOptimalPlannerPtr>::iterator it_planner = planners_.begin() ; it_planner != planners_.end() ; ++it_planner)
    {
      (*it_planner)->clearPlanner();
    }
    ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting in any layer.");
    
    ++nbr_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner was not able to obtain a local plan for the current setting in any layer.";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  /* Select best planner strategy */
  // Define local variables
  double min_cost = std::numeric_limits<double>::max(); // maximum cost
  double min_cost_last_best = std::numeric_limits<double>::max();

  // check if last best planner is still a valid candidate
  if (current_best_)
  {
    // get cost of this candidate
    min_cost_last_best = (last_best_->getCurrentCost()) * cfg_.hcp.selection_cost_hysteresis; // small hysteresis
    last_best_ = current_best_;
  }
  else
  {
    last_best_.reset();
  }

  current_best_.reset(); // reset pointer
  best_index_ = -1;

  // Define counters to be able to check if all planners diverged or planned unfeasible trajectories
  //unsigned int divergence_counter, unfeasible_counter, invalid_command_counter, unfinite_steering_counter;
  //divergence_counter = unfeasible_counter = invalid_command_counter = unfinite_steering_counter = planners_.size();
  unsigned int fail_counter = planners_.size();
  
  // Consult results on each planner
  for (std::vector<TebOptimalPlannerPtr>::iterator it_planner = planners_.begin(); it_planner != planners_.end(); ++it_planner)
  {
    // First, check for divergence
    if ((*it_planner)->hasDiverged())
    {
      // Reset everything to start again with the initialization of a new trajectory.
      (*it_planner)->clearPlanner();
      ROS_WARN_THROTTLE(1.0, "TebLocalPlannerROS: One of the trajectories has diverged. Resetting corresponding planner...");
      //--divergence_counter;
      --fail_counter;
            
      if (fail_counter <= 0) // = If all previous planners have failed and this one diverges
      {
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        ++nbr_infeasible_plans_; // increase number of infeasible solutions in a row
        time_last_infeasible_plan_ = ros::Time::now();
        last_cmd_ = cmd_vel.twist;
        ROS_WARN("All planners failed (divergence)");
        return mbf_msgs::ExePathResult::NO_VALID_CMD;    	 
      }
      
      // This planner has failed. Skip to next one.
      continue;
    }
    // Check feasibility (but within the first few states only)
    if(cfg_.robot.is_footprint_dynamic)
    {
      // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
      footprint_spec_ = costmap_ros_->getRobotFootprint();
      costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
    }

    bool feasible = (*it_planner)->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses, cfg_.trajectory.feasibility_check_lookahead_distance);
    if (!feasible)
    {
        // now we reset everything to start again with the initialization of a trajectory.
        (*it_planner)->clearPlanner();
        ROS_WARN("TebLocalPlannerROS: One of the trajectories is not feasible. Resetting corresponding planner...");
        //--unfeasible_counter;
        --fail_counter;
        
        if (fail_counter <= 0) // = If all previous planners have failed and this one gives an unfeasible trajectory
        {
          cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
          ++nbr_infeasible_plans_; // increase number of infeasible solutions in a row
          time_last_infeasible_plan_ = ros::Time::now();
          last_cmd_ = cmd_vel.twist;
          ROS_WARN("All planners failed (trajectory not feasible)");
          message = "teb_local_planner trajectory is not feasible";
          return mbf_msgs::ExePathResult::NO_VALID_CMD; 
        }
        
        // This planner has failed. Skip to next one.
        continue;
    }    
    // Design choice: checking command validity of this planner first and then compare cost to the others before choosing the velocity command to keep
    // Thus we prefer storing potential velocity commands in temporary variables
    double tmp_lin_x, tmp_lin_y, tmp_ang_z;
    
    // Get the velocity command for this sampling interval
    bool command_valid = (*it_planner)->getVelocityCommand(tmp_lin_x, tmp_lin_y, tmp_ang_z, cfg_.trajectory.control_look_ahead_poses);
    if (!command_valid)
    {
      (*it_planner)->clearPlanner();
      ROS_WARN("TebLocalPlannerROS: A velocity command is invalid in one planner. Resetting corresponding planner...");
      //--invalid_command_counter;
      --fail_counter;
      
      if (fail_counter <= 0) // = If all previous planners have failed and this one gives unvalid command
      {
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        ++nbr_infeasible_plans_; // increase number of infeasible solutions in a row
        time_last_infeasible_plan_ = ros::Time::now();
        last_cmd_ = cmd_vel.twist;
        ROS_WARN("All planners failed (invalid command)");
        message = "A velocity command is invalid in one planner";
        return mbf_msgs::ExePathResult::NO_VALID_CMD;
      }
      
      // This planner has failed. Skip to next one.
      continue;
    }
    else // = if command valid
    {
      // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
      saturateVelocity(tmp_lin_x, tmp_lin_y, tmp_ang_z,
                   cfg_.robot.max_vel_x, cfg_.robot.max_vel_y, cfg_.robot.max_vel_trans, cfg_.robot.max_vel_theta, 
                   cfg_.robot.max_vel_x_backwards);
    
      // convert rot-vel to steering angle if desired (carlike robot).
      // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
      // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
      if (cfg_.robot.cmd_angle_instead_rotvel)
      {
        tmp_ang_z = convertTransRotVelToSteeringAngle(tmp_lin_x, tmp_lin_y, cfg_.robot.wheelbase, 0.95*cfg_.robot.min_turning_radius);
        if (!std::isfinite(tmp_ang_z))
        {
          (*it_planner)->clearPlanner();
          ROS_WARN("TebLocalPlannerROS: One of the resulting steering angle is not finite. Resetting corresponding planner...");
          --fail_counter;
          
          if (fail_counter <= 0) // = If all previous planners have failed and this one gives a not finite steering angle
          {
            cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
            last_cmd_ = cmd_vel.twist;

            ++nbr_infeasible_plans_; // increase number of infeasible solutions in a row
            time_last_infeasible_plan_ = ros::Time::now();
            ROS_WARN("All planners failed (steering angle not finite)");
            message = "teb_local_planner steering angle is not finite";
            return mbf_msgs::ExePathResult::NO_VALID_CMD;
          }
          
         // This planner has failed. Skip to next one.
         continue;
        } // end if steering angle is not finite 
      }
      
      // If we reach this point, all conditions of errors should be checked. We can now compare costs.
      double planner_cost;
      planner_cost = (*it_planner)->getCurrentCost();

      // If this planner gives a better result, keep velocity commands
      if (planner_cost < min_cost)
      {
        cmd_vel.twist.linear.x = tmp_lin_x;
        cmd_vel.twist.linear.y = tmp_lin_y;
        cmd_vel.twist.angular.z = tmp_ang_z;

        current_best_ = *it_planner;

        (*it_planner)->visualize();

        best_index_ = it_planner - planners_.begin();
        min_cost = planner_cost;
      }

      // check if we are allowed to change
      if (last_best_ && current_best_ != last_best_)
      {
        ros::Time now = ros::Time::now();
        if ((now-last_switching_time_).toSec() > cfg_.hcp.switching_blocking_period)
        {
          last_switching_time_ = now;
        }
        else
        {
          ROS_INFO("Switching between layers classes blocked (check parameter switching_blocking_period.");
          // block switching, so revert best_teb_
          current_best_ = last_best_;
        }
      }
    }
  } // end for loop on planners

  if (!current_best_)
  {
    ROS_ERROR("Couldn't retrieve the best plan");
    return mbf_msgs::ExePathResult::NO_VALID_CMD; 
  }

  // a feasible solution should be found, reset counter
  nbr_infeasible_plans_ = 0;
  
  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel.twist;
  
  // Debug purpose
  ROS_INFO("Using planner on %s layer", (cfg_.obstacle_layers_names[best_index_]).c_str());

  // For the moment, we choose to visualize only what is actually used for the robot (ie. current best planner info)   
  current_best_->visualize();

  visualizations_[best_index_]->publishObstacles((*(obstacles_[best_index_])), costmaps_[best_index_]->getResolution());
  visualizations_[best_index_]->publishViaPoints(via_points_);
  visualizations_[best_index_]->publishGlobalPlan(global_plan_);
  return mbf_msgs::ExePathResult::SUCCESS;
}


bool TebLocalPlannerROS::isGoalReached()
{
  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    std_msgs::String mirador_log;
    mirador_log.data = "Waypoint reached, waiting goal";
    log_pub_.publish(mirador_log); // TODO: check if a ros::spinOnce() is needed, and if it could disturb something in local_planner/move_base...
    for (std::vector<TebOptimalPlannerPtr>::iterator it_planner = planners_.begin() ; it_planner != planners_.end(); ++it_planner)
    {
      (*it_planner)->clearPlanner();
    }  
    return true;
  }
  return false;
}


void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
{  
  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles)
  {
    for (std::pair<std::vector<costmap_2d::Costmap2D*>::iterator, std::vector<boost::shared_ptr<ObstContainer>>::iterator> it(costmaps_.begin(), obstacles_.begin());
             it.first != costmaps_.end() /* && it.second != obstacles_.end() */ ;
             ++it.first, ++it.second)
    {
      Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();
    
      for (unsigned int i=0; i<(*(it.first))->getSizeInCellsX()-1; ++i)
      {
        for (unsigned int j=0; j<(*(it.first))->getSizeInCellsY()-1; ++j)
        {
          if ((*it.first)->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
          {
            Eigen::Vector2d obs;
            (*it.first)->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
            
            // check if obstacle is interesting (e.g. not far behind the robot)
            Eigen::Vector2d obs_dir = obs-robot_pose_.position();
            if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist  )
              continue;
            
            (*(it.second))->push_back(ObstaclePtr(new PointObstacle(obs)));
          }
        }
      }
    }
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
  if (costmap_converters_.empty())
    return;
    
  for (std::pair<std::vector<boost::shared_ptr<costmap_converter::BaseCostmapToPolygons>>::iterator, std::vector<boost::shared_ptr<ObstContainer>>::iterator> it(costmap_converters_.begin(), obstacles_.begin());
       it.first != costmap_converters_.end() /* && it.second != obstacles_.end() */ ;
       ++it.first, ++it.second)
  {
    //Get obstacles from costmap converter
    costmap_converter::ObstacleArrayConstPtr obstacles = (*(it.first))->getObstacles();
    // If no obstacle in this layer, spik to next one
    if (!obstacles)
      continue;

    for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
    {
      const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
      const geometry_msgs::Polygon* polygon = &obstacle->polygon;

      if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
      {
        (*(it.second))->push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
      }
      else if (polygon->points.size()==1) // Point
      {
        (*(it.second))->push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
      }
      else if (polygon->points.size()==2) // Line
      {
        (*(it.second))->push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y )));
      }
      else if (polygon->points.size()>2) // Real polygon
      {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (std::size_t j=0; j<polygon->points.size(); ++j)
        {
          polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        (*(it.second))->push_back(ObstaclePtr(polyobst));
      }

      // Set velocity, if obstacle is moving
      if(!(*(it.second))->empty())
        (*(it.second))->back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
    }
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
  // Add custom obstacles obtained via message
  boost::mutex::scoped_lock l(custom_obst_mutex_);

  if (!custom_obstacle_msg_.obstacles.empty())
  {
    // We only use the global header to specify the obstacle coordinate system instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try 
    {
      geometry_msgs::TransformStamped obstacle_to_map =  tf_->lookupTransform(global_frame_, ros::Time(0),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Time(0),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));
      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      obstacle_to_map_eig.setIdentity();
    }
    
    for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
    {
      for (std::vector<boost::shared_ptr<ObstContainer>>::iterator it_obs = obstacles_.begin() ; it_obs != obstacles_.end() ; ++it_obs)
      {
        if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
        {
          Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
          (*it_obs)->push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
        }
        else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
        {  
          Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
          (*it_obs)->push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
        }
        else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
        {
          Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                      custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                      custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
          Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
          (*it_obs)->push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
                                                           (obstacle_to_map_eig * line_end).head(2) )));
        }
        else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
        {
          ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
          // TODO: We don't need to publish it as many times as the number of layers...
          continue;
        }
        else // polygon
        {
          PolygonObstacle* polyobst = new PolygonObstacle;
          for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
          {
            Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                                 custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                                 custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
            polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
          }
          polyobst->finalizePolygon();
          (*it_obs)->push_back(ObstaclePtr(polyobst));
        }

        // Set velocity, if obstacle is moving
        if(!(*it_obs)->empty())
          (*it_obs)->back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
      }
    }
  }
}

void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();
  
  if (min_separation<=0)
    return;
  
  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;
        
    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }
  
}



Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
{
  Eigen::Vector2d vel;
  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
  return vel;
}
      
    
bool TebLocalPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}
      

bool TebLocalPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                  std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }
   
    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                                  plan_pose.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));
    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id,ros::Duration(cfg_.robot.transform_tolerance));

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap
    

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    
    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
      }
    }

    geometry_msgs::PoseStamped newer_pose;
    
    double plan_length = 0; // check cumulative Euclidean distance along the plan
    
    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      
      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }

    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);
      
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }
    
    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex) 
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) 
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}

   
double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
              int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();
  
  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
      tf2::Quaternion global_orientation;
      tf2::convert(global_plan.back().pose.orientation, global_orientation);
      tf2::Quaternion rotation;
      tf2::convert(tf_plan_to_global.transform.rotation, rotation);
      // TODO(roesmann): avoid conversion to tf2::Quaternion
      return tf2::getYaw(rotation *  global_orientation);
    }     
  }
  
  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
  
  std::vector<double> candidates;
  geometry_msgs::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::PoseStamped tf_pose_kp1;
  
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);

    // calculate yaw angle  
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
        tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
    
    if (i<range_end-1) 
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}
  
void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_trans, double max_vel_theta, 
              double max_vel_x_backwards) const
{
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  // Limit translational velocity for forward driving
  if (vx > max_vel_x)
    ratio_x = max_vel_x / vx;
  
  // limit strafing velocity
  if (vy > max_vel_y || vy < -max_vel_y)
    ratio_y = std::abs(max_vel_y / vy);
  
  // Limit angular velocity
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);
  
  // Limit backwards velocity
  if (max_vel_x_backwards<=0)
  {
    ROS_WARN_ONCE("TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  }
  else if (vx < -max_vel_x_backwards)
    ratio_x = - max_vel_x_backwards / vx;

  if (cfg_.robot.use_proportional_saturation)
  {
    double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
    vx *= ratio;
    vy *= ratio;
    omega *= ratio;
  }
  else
  {
    vx *= ratio_x;
    vy *= ratio_y;
    omega *= ratio_omega;
  }

  double vel_linear = std::hypot(vx, vy);
  if (vel_linear > max_vel_trans)
  {
    double max_vel_trans_ratio = max_vel_trans / vel_linear;
    vx *= max_vel_trans_ratio;
    vy *= max_vel_trans_ratio;
  }
}
     

double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
{
  if (omega==0 || v==0)
    return 0;
    
  double radius = v/omega;
  
  if (fabs(radius) < min_turning_radius)
    radius = double(g2o::sign(radius)) * min_turning_radius; 

  return std::atan(wheelbase / radius);
}
     

void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}



void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx)
{
    ros::Time current_time = ros::Time::now();
    
    // reduced horizon backup mode
    if (cfg_.recovery.shrink_horizon_backup && 
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (nbr_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        ROS_INFO_COND(nbr_infeasible_plans_==1, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.recovery.shrink_horizon_min_duration);


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;
        
        if (nbr_infeasible_plans_ > 9)
        {
            ROS_INFO_COND(nbr_infeasible_plans_==10, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;
        }
        
        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-) 
    }
    
    
    // detect and resolve oscillations
    if (cfg_.recovery.oscillation_recovery)
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_.robot.max_vel_x : cfg_.robot.max_vel_x_backwards;
        if (cfg_.robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_.robot.min_turning_radius),  cfg_.robot.max_vel_theta );
        else
            max_vel_theta = cfg_.robot.max_vel_theta;
        
        failure_detector_.update(last_cmd_, cfg_.robot.max_vel_x, cfg_.robot.max_vel_x_backwards, max_vel_theta,
                               cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);
        
        bool oscillating = true;//failure_detector_.isOscillating();
        bool recently_oscillated = (ros::Time::now()-time_last_oscillation_).toSec() < cfg_.recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently
        
        if (oscillating)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                ROS_WARN("TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = ros::Time::now();  
            
            // TODO: Check if setting preferred turing dir on current planner (ie. current_best_) is sufficient
            if (current_best_)
            {
              current_best_->setPreferredTurningDir(last_preferred_rotdir_);
            }
            
        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            if (current_best_)
            {
              current_best_->setPreferredTurningDir(last_preferred_rotdir_);
              ROS_INFO("TebLocalPlannerROS: oscillation recovery disabled/expired.");
            }
        }
    }

}

 
void TebLocalPlannerROS::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;  
}


void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  if (cfg_.trajectory.global_plan_viapoint_sep > 0)
  {
    ROS_WARN("Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
             "Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  boost::mutex::scoped_lock l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  custom_via_points_active_ = !via_points_.empty();
}
     

RobotFootprintModelPtr TebLocalPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle& nh, const TebConfig& config)
{
  std::string model_name; 
  if (!nh.getParam("footprint_model/type", model_name))
  {
    ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
    return boost::make_shared<PointRobotFootprint>();
  }
    
  // point  
  if (model_name.compare("point") == 0)
  {
    ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
    return boost::make_shared<PointRobotFootprint>(config.obstacles.min_obstacle_dist);
  }
  
  // circular
  if (model_name.compare("circular") == 0)
  {
    // get radius
    double radius;
    if (!nh.getParam("footprint_model/radius", radius))
    {
      ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/radius' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
    return boost::make_shared<CircularRobotFootprint>(radius);
  }
  
  // line
  if (model_name.compare("line") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get line coordinates
    std::vector<double> line_start, line_end;
    nh.getParam("footprint_model/line_start", line_start);
    nh.getParam("footprint_model/line_end", line_end);
    if (line_start.size() != 2 || line_end.size() != 2)
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
    ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
                     << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
    return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()), config.obstacles.min_obstacle_dist);
  }
  
  // two circles
  if (model_name.compare("two_circles") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") 
        || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
    {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius 
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }

  // polygon
  if (model_name.compare("polygon") == 0)
  {

    // check parameters
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) )
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get vertices
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      try
      {
        Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
        ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
        return boost::make_shared<PolygonRobotFootprint>(polygon);
      } 
      catch(const std::exception& ex)
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
    }
    else
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
  }
  
  // otherwise
  ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
  return boost::make_shared<PointRobotFootprint>();
}
       
       
Point2dContainer TebLocalPlannerROS::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name)
{
   // Make sure we have an array of at least 3 elements.
   if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       footprint_xmlrpc.size() < 3)
   {
     ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
     throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                              "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
   }
 
   Point2dContainer footprint;
   Eigen::Vector2d pt;
 
   for (int i = 0; i < footprint_xmlrpc.size(); ++i)
   {
     // Make sure each element of the list is an array of size 2. (x and y coordinates)
     XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
     if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         point.size() != 2)
     {
       ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                 "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                  full_param_name.c_str());
       throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x() = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y() = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

double TebLocalPlannerROS::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
     throw std::runtime_error("Values in the footprint specification must be numbers");
   }
   return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

costmap_2d::Costmap2D* TebLocalPlannerROS::extract_layer(std::string layer_name)
{
  if (!costmap_ros_)
  {
    ROS_WARN("costmap_ros_ not initialized, could not load %s layer from the Costmap2DROS", layer_name.c_str());
    return nullptr;
  }
  
  costmap_2d::Costmap2D* costmap=NULL;
  std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins = costmap_ros_->getLayeredCostmap()->getPlugins();

  // Search for the plugin corresponding to the name layer_name
  for ( std::vector<boost::shared_ptr<costmap_2d::Layer>>::iterator pluginp=plugins->begin(); pluginp != plugins->end();++pluginp)
  {
    boost::shared_ptr<costmap_2d::Layer> plugin=*pluginp;
    if (strcmp(plugin->getName().c_str(), layer_name.c_str()) == 0)
    {
      boost::shared_ptr<costmap_2d::CostmapLayer> local_handler(boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin));
      costmap = local_handler.get();
      costmap->resizeMap(costmap_ros_->getCostmap()->getSizeInCellsX(),costmap_ros_->getCostmap()->getSizeInCellsY(), costmap_ros_->getCostmap()->getResolution(),
                          costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getOriginY());
      break;
    }
  }
  ROS_INFO("Layer %s loaded from the Costmap2DROS", layer_name.c_str());
  
  return costmap;
}


} // end namespace teb_local_planner


