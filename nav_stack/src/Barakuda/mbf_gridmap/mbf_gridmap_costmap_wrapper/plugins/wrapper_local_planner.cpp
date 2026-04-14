//
// Created by seb-sti1 on 14/01/2026.
//

#include "mbf_gridmap_costmap_wrapper/wrapper_local_planner.hpp"

#include <pluginlib/class_list_macros.h>

#include "mbf_gridmap_costmap_wrapper/movebase_gridmap_layer.hpp"

PLUGINLIB_EXPORT_CLASS(mbf_gridmap_costmap_wrapper::WrapperLocalPlanner, mbf_gridmap_core::GridmapController)

namespace mbf_gridmap_costmap_wrapper {
uint32_t WrapperLocalPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                      const geometry_msgs::TwistStamped& velocity,
                                                      geometry_msgs::TwistStamped& cmd_vel, std::string& message) {
  bool success = nav_core_plugin_->computeVelocityCommands(cmd_vel.twist);
  message = success ? "Goal reached" : "Controller failed";
  return success ? 0 : 100;
}

bool WrapperLocalPlanner::isGoalReached(double dist_tolerance, double angle_tolerance) {
  return nav_core_plugin_->isGoalReached();
}

bool WrapperLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
  return nav_core_plugin_->setPlan(plan);
}

bool WrapperLocalPlanner::cancel() {
  ROS_WARN_STREAM(
      "The cancel method is not implemented. "
      "Note: you are running a nav_core based plugin, "
      "which is wrapped into the MBF interface.");
  return false;
}

bool WrapperLocalPlanner::initialize(const std::string& name, const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                                     const boost::shared_ptr<mbf_gridmap_core::GridmapHandler>& grid_map_ptr) {
  // save and load data/params
  grid_map_ptr_ = grid_map_ptr;
  name_ = name;
  nh_ = ros::NodeHandle("~/" + name);
  nh_.param<std::string>("plugin_type", type_, "");

  // Check if the plugin_type is properly set
  if (type_.empty()) {
    throw std::invalid_argument("The plugin_type (ros1 plugin type) is empty!");
  }

  // create costmap used by ros1 plugin
  costmap_ptr_ = boost::make_shared<costmap_2d::Costmap2DROS>("local_costmap", *tf_ptr);
  costmap_ptr_->pause();

  // give the grid_map to move_base costmap_2d plugin(s)
  set_gridmap_to_mb_gridmap_layer(costmap_ptr_->getLayeredCostmap(), "local_costmap", grid_map_ptr_);

  // try to load and init the ros1 local planner plugin
  try {
    nav_core_plugin_ = plugin_loader_.createInstance(type_);
    ROS_DEBUG_STREAM("nav_core controller plugin " << name << " (" << type_ << ") loaded through gridmap wrapper.");
  } catch (const pluginlib::PluginlibException& e) {
    ROS_FATAL_STREAM("Failed to load nav_core controller plugin " << name << " (" << type_
                                                                  << ") through gridmap wrapper: " << e.what());
    return false;
  }
  nav_core_plugin_->initialize(name, &*tf_ptr, &*costmap_ptr_);
  costmap_ptr_->start();
  return true;
}

WrapperLocalPlanner::WrapperLocalPlanner() : plugin_loader_("nav_core", "nav_core::BaseLocalPlanner") {}

WrapperLocalPlanner::~WrapperLocalPlanner() { grid_map_ptr_.reset(); }

}  // namespace mbf_gridmap_costmap_wrapper