//
// Created by seb-sti1 on 20/01/2026.
//

#include "mbf_gridmap_costmap_wrapper/wrapper_global_planner.hpp"

#include <pluginlib/class_list_macros.h>

#include "mbf_gridmap_costmap_wrapper/movebase_gridmap_layer.hpp"

PLUGINLIB_EXPORT_CLASS(mbf_gridmap_costmap_wrapper::WrapperGlobalPlanner, mbf_gridmap_core::GridmapPlanner)

namespace mbf_gridmap_costmap_wrapper {
uint32_t WrapperGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                                        std::string& message) {
  bool success = nav_core_plugin_->makePlan(start, goal, plan, cost);
  message = success ? "Plan found" : "Planner failed";
  return success ? 0 : 50;
}

bool WrapperGlobalPlanner::cancel() { return false; }

bool WrapperGlobalPlanner::initialize(const std::string& name, const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
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
  costmap_ptr_ = boost::make_shared<costmap_2d::Costmap2DROS>("global_costmap", *tf_ptr);
  costmap_ptr_->pause();

  // give the grid_map to move_base costmap_2d plugin(s)
  set_gridmap_to_mb_gridmap_layer(costmap_ptr_->getLayeredCostmap(), "global_costmap", grid_map_ptr_);

  // try to load and init the ros1 global planner plugin
  try {
    nav_core_plugin_ = plugin_loader_.createInstance(type_);
    ROS_DEBUG_STREAM("nav_core global plugin " << name << " (" << type_ << ") loaded through gridmap wrapper.");
  } catch (const pluginlib::PluginlibException& e) {
    ROS_FATAL_STREAM("Failed to load nav_core global plugin " << name << " (" << type_
                                                              << ") through gridmap wrapper: " << e.what());
    return false;
  }
  nav_core_plugin_->initialize(name, &*costmap_ptr_);
  costmap_ptr_->start();
  return true;
}

WrapperGlobalPlanner::WrapperGlobalPlanner() : plugin_loader_("nav_core", "nav_core::BaseGlobalPlanner") {}

WrapperGlobalPlanner::~WrapperGlobalPlanner() { grid_map_ptr_.reset(); }

}  // namespace mbf_gridmap_costmap_wrapper