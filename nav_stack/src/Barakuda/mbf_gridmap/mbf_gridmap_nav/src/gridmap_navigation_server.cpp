/*
 *  Copyright 2026, Sebastian Pütz, Sébastien Kerbourc'h
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *           Sébastien Kerbourc'h <sebastien.kerbourch@ensta.fr>
 *
 */

#include "mbf_gridmap_nav/gridmap_navigation_server.h"

#include <grid_map_ros/GridMapRosConverter.hpp>

namespace mbf_gridmap_nav {
// TODO deals with layers of gridmap

GridmapNavigationServer::GridmapNavigationServer(const TFPtr& tf_listener_ptr)
    : AbstractNavigationServer(tf_listener_ptr),
      planner_plugin_loader_("mbf_gridmap_core", "mbf_gridmap_core::GridmapPlanner"),
      controller_plugin_loader_("mbf_gridmap_core", "mbf_gridmap_core::GridmapController"),
      recovery_plugin_loader_("mbf_gridmap_core", "mbf_gridmap_core::GridmapRecovery"),
      private_nh_("~"),
      gridmap_ptr_(boost::make_shared<mbf_gridmap_core::GridmapHandler>()) {
  ROS_DEBUG("Creating subscription and services.");
  // subscribe to grid coming from elevation mapping cupy
  private_nh_.param<std::string>("gridmap_topic", grid_map_topic_name_, "/gridmap");
  gridmap_sub_ = private_nh_.subscribe(grid_map_topic_name_, 1, &GridmapNavigationServer::gridmapCallback, this);

  // advertise services and current goal topic
  check_pose_cost_srv_ =
      private_nh_.advertiseService("check_pose_cost", &GridmapNavigationServer::callServiceCheckPoseCost, this);
  check_path_cost_srv_ =
      private_nh_.advertiseService("check_path_cost", &GridmapNavigationServer::callServiceCheckPathCost, this);

  // dynamic reconfigure server for mbf_gridmap_nav specific config
  dsrv_gridmap_ = boost::make_shared<dynamic_reconfigure::Server<mbf_gridmap_nav::MoveBaseFlexConfig>>(private_nh_);
  dsrv_gridmap_->setCallback(boost::bind(&GridmapNavigationServer::reconfigure, this, _1, _2));

  ROS_DEBUG("Finishing mbf initialisation.");

  // initialize all plugins
  initializeServerComponents();

  // start all action servers
  startActionServers();
}

void GridmapNavigationServer::gridmapCallback(const grid_map_msgs::GridMap& new_map) const {
  const std::function<void(grid_map::GridMap&)> tmp = [new_map](grid_map::GridMap& in_grid_map) {
    grid_map::GridMapRosConverter::fromMessage(new_map, in_grid_map);
  };
  gridmap_ptr_->callWritingFunc(tmp);
}

mbf_abstract_nav::AbstractPlannerExecution::Ptr GridmapNavigationServer::newPlannerExecution(
    const std::string& plugin_name, const mbf_abstract_core::AbstractPlanner::Ptr& plugin_ptr) {
  return boost::make_shared<GridmapPlannerExecution>(
      plugin_name, boost::static_pointer_cast<mbf_gridmap_core::GridmapPlanner>(plugin_ptr), boost::ref(gridmap_ptr_),
      last_config_);
}

mbf_abstract_nav::AbstractControllerExecution::Ptr GridmapNavigationServer::newControllerExecution(
    const std::string& plugin_name, const mbf_abstract_core::AbstractController::Ptr& plugin_ptr) {
  return boost::make_shared<GridmapControllerExecution>(
      plugin_name, boost::static_pointer_cast<mbf_gridmap_core::GridmapController>(plugin_ptr), vel_pub_, goal_pub_,
      tf_listener_ptr_, boost::ref(gridmap_ptr_), last_config_);
}

mbf_abstract_nav::AbstractRecoveryExecution::Ptr GridmapNavigationServer::newRecoveryExecution(
    const std::string& plugin_name, const mbf_abstract_core::AbstractRecovery::Ptr& plugin_ptr) {
  return boost::make_shared<GridmapRecoveryExecution>(
      plugin_name, boost::static_pointer_cast<mbf_gridmap_core::GridmapRecovery>(plugin_ptr), tf_listener_ptr_,
      boost::ref(gridmap_ptr_), last_config_);
}

mbf_abstract_core::AbstractPlanner::Ptr GridmapNavigationServer::loadPlannerPlugin(const std::string& planner_type) {
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr;
  ROS_DEBUG("Trying to load '%s' planner plugin.", planner_type.c_str());
  try {
    planner_ptr = planner_plugin_loader_.createInstance(planner_type);
    std::string planner_name = planner_plugin_loader_.getName(planner_type);
    ROS_INFO("'%s' planner plugin as a GridmapPlanner.", planner_type.c_str());
  } catch (const pluginlib::PluginlibException& ex) {
    ROS_WARN("Failed to load the '%s' planner as a GridmapPlanner: %s", planner_type.c_str(), ex.what());
  }
  return planner_ptr;
}

bool GridmapNavigationServer::initializePlannerPlugin(const std::string& name,
                                                      const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr) {
  mbf_gridmap_core::GridmapPlanner::Ptr gridmap_planner_ptr =
      boost::static_pointer_cast<mbf_gridmap_core::GridmapPlanner>(planner_ptr);
  ROS_DEBUG("Initializing GridmapPlanner '%s'.", name.c_str());
  gridmap_planner_ptr->initialize(name, tf_listener_ptr_, boost::ref(gridmap_ptr_));
  ROS_INFO("'%s' initialized.", name.c_str());
  return true;
}

mbf_abstract_core::AbstractController::Ptr GridmapNavigationServer::loadControllerPlugin(
    const std::string& controller_type) {
  mbf_abstract_core::AbstractController::Ptr controller_ptr;
  ROS_DEBUG("Trying to load '%s' controller plugin.", controller_type.c_str());
  try {
    controller_ptr = controller_plugin_loader_.createInstance(controller_type);
    std::string controller_name = controller_plugin_loader_.getName(controller_type);
    ROS_INFO("'%s' controller plugin as a GridmapController.", controller_type.c_str());
  } catch (const pluginlib::PluginlibException& ex) {
    ROS_WARN("Failed to load the '%s' controller as a GridmapController: %s", controller_type.c_str(), ex.what());
  }
  return controller_ptr;
}

bool GridmapNavigationServer::initializeControllerPlugin(
    const std::string& name, const mbf_abstract_core::AbstractController::Ptr& controller_ptr) {
  mbf_gridmap_core::GridmapController::Ptr gridmap_controller_ptr =
      boost::static_pointer_cast<mbf_gridmap_core::GridmapController>(controller_ptr);
  ROS_DEBUG("Initializing GridmapController '%s'.", name.c_str());
  gridmap_controller_ptr->initialize(name, tf_listener_ptr_, gridmap_ptr_);
  ROS_INFO("'%s' initialized.", name.c_str());
  return true;
}

mbf_abstract_core::AbstractRecovery::Ptr GridmapNavigationServer::loadRecoveryPlugin(const std::string& recovery_type) {
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr;
  ROS_DEBUG("Trying to load '%s' recovery plugin.", recovery_type.c_str());
  try {
    recovery_ptr = recovery_plugin_loader_.createInstance(recovery_type);
    std::string recovery_name = controller_plugin_loader_.getName(recovery_type);
    ROS_INFO("'%s' recovery plugin as a GridmapRecovery.", recovery_type.c_str());
  } catch (pluginlib::PluginlibException& ex) {
    ROS_WARN("Failed to load the '%s' recovery as a GridmapRecovery: %s", recovery_type.c_str(), ex.what());
  }
  return recovery_ptr;
}

bool GridmapNavigationServer::initializeRecoveryPlugin(const std::string& name,
                                                       const mbf_abstract_core::AbstractRecovery::Ptr& recovery_ptr) {
  mbf_gridmap_core::GridmapRecovery::Ptr gridmap_recovery_ptr =
      boost::static_pointer_cast<mbf_gridmap_core::GridmapRecovery>(recovery_ptr);
  ROS_DEBUG("Initializing GridmapRecovery '%s'.", name.c_str());
  gridmap_recovery_ptr->initialize(name, tf_listener_ptr_, boost::ref(gridmap_ptr_));
  ROS_INFO("'%s' initialized.", name.c_str());
  return true;
}

void GridmapNavigationServer::stop() { AbstractNavigationServer::stop(); }

GridmapNavigationServer::~GridmapNavigationServer() {
  planner_plugin_manager_.clearPlugins();
  controller_plugin_manager_.clearPlugins();
  recovery_plugin_manager_.clearPlugins();
  dsrv_gridmap_.reset();
  gridmap_ptr_.reset();
}

void GridmapNavigationServer::reconfigure(mbf_gridmap_nav::MoveBaseFlexConfig& config, uint32_t level) {
  {  // lock the configuration mutex when interaction with configs
    boost::unique_lock<boost::mutex> sl(configuration_mutex_);

    // Make sure we have the original configuration the first time we're called, so we can restore it if needed
    if (!setup_reconfigure_) {
      default_config_ = config;
      setup_reconfigure_ = true;
    }

    if (config.restore_defaults) {
      config = default_config_;
      // if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    last_config_ = config;
  }
  // fill the abstract configuration common to all MBF-based navigation
  mbf_abstract_nav::MoveBaseFlexConfig abstract_config;
  abstract_config.planner_frequency = config.planner_frequency;
  abstract_config.planner_patience = config.planner_patience;
  abstract_config.planner_max_retries = config.planner_max_retries;
  abstract_config.controller_frequency = config.controller_frequency;
  abstract_config.controller_patience = config.controller_patience;
  abstract_config.controller_max_retries = config.controller_max_retries;
  abstract_config.recovery_enabled = config.recovery_enabled;
  abstract_config.recovery_patience = config.recovery_patience;
  abstract_config.oscillation_timeout = config.oscillation_timeout;
  abstract_config.oscillation_distance = config.oscillation_distance;
  abstract_config.restore_defaults = config.restore_defaults;
  mbf_abstract_nav::AbstractNavigationServer::reconfigure(abstract_config, level);
}

bool GridmapNavigationServer::callServiceCheckPoseCost(mbf_msgs::CheckPose::Request& request,
                                                       mbf_msgs::CheckPose::Response& response) {
  // TODO implement
  return false;
}

bool GridmapNavigationServer::callServiceCheckPathCost(mbf_msgs::CheckPath::Request& request,
                                                       mbf_msgs::CheckPath::Response& response) {
  // TODO implement
  return false;
}

bool GridmapNavigationServer::callServiceClearGridmaps(std_srvs::Empty::Request& request,
                                                       std_srvs::Empty::Response& response) {
  // TODO implement
  return false;
}
} /* namespace mbf_gridmap_nav */