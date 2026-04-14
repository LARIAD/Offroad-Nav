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

#ifndef MBF_GRIDMAP_NAV__GRIDMAP_NAVIGATION_SERVER_H_
#define MBF_GRIDMAP_NAV__GRIDMAP_NAVIGATION_SERVER_H_

#include <grid_map_msgs/GridMap.h>
#include <mbf_abstract_nav/abstract_navigation_server.h>
#include <mbf_gridmap_core/grid_map_handler.h>
#include <mbf_msgs/CheckPath.h>
#include <mbf_msgs/CheckPose.h>
#include <std_srvs/Empty.h>

#include "gridmap_controller_execution.h"
#include "gridmap_planner_execution.h"
#include "gridmap_recovery_execution.h"
#include "mbf_gridmap_nav/MoveBaseFlexConfig.h"

namespace mbf_gridmap_nav {
/**
 * @defgroup move_base_server Move Base Server
 * @brief Classes belonging to the Move Base Server level.
 */

typedef boost::shared_ptr<dynamic_reconfigure::Server<mbf_gridmap_nav::MoveBaseFlexConfig>>
    DynamicReconfigureServerGridmapNav;

/**
 * @brief The GridmapNavigationServer combines the execution classes which use the GridmapControllerExecution,
 * GridmapPlannerExecution and the GridmapRecoveryExecution base classes as plugin interfaces. These plugin interface
 * are the very similar to the ones of move_base.
 *
 * @ingroup navigation_server move_base_server
 */
class GridmapNavigationServer : public mbf_abstract_nav::AbstractNavigationServer {
 public:
  typedef boost::shared_ptr<GridmapNavigationServer> Ptr;

  typedef boost::shared_ptr<mbf_gridmap_core::GridmapHandler> GridmapPtr;

  /**
   * @brief Constructor
   * @param tf_listener_ptr Shared pointer to a common TransformListener
   */
  explicit GridmapNavigationServer(const TFPtr& tf_listener_ptr);

  /**
   * @brief Destructor
   */
  ~GridmapNavigationServer() override;

  void stop() override;

 private:
  //! shared pointer to a new @ref planner_execution "PlannerExecution"
  mbf_abstract_nav::AbstractPlannerExecution::Ptr newPlannerExecution(
      const std::string& plugin_name, const mbf_abstract_core::AbstractPlanner::Ptr& plugin_ptr) override;

  //! shared pointer to a new @ref controller_execution "ControllerExecution"
  mbf_abstract_nav::AbstractControllerExecution::Ptr newControllerExecution(
      const std::string& plugin_name, const mbf_abstract_core::AbstractController::Ptr& plugin_ptr) override;

  //! shared pointer to a new @ref recovery_execution "RecoveryExecution"
  mbf_abstract_nav::AbstractRecoveryExecution::Ptr newRecoveryExecution(
      const std::string& plugin_name, const mbf_abstract_core::AbstractRecovery::Ptr& plugin_ptr) override;

  /**
   * @brief Loads the plugin associated with the given planner_type parameter.
   * @param planner_type The type of the planner plugin to load.
   * @return true, if the local planner plugin was successfully loaded.
   */
  mbf_abstract_core::AbstractPlanner::Ptr loadPlannerPlugin(const std::string& planner_type) override;

  /**
   * @brief Initializes the controller plugin with its name and pointer to the gridmap
   * @param name The name of the planner
   * @param planner_ptr pointer to the planner object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  bool initializePlannerPlugin(const std::string& name,
                               const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr) override;

  /**
   * @brief Loads the plugin associated with the given controller type parameter
   * @param controller_type The type of the controller plugin
   * @return A shared pointer to a new loaded controller, if the controller plugin was loaded successfully,
   *         an empty pointer otherwise.
   */
  mbf_abstract_core::AbstractController::Ptr loadControllerPlugin(const std::string& controller_type) override;

  /**
   * @brief Initializes the controller plugin with its name, a pointer to the TransformListener
   *        and pointer to the gridmap
   * @param name The name of the controller
   * @param controller_ptr pointer to the controller object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  bool initializeControllerPlugin(const std::string& name,
                                  const mbf_abstract_core::AbstractController::Ptr& controller_ptr) override;

  /**
   * @brief Loads a Recovery plugin associated with given recovery type parameter
   * @param recovery_name The name of the Recovery plugin
   * @return A shared pointer to a Recovery plugin, if the plugin was loaded successfully, an empty pointer otherwise.
   */
  mbf_abstract_core::AbstractRecovery::Ptr loadRecoveryPlugin(const std::string& recovery_name) override;

  /**
   * @brief Initializes a recovery behavior plugin with its name and pointers to the global and local gridmaps
   * @param name The name of the recovery behavior
   * @param behavior_ptr pointer to the recovery behavior object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  bool initializeRecoveryPlugin(const std::string& name,
                                const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr) override;

  /**
   * @brief Receives the new grid map from elevation mapping cuppy
   * @param new_map
   */
  void gridmapCallback(const grid_map_msgs::GridMap& new_map) const;

  /**
   * @brief Callback method for the check_pose_cost service
   * @param request Request object, see the mbf_msgs/CheckPose service definition file.
   * @param response Response object, see the mbf_msgs/CheckPose service definition file.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceCheckPoseCost(mbf_msgs::CheckPose::Request& request, mbf_msgs::CheckPose::Response& response);

  /**
   * @brief Callback method for the check_path_cost service
   * @param request Request object, see the mbf_msgs/CheckPath service definition file.
   * @param response Response object, see the mbf_msgs/CheckPath service definition file.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceCheckPathCost(mbf_msgs::CheckPath::Request& request, mbf_msgs::CheckPath::Response& response);

  /**
   * @brief Callback method for the make_plan service
   * @param request Empty request object.
   * @param response Empty response object.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceClearGridmaps(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /**
   *
   * @brief Reconfiguration method called by dynamic reconfigure.
   * @param config Configuration parameters. See the MoveBaseFlexConfig definition.
   * @param level bit mask, which parameters are set.
   */
  void reconfigure(mbf_gridmap_nav::MoveBaseFlexConfig& config, uint32_t level);

  // plugin class loaders
  pluginlib::ClassLoader<mbf_gridmap_core::GridmapPlanner> planner_plugin_loader_;
  pluginlib::ClassLoader<mbf_gridmap_core::GridmapController> controller_plugin_loader_;
  pluginlib::ClassLoader<mbf_gridmap_core::GridmapRecovery> recovery_plugin_loader_;

  //! Dynamic reconfigure server for the mbf_gridmap specific part
  DynamicReconfigureServerGridmapNav dsrv_gridmap_;

  //! last configuration save
  mbf_gridmap_nav::MoveBaseFlexConfig last_config_;

  //! the default parameter configuration save
  mbf_gridmap_nav::MoveBaseFlexConfig default_config_;

  //! Node handle to interact with ros
  ros::NodeHandle private_nh_;

  //! The subscriber to get gridmap from emc
  std::string grid_map_topic_name_;
  ros::Subscriber gridmap_sub_;

  //! Shared pointer to the gridmap
  GridmapPtr gridmap_ptr_;

  //! Service Server for the check_pose_cost service
  ros::ServiceServer check_pose_cost_srv_;

  //! Service Server for the check_path_cost service
  ros::ServiceServer check_path_cost_srv_;
};

} /* namespace mbf_gridmap_nav */

#endif /* MBF_GRIDMAP_NAV__GRIDMAP_NAVIGATION_SERVER_H_ */
