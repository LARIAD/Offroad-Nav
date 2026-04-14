//
// Created by seb-sti1 on 14/01/2026.
//

#ifndef SRC_WRAPPERLOCALPLANNER_H
#define SRC_WRAPPERLOCALPLANNER_H

#include <mbf_gridmap_core/gridmap_controller.h>
#include <nav_core/base_local_planner.h>

namespace mbf_gridmap_costmap_wrapper {
/**
 * This class allow legacy move_base local planners to be used inside mbf_gridmap.
 *
 * It loads a nav_core::BaseLocalPlanner and create a costmap with a conversion layer, extracting obstacle data from the
 * GridMap.
 */
class WrapperLocalPlanner : public mbf_gridmap_core::GridmapController {
 public:
  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send.
   * @param pose The current pose of the robot.
   * @param velocity The current velocity of the robot.
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on ExePath action result (see ExePath.action)
   */
  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity,
                                   geometry_msgs::TwistStamped& cmd_vel, std::string& message) override;

  /**
   * @brief Check if the goal pose has been achieved by the local planner
   * @param angle_tolerance The angle tolerance in which the current pose will be partly accepted as reached goal
   * @param dist_tolerance The distance tolerance in which the current pose will be partly accepted as reached goal
   * @return True if achieved, false otherwise
   */
  bool isGoalReached(double dist_tolerance, double angle_tolerance) override;

  /**
   * @brief  Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  bool cancel() override;

  /**
   * @brief Initializes the controller plugin with a name, a tf pointer and a grid map pointer
   * @param name The controller plugin name, defined by the user. It defines the controller namespace.
   * @param tf_ptr A shared pointer to a transformation buffer
   * @param grid_map_ptr A shared pointer to a grid_map
   * @return true if the plugin has been initialized successfully
   */
  bool initialize(const std::string& name, const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                  const boost::shared_ptr<mbf_gridmap_core::GridmapHandler>& grid_map_ptr) override;

  /**
   * @brief Constructor
   */
  WrapperLocalPlanner();

  /**
   * @brief Destructor
   */
  ~WrapperLocalPlanner() override;

 private:
  //! Plugin loader for the ros1 local planner
  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> plugin_loader_;
  //! The loaded ros1 local planner plugin
  boost::shared_ptr<nav_core::BaseLocalPlanner> nav_core_plugin_;

  //! The node handle to access params
  ros::NodeHandle nh_;

  //! The gridmap pointer from mbg_gridmap
  boost::shared_ptr<mbf_gridmap_core::GridmapHandler> grid_map_ptr_;
  //! The costmap used by the ros1 local planner
  boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr_;

  //! The name of the plugin
  std::string name_;
  //! The ros1 type of the ros1 plugin to load
  std::string type_;
};
}  // namespace mbf_gridmap_costmap_wrapper

#endif  // SRC_WRAPPERLOCALPLANNER_H
