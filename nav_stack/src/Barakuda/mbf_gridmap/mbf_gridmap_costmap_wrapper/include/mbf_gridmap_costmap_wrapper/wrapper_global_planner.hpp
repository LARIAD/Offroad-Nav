//
// Created by seb-sti1 on 20/01/2026.
//

#ifndef SRC_WRAPPERGLOBALPLANNER_H
#define SRC_WRAPPERGLOBALPLANNER_H

#include <mbf_gridmap_core/gridmap_planner.h>
#include <nav_core/base_global_planner.h>

namespace mbf_gridmap_costmap_wrapper {
/**
 * This class allow legacy move_base global planners to be used inside mbf_gridmap.
 *
 * It loads a nav_core::BaseGlobalPlanner and create a costmap with a conversion layer, extracting obstacle data from
 * the GridMap.
 */
class WrapperGlobalPlanner : public mbf_gridmap_core::GridmapPlanner {
 public:
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
   *        in x and y before failing
   * @param plan The plan filled by the planner
   * @param cost The cost of the plan
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on GetPath action result. (see GetPath.action)
   */
  uint32_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                    std::vector<geometry_msgs::PoseStamped>& plan, double& cost, std::string& message) override;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  bool cancel() override;

  /**
   * @brief Initializes the planner plugin with a user configured name and a shared pointer to the grid map
   * @param name The planner plugin name, defined by the user. It defines the planner namespace.
   * @param tf_ptr A shared pointer to a transformation buffer
   * @param grid_map_ptr A shared pointer to the grid map instance.
   * @return true if the plugin has been initialized successfully
   */
  bool initialize(const std::string& name, const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                  const boost::shared_ptr<mbf_gridmap_core::GridmapHandler>& grid_map_ptr) override;

  /**
   * @brief Constructor
   */
  WrapperGlobalPlanner();

  /**
   * @brief Destructor
   */
  ~WrapperGlobalPlanner() override;

 private:
  //! Plugin loader for the ros1 global planner
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> plugin_loader_;
  //! The loaded ros1 global planner plugin
  boost::shared_ptr<nav_core::BaseGlobalPlanner> nav_core_plugin_;

  //! The node handle to access params
  ros::NodeHandle nh_;

  //! The gridmap pointer from mbg_gridmap
  boost::shared_ptr<mbf_gridmap_core::GridmapHandler> grid_map_ptr_;
  //! The costmap used by the ros1 global planner
  boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr_;

  //! The name of the plugin
  std::string name_;
  //! The ros1 type of the ros1 plugin to load
  std::string type_;
};
}  // namespace mbf_gridmap_costmap_wrapper

#endif  // SRC_WRAPPERGLOBALPLANNER_H
