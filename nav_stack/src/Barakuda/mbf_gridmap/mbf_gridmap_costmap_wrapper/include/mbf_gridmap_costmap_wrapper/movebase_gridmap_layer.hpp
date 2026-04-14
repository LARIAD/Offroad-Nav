////  \file movebase_gridmap_layer.hpp
////  \author Poire Adrien
////  \date 09 Feb2026

#ifndef MBGRIDMAPLAYER_H
#define MBGRIDMAPLAYER_H

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <mbf_gridmap_core/grid_map_handler.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <type_traits>

#include "boost/bind.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "mbf_gridmap_costmap_wrapper/MBGridmapLayerConfig.h"
#include "mbf_costmap_nav/footprint_helper.h"

namespace mbf_gridmap_costmap_wrapper {
/**
 * This is a class inherit of costmap_2d::CostmapLayer, it is a plugin layer for costmap2dros (costmap orchestrator for
 * move_base).
 *
 * Its job is to complete, in adequacy with params (no_information_, lethal_obstacle_threshold_,
 * inscribed_inflation_obstacle_threshold_, free_space_threshold_), the costmap from grid_map layer information.
 */
class MoveBaseGridMapLayer : public costmap_2d::CostmapLayer {
 public:
  MoveBaseGridMapLayer();
  ~MoveBaseGridMapLayer() override;

  /** @brief function which get called at initialization of the costmap
   *        it defines the reconfigure callback, gets params from server and subscribes to topics
   */
  void onInitialize() override;

  /** @brief this is called by the LayeredCostmap to poll this plugin as to how much of the costmap it needs to update.
   *        each layer can increase the size of these bounds.
   */
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;

  /// @brief function which get called at every cost updating procedure of the overlay costmap. The before read
  /// costs will get filled
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

  /// @brief function to set gridmap in this layer
  void setGridmap(boost::shared_ptr<mbf_gridmap_core::GridmapHandler>& grid_map);

 private:
  /** @brief dynamic reconfiguration callback
   *
   * @param config config as define in MBGridmapLayer.cfg file.
   * @param level bit mask.
   */
  void reconfigureCb(mbf_gridmap_costmap_wrapper::MBGridmapLayerConfig& config, uint32_t level);

  /** @brief function to convert a value to a cost for costmap_2d.
   *
   * @param gridmapValue the value to convert.
   * @return value as unit8_t to feed cost map.
   */
  template <typename DataType>
  uint8_t toCostmap(DataType gridmapValue) {
    // Check special grid map values.
    if (gridmapValue == static_cast<DataType>(no_information_)) {
      return costmap_2d::NO_INFORMATION;
    } else if (gridmapValue <= static_cast<DataType>(free_space_threshold_)) {
      return costmap_2d::FREE_SPACE;
    } else if (gridmapValue >= static_cast<DataType>(lethal_obstacle_threshold_)) {
      return costmap_2d::LETHAL_OBSTACLE;
    } else if (gridmapValue >= static_cast<DataType>(inscribed_inflation_obstacle_threshold_)) {
      return costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }
    // Map gridmap interval to costmap interval for values between free space and inflated obstacle
    DataType costmapIntervalStart = costmap_2d::FREE_SPACE;
    DataType costmapIntervalWidth = costmap_2d::INSCRIBED_INFLATED_OBSTACLE - costmapIntervalStart;
    DataType gridmapIntervalStart = free_space_threshold_;
    DataType gridmapIntervalWidth = inscribed_inflation_obstacle_threshold_ - gridmapIntervalStart;
    DataType interpolatedValue =
        costmapIntervalStart + (gridmapValue - gridmapIntervalStart) * costmapIntervalWidth / gridmapIntervalWidth;

    return std::round(interpolatedValue);
  };

  void setGridmap();

  void setMethod2Conver();

  void setCostmap(costmap_2d::Costmap2D& outputCostmap);

  void tfFromOdomCallback(nav_msgs::Odometry odom_msg);

  void tfTimerCallback(const ros::TimerEvent& event);

  void simpleCopy(costmap_2d::Costmap2D& outputCostmap);

  void putGridmapInCostmap(costmap_2d::Costmap2D& outputCostmap);

  std::function<void(costmap_2d::Costmap2D&)> complete_map_;

  boost::shared_ptr<mbf_gridmap_core::GridmapHandler> grid_map_;

  // gridmap meta params
  double gridmap_resolution_;
  std::string gridmap_frame_;
  grid_map::Size gridmap_size_;
  bool grid_map_initialized_;

  // costmap meta params
  double costmap_resolution_;
  std::string costmap_frame_;
  grid_map::Size costmap_size_;
  bool costmap_initialized_;

  //! dynamic_reconfigure server for the costmap
  std::shared_ptr<dynamic_reconfigure::Server<mbf_gridmap_costmap_wrapper::MBGridmapLayerConfig>> dsrv_;

  //! name of gridmap layer that will be used
  std::string gridmap_layer_name_;

  // threshold values for convertion
  float no_information_;
  float lethal_obstacle_threshold_;
  float inscribed_inflation_obstacle_threshold_;
  float free_space_threshold_;

  //! robot frame ID use for tf listening
  std::string base_frame_;
  //! name of odom topic default not used
  std::string odom_topic_;

  //! subscriber for odom default not used
  ros::Subscriber sub_;
  //! timer for tf listening
  ros::Timer timer_;

  //! intern flag to know if costmap and gridmap are in same frame
  bool should_transform_;

  // eigen transform
  Eigen::Matrix4f tf_gridmap_frame_to_costmap_frame_;
  Eigen::Vector3f robot_translation_;
  Eigen::Matrix3f robot_rotation_matrix_;
  double robot_yaw_costmap_;

  // fotprint helper
  std::vector<mbf_costmap_nav::Cell> cells_in_footprint;
};

/**
 * @brief fuction to get specific costmap layer.
 *
 * @param layered_costmap container of all layers.
 * @param outputCostmaplayer layer found, it type should derive from costmap_2d::Layer.
 * @param layer_name name of reseached layer.
 * @param check_list_plugins optional to desplay in debug level all existing layers.
 *
 * @return true if one layer was found, else false.
 */
template <typename costmap_layer>
bool get_costmap_layer(costmap_2d::LayeredCostmap* layered_costmap,
                       boost::shared_ptr<costmap_layer>& outputCostmaplayer, std::string layer_name,
                       bool check_list_plugins = false) {
  static_assert(std::is_base_of<costmap_2d::Layer, costmap_layer>::value,
                "type parameter of this class must derive from costmap_2d::Layer");
  std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins = layered_costmap->getPlugins();
  if (check_list_plugins) {
    ROS_DEBUG_STREAM("Looking for layer " << layer_name.c_str() << " in following plugins:");
    for (const auto& plugin : *plugins) {
      ROS_DEBUG_STREAM("   " << plugin->getName().c_str() << "\n");
    }
  }
  for (auto plugin : *plugins) {
    ROS_DEBUG_STREAM("   " << plugin->getName().c_str() << "\n");
    ROS_DEBUG_STREAM("   " << layer_name.c_str() << "\n");
    if (std::strcmp(plugin->getName().c_str(), layer_name.c_str()) == 0) {
      ROS_DEBUG_STREAM("  in " << layer_name.c_str() << "\n");
      outputCostmaplayer = boost::static_pointer_cast<costmap_layer>(plugin);
      return true;
    }
  }
  ROS_DEBUG_STREAM(" end  " << layer_name.c_str() << "\n");
  return false;
};

/**
 * @brief function to set gridmap to all layer of type MoveBaseGridMapLayer.
 *
 * @param layered_costmap container of all layers.
 * @param name_space use to get param plugins of costmap_2d_ros (e.g.: "global" or "local").
 * @param grid_map_ptr shared_ptr of mbf_gridmap_core::GridmapHandler on gridmap that should be used to convert into
 * costmap.
 *
 * @return true if one layer of type MoveBaseGridMapLayer was found and feed with gridmap, else false.
 */
bool set_gridmap_to_mb_gridmap_layer(costmap_2d::LayeredCostmap* layered_costmap, std::string name_space,
                                     boost::shared_ptr<mbf_gridmap_core::GridmapHandler>& grid_map_ptr);

}  // namespace mbf_gridmap_costmap_wrapper
#endif
