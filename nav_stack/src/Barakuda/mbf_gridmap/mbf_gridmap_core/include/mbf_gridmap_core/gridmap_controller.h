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
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *          Sébastien Kerbourc'h <sebastien.kerbourch@ensta.fr>
 *
 */

#ifndef MBF_GRIDMAP_CORE__GRIDMAP_CONTROLLER_H_
#define MBF_GRIDMAP_CORE__GRIDMAP_CONTROLLER_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mbf_abstract_core/abstract_controller.h>
#include <tf2_ros/buffer.h>

#include <boost/shared_ptr.hpp>

#include "mbf_gridmap_core/grid_map_handler.h"

namespace mbf_gridmap_core {

/**
 * This is the abstract class for a controller plugin (or local planner plugin). Any local plugin needs to inherit this
 * class.
 *
 * All the virtual pure methods need to be implemented. The main method to implement is computeVelocityCommands where
 * using the current pose and velocity of the robot and the path (received from setPlan), a new velocity needs to be
 * computed.
 */
class GridmapController : public mbf_abstract_core::AbstractController {
 public:
  typedef boost::shared_ptr<mbf_gridmap_core::GridmapController> Ptr;

  /**
   * @brief Destructor
   */
  ~GridmapController() override = default;

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send.
   * @param pose The current pose of the robot.
   * @param velocity The current velocity of the robot.
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on ExePath action result (see ExePath.action)
   */
  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity,
                                   geometry_msgs::TwistStamped& cmd_vel, std::string& message) override = 0;

  /**
   * @brief Check if the goal pose has been achieved by the local planner
   * @param angle_tolerance The angle tolerance in which the current pose will be partly accepted as reached goal
   * @param dist_tolerance The distance tolerance in which the current pose will be partly accepted as reached goal
   * @return True if achieved, false otherwise
   */
  bool isGoalReached(double dist_tolerance, double angle_tolerance) override = 0;

  /**
   * @brief  Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override = 0;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  bool cancel() override = 0;

  /**
   * @brief Initializes the controller plugin with a name, a tf pointer and a grid map pointer
   * @param name The controller plugin name, defined by the user. It defines the controller namespace.
   * @param tf_ptr A shared pointer to a transformation buffer
   * @param grid_map_ptr A shared pointer to a grid_map
   * @return true if the plugin has been initialized successfully
   */
  virtual bool initialize(const std::string& name, const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                          const boost::shared_ptr<GridmapHandler>& grid_map_ptr) = 0;

 protected:
  /**
   * @brief Constructor
   */
  GridmapController() {};
};
}; /* namespace mbf_gridmap_core */

#endif /* MBF_GRIDMAP_CORE__GRIDMAP_CONTROLLER_H_ */
