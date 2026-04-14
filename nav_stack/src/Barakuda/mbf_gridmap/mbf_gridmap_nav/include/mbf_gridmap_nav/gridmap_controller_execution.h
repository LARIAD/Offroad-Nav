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

#ifndef MBF_GRIDMAP_NAV__GRIDMAP_CONTROLLER_EXECUTION_H_
#define MBF_GRIDMAP_NAV__GRIDMAP_CONTROLLER_EXECUTION_H_

#include <mbf_abstract_nav/abstract_controller_execution.h>
#include <mbf_gridmap_core/gridmap_controller.h>
#include <mbf_gridmap_nav/MoveBaseFlexConfig.h>

namespace mbf_gridmap_nav {
/**
 * @brief The GridmapControllerExecution binds a gridmap to the AbstractControllerExecution and uses the
 * mbf_gridmap_core/GridmapController class as base plugin interface.
 *
 * @ingroup controller_execution move_base_server
 */
class GridmapControllerExecution : public mbf_abstract_nav::AbstractControllerExecution {
 public:
  typedef boost::shared_ptr<mbf_gridmap_core::GridmapHandler> GridmapPtr;

  /**
   * @brief Constructor
   * @param name                The user defined name of the corresponding plugin
   * @param controller_ptr      The shared pointer to the plugin object
   * @param vel_pub             The velocity publisher for the controller execution
   * @param goal_pub            The current goal publisher for the controller execution
   * @param tf_listener_ptr     A shared pointer to the transform listener
   * @param gridmap_ptr         A pointer to the grid map object
   * @param config              The current config object
   */
  GridmapControllerExecution(const std::string& name, const mbf_gridmap_core::GridmapController::Ptr& controller_ptr,
                             const ros::Publisher& vel_pub, const ros::Publisher& goal_pub,
                             const TFPtr& tf_listener_ptr, const GridmapPtr& gridmap_ptr,
                             const MoveBaseFlexConfig& config);

  /**
   * @brief Destructor
   */
  ~GridmapControllerExecution() override = default;

 private:
  mbf_abstract_nav::MoveBaseFlexConfig toAbstract(const MoveBaseFlexConfig& config);

  const GridmapPtr& gridmap_ptr_;
};

} /* namespace mbf_gridmap_nav */

#endif /* MBF_GRIDMAP_NAV__GRIDMAP_CONTROLLER_EXECUTION_H_ */
