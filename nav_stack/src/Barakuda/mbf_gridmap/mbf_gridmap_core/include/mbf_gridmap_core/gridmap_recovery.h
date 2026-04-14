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

#ifndef MBF_GRIDMAP_CORE__GRIDMAP_RECOVERY_H_
#define MBF_GRIDMAP_CORE__GRIDMAP_RECOVERY_H_

#include <tf2_ros/buffer.h>

#include <boost/shared_ptr.hpp>

#include "mbf_abstract_core/abstract_recovery.h"
#include "mbf_gridmap_core/grid_map_handler.h"

namespace mbf_gridmap_core {

/**
 * This is the abstract class for a recovery behaviors plugin. Any recovery behaviors plugin needs to inherit this
 * class.
 *
 * All the virtual pure methods need to be implemented. The main method to implement is makePlan where using the start
 * pose and the goal pose a plan needs to be generated.
 */
class GridmapRecovery : public mbf_abstract_core::AbstractRecovery {
 public:
  typedef boost::shared_ptr<mbf_gridmap_core::GridmapRecovery> Ptr;

  /**
   * @brief Runs the GridmapRecovery
   * @param message The recovery behavior could set, the message should correspond to the return value
   * @return An outcome which will be hand over to the action result.
   */
  uint32_t runBehavior(std::string& message) override = 0;

  /**
   * @brief Virtual destructor for the interface
   */
  ~GridmapRecovery() override = default;

  /**
   * @brief Requests the recovery behavior to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  bool cancel() override = 0;

  /**
   * @brief Initializes the recovery plugin with a name, a tf pointer and a grid map pointer
   * @param name The recovery behavior plugin name, defined by the user. It defines the plugins namespace
   * @param tf_ptr A shared pointer to a transformation buffer
   * @param grid_map_ptr A shared pointer to the grid map
   * @return true if the plugin has been initialized successfully
   */
  virtual bool initialize(const std::string& name, const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                          const boost::shared_ptr<GridmapHandler>& grid_map_ptr) = 0;

 protected:
  /**
   * @brief Constructor
   */
  GridmapRecovery() {};
};
}; /* namespace mbf_gridmap_core */

#endif /* MBF_GRIDMAP_CORE__GRIDMAP_RECOVERY_H_ */
