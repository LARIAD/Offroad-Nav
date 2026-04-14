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

#ifndef MBF_GRIDMAP_NAV__GRIDMAP_RECOVERY_EXECUTION_H_
#define MBF_GRIDMAP_NAV__GRIDMAP_RECOVERY_EXECUTION_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <mbf_abstract_nav/abstract_recovery_execution.h>
#include <mbf_gridmap_core/gridmap_recovery.h>
#include <mbf_gridmap_nav/MoveBaseFlexConfig.h>

namespace mbf_gridmap_nav {

/**
 * @brief The GridmapRecoveryExecution binds a gridmap to the AbstractPlannerExecution and uses the
 * mbf_gridmap_core/GridmapRecovery class as base plugin interface.
 *
 * @ingroup planner_execution move_base_server
 */
class GridmapRecoveryExecution : public mbf_abstract_nav::AbstractRecoveryExecution {
 public:
  typedef boost::shared_ptr<mbf_gridmap_core::GridmapHandler> GridmapPtr;
  typedef boost::shared_ptr<GridmapRecoveryExecution> Ptr;

  /**
   * @brief Constructor
   * @param tf_listener_ptr Shared pointer to a common tf listener
   * @param gridmap_ptr Shared pointer to the gridmap.
   */
  GridmapRecoveryExecution(const std::string &name, const mbf_gridmap_core::GridmapRecovery::Ptr &recovery_ptr,
                           const TFPtr &tf_listener_ptr, GridmapPtr &gridmap_ptr, const MoveBaseFlexConfig &config);

  /**
   * Destructor
   */
  ~GridmapRecoveryExecution() override = default;

 protected:
  //! Shared pointer to the gridmap
  GridmapPtr &gridmap_ptr_;

 private:
  mbf_abstract_nav::MoveBaseFlexConfig toAbstract(const MoveBaseFlexConfig &config);
};

} /* namespace mbf_gridmap_nav */

#endif /* MBF_GRIDMAP_NAV__GRIDMAP_RECOVERY_EXECUTION_H_ */
