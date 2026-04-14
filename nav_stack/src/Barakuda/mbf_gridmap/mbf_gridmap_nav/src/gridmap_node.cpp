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
 */

#include <signal.h>
#include <tf2_ros/transform_listener.h>

#include "mbf_gridmap_nav/gridmap_navigation_server.h"

mbf_gridmap_nav::GridmapNavigationServer::Ptr grid_nav_srv_ptr;

void sigintHandler(int sig) {
  ROS_INFO_STREAM("Shutdown gridmap navigation server.");
  if (grid_nav_srv_ptr) {
    grid_nav_srv_ptr->stop();
  }
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mbf_grid_nav", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  double cache_time;
  private_nh.param("tf_cache_time", cache_time, 10.0);

  signal(SIGINT, sigintHandler);
  TFPtr tf_listener_ptr(new TF(ros::Duration(cache_time)));
  tf2_ros::TransformListener tf_listener(*tf_listener_ptr);

  grid_nav_srv_ptr = boost::make_shared<mbf_gridmap_nav::GridmapNavigationServer>(tf_listener_ptr);
  ros::spin();
  return EXIT_SUCCESS;
}
