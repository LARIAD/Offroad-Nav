////  \file movebase_gridmap_layer.cpp
////  \author Poire Adrien
////  \date 09 Feb2026

#include "mbf_gridmap_costmap_wrapper/movebase_gridmap_layer.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mbf_gridmap_costmap_wrapper::MoveBaseGridMapLayer, costmap_2d::Layer);

namespace mbf_gridmap_costmap_wrapper {

MoveBaseGridMapLayer::MoveBaseGridMapLayer() : dsrv_(nullptr) {}

MoveBaseGridMapLayer::~MoveBaseGridMapLayer() {
  if (dsrv_) {
    dsrv_ = nullptr;
  }
}

void MoveBaseGridMapLayer::onInitialize() {
  ROS_DEBUG("onInitialize of MoveBaseGridMapLayer triggered");
  // init flags
  grid_map_initialized_ = false;
  costmap_initialized_ = false;
  current_ = true;
  enabled_ = true;

  // get ros handler with plugin name
  ros::NodeHandle nh("~/" + name_);

  // get ros param
  nh.param<std::string>("base_frame", base_frame_, "base_link");
  nh.param<std::string>("gridmap_layer_name", gridmap_layer_name_, "elevation");

  nh.param<float>("no_information", no_information_, -1.);
  nh.param<float>("lethal_obstacle_threshold", lethal_obstacle_threshold_, 0.30);
  nh.param<float>("inscribed_inflation_obstacle_threshold", inscribed_inflation_obstacle_threshold_, 0.29);
  nh.param<float>("free_space_threshold", free_space_threshold_, 0.15);

  nh.param<std::string>("odom_topic", odom_topic_, "");

  // init eigen params to no transform effect
  robot_translation_ << 0., 0., 0.;
  robot_rotation_matrix_.setIdentity();
  tf_gridmap_frame_to_costmap_frame_.setIdentity();

  // try to get costmap specifications
  setCostmap(*(layered_costmap_->getCostmap()));

  // start dynamic reconfigure server
  dsrv_ = std::make_shared<dynamic_reconfigure::Server<mbf_gridmap_costmap_wrapper::MBGridmapLayerConfig>>(nh);
  dsrv_->setCallback(boost::bind(&MoveBaseGridMapLayer::reconfigureCb, this, _1, _2));
  ROS_DEBUG("onInitialize of MoveBaseGridMapLayer finished");
}

void MoveBaseGridMapLayer::tfFromOdomCallback(nav_msgs::Odometry odom_msg) {
  /// Warning message frame id must be same as gridmap frame id
  if (!enabled_) return;

  const double yaw = tf2::getYaw(odom_msg.pose.pose.orientation);
  const Eigen::Vector3f d_odom_robot(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
                                     odom_msg.pose.pose.position.z);

  // set 2D rotation matrix from odom
  Eigen::Matrix3f R_robot_odom;
  R_robot_odom << static_cast<float>(cos(yaw)), static_cast<float>(-sin(yaw)), 0., static_cast<float>(sin(yaw)),
      static_cast<float>(cos(yaw)), 0., 0., 0., 1.;

  Eigen::Matrix4f trans_robot_map, trans_robot_odom_inv;
  trans_robot_odom_inv.setIdentity();
  trans_robot_map.setIdentity();

  // set homogeneous transform matrix from odom msg (odom frame to robot frame)
  trans_robot_odom_inv.block<3, 3>(0, 0) = R_robot_odom.inverse();
  trans_robot_odom_inv.block<3, 1>(0, 3) = -R_robot_odom.inverse() * d_odom_robot;

  // set homogeneous transform matrix from costmap infos (robot frame to map frame)
  trans_robot_map.block<3, 3>(0, 0) = robot_rotation_matrix_;
  trans_robot_map.block<3, 1>(0, 3) = robot_translation_;

  // combine matrix to get transform between gridmap and costmap (if all is correctly set)
  tf_gridmap_frame_to_costmap_frame_ = trans_robot_map * trans_robot_odom_inv;
}

void MoveBaseGridMapLayer::tfTimerCallback(const ros::TimerEvent& event) {
  if (!enabled_) return;
  geometry_msgs::TransformStamped gridmap_frame_tocostmap_frame_;
  try {
    gridmap_frame_tocostmap_frame_ =
        tf_->lookupTransform(costmap_frame_, gridmap_frame_, ros::Time::now(), ros::Duration(.1));
  } catch (tf2::ExtrapolationException e) {
    ROS_WARN("TF Exception between costmap and grid_map frames in %s :%s", name_.c_str(), e.what());
    return;
  } catch (tf2::InvalidArgumentException e) {
    ROS_WARN("TF Exception between costmap and grid_map frames in %s :%s", name_.c_str(), e.what());
    return;
  }

  double yaw = tf2::getYaw(gridmap_frame_tocostmap_frame_.transform.rotation);

  // set homogeneous transform matrix between gridmap and costmap (only 2D)
  tf_gridmap_frame_to_costmap_frame_ << cos(yaw), -sin(yaw), 0., gridmap_frame_tocostmap_frame_.transform.translation.x,
      sin(yaw), cos(yaw), 0., gridmap_frame_tocostmap_frame_.transform.translation.y, 0., 0., 0.,
      gridmap_frame_tocostmap_frame_.transform.translation.z, 0., 0., 0., 1.;
}

void MoveBaseGridMapLayer::reconfigureCb(mbf_gridmap_costmap_wrapper::MBGridmapLayerConfig& config, uint32_t level) {
  enabled_ = config.enabled;
  // reset gridmap layer name
  gridmap_layer_name_ = config.gridmap_layer_name;
  // reset threshold to interpret gridmap values
  no_information_ = config.no_information;
  lethal_obstacle_threshold_ = config.lethal_obstacle_threshold;
  inscribed_inflation_obstacle_threshold_ = config.inscribed_inflation_obstacle_threshold;
  free_space_threshold_ = config.free_space_threshold;
}

void MoveBaseGridMapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                        double* max_x, double* max_y) {
  if (!enabled_) return;

  robot_yaw_costmap_ = robot_yaw;
  // get robot 2d pose
  robot_translation_ << static_cast<float>(robot_x), static_cast<float>(robot_y), 0.;
  robot_rotation_matrix_ << static_cast<float>(cos(robot_yaw)), static_cast<float>(-sin(robot_yaw)), 0.,
      static_cast<float>(sin(robot_yaw)), static_cast<float>(cos(robot_yaw)), 0., 0., 0., 1.;

  // compute bounds (all the map)
  if (layered_costmap_->isRolling()) {
    *min_x = robot_x - layered_costmap_->getCostmap()->getSizeInMetersX() / 2;
    *min_y = robot_y - layered_costmap_->getCostmap()->getSizeInMetersY() / 2;
    *max_x = robot_x + layered_costmap_->getCostmap()->getSizeInMetersX() / 2;
    *max_y = robot_y + layered_costmap_->getCostmap()->getSizeInMetersY() / 2;
  } else {
    *min_x = layered_costmap_->getCostmap()->getOriginX();
    *min_y = layered_costmap_->getCostmap()->getOriginY();
    *max_x = layered_costmap_->getCostmap()->getOriginX() + layered_costmap_->getCostmap()->getSizeInMetersX();
    *max_y = layered_costmap_->getCostmap()->getOriginY() + layered_costmap_->getCostmap()->getSizeInMetersY();
  }
}

void MoveBaseGridMapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
  if (!enabled_) return;
  // call of method to complete costmap or if gridmap or costmap wasn't initialized call setCostmap
  complete_map_(master_grid);
}

void MoveBaseGridMapLayer::setCostmap(costmap_2d::Costmap2D& master_grid) {
  if (!layered_costmap_->isInitialized()) {
    ROS_WARN_STREAM("Wait init costmap in " << name_.c_str() << " plugin");
    complete_map_ = boost::bind(&MoveBaseGridMapLayer::setCostmap, this, _1);
    return;
  }

  // set costmap meta params if layered costmap is initialized
  costmap_frame_ = layered_costmap_->getGlobalFrameID();
  costmap_size_(0) = layered_costmap_->getCostmap()->getSizeInCellsX();
  costmap_size_(1) = layered_costmap_->getCostmap()->getSizeInCellsY();
  costmap_resolution_ = layered_costmap_->getCostmap()->getResolution();
  costmap_initialized_ = true;

  if (grid_map_initialized_) {
    setMethod2Conver();
  } else {
    ROS_WARN_STREAM("Wait init grid_map in " << name_.c_str() << " plugin");
    setGridmap();
  }
}

void MoveBaseGridMapLayer::setGridmap(boost::shared_ptr<mbf_gridmap_core::GridmapHandler>& grid_map) {
  grid_map_ = grid_map;
  setGridmap();
}

void MoveBaseGridMapLayer::setGridmap() {
  if (grid_map_.use_count() <= 1) {
    ROS_WARN("grid_map_handler pointer not initialised");
    return;
  }

  std::function<void(const grid_map::GridMap&)> tmp = [this](const grid_map::GridMap& in_grid_map) {
    // set costmap meta params if gridmap handler is init
    ROS_DEBUG_STREAM(this->name_.c_str() << " grid_map resolution: " << in_grid_map.getResolution());
    this->gridmap_resolution_ = in_grid_map.getResolution();
    if (in_grid_map.getResolution() == 0.0) return;

    ROS_DEBUG_STREAM(this->name_.c_str() << " grid_map frame_id: " << in_grid_map.getFrameId().c_str());
    this->gridmap_frame_ = in_grid_map.getFrameId();

    ROS_DEBUG_STREAM(this->name_.c_str() << " grid_map size: " << in_grid_map.getSize());
    this->gridmap_size_ = in_grid_map.getSize();
  };
  grid_map_->callReadingFunc(tmp);

  if (gridmap_resolution_ == 0.0) {
    ROS_WARN_STREAM(name_.c_str() << " not initialise gridmap plugin");
    return;
  }
  grid_map_initialized_ = true;

  ROS_WARN_STREAM("grid_map_handler initialised for " << this->name_.c_str());
  if (costmap_initialized_) {
    setMethod2Conver();
  }
}

void MoveBaseGridMapLayer::setMethod2Conver() {
  ROS_DEBUG("get method to complete costmap");
  // init transform strategy if frames are different
  if (gridmap_frame_ != costmap_frame_) {
    ros::NodeHandle nh("~/" + name_);
    if (!odom_topic_.empty()) {
      sub_ = nh.subscribe(odom_topic_, 1, &MoveBaseGridMapLayer::tfFromOdomCallback, this);
    } else {
      timer_ = nh.createTimer(ros::Duration(0.05), &MoveBaseGridMapLayer::tfTimerCallback, this);
    }
    should_transform_ = true;
  }

  // set method to convert gridmap into costmap (not all cases are set)
  if ((costmap_size_ == gridmap_size_).all() and not should_transform_) {
    if (costmap_resolution_ != gridmap_resolution_) {
      ROS_ERROR("Costmap should have same resolution than Gridmap to copy");
      enabled_ = false;
      return;
    }
    complete_map_ = boost::bind(&MoveBaseGridMapLayer::simpleCopy, this, _1);
  } else if (should_transform_) {
    complete_map_ = boost::bind(&MoveBaseGridMapLayer::putGridmapInCostmap, this, _1);
  } else {
    ROS_ERROR_STREAM(" [" << name_.c_str() << "] : Case not already implement: \n"
                          << " Cases:\n"
                          << "       - size of costmap = size of gridmap and express in same Frame ID\n"
                          << "       - not express in same Frame ID\n"
                          << " resolution: GM " << gridmap_resolution_ << " , CM " << costmap_resolution_ << "\n"
                          << " frame: GM " << gridmap_frame_.c_str() << " , CM " << costmap_frame_.c_str() << "\n"
                          << " size: GM " << gridmap_size_(0) << "," << gridmap_size_(1) << " , CM " << costmap_size_(0)
                          << "," << costmap_size_(1) << "\n");
  }
}

void MoveBaseGridMapLayer::simpleCopy(costmap_2d::Costmap2D& outputCostmap) {
  // do a simple copy of gridmap to costmap
  const std::function<void(const grid_map::GridMap&)> tmp = [this, &outputCostmap](const grid_map::GridMap& in_grid_map) {
    const size_t nCells = this->gridmap_size_.prod();
    for (size_t i = 0, j = nCells - 1; i < nCells; ++i, --j) {
      outputCostmap.getCharMap()[j] = this->toCostmap(in_grid_map.get(this->gridmap_layer_name_).data()[i]);
    }
  };
  grid_map_->callReadingFunc(tmp);
  cells_in_footprint = mbf_costmap_nav::FootprintHelper::getFootprintCells(
      robot_translation_[0], robot_translation_[1], robot_yaw_costmap_, layered_costmap_->getFootprint(), outputCostmap,
      true);
  for (const auto cell : cells_in_footprint) {
    outputCostmap.setCost(cell.x + 1, cell.y + 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x + 1, cell.y - 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x - 1, cell.y + 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x - 1, cell.y - 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x, cell.y + 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x, cell.y - 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x + 1, cell.y, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x - 1, cell.y, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x, cell.y, costmap_2d::FREE_SPACE);
  }
}

void MoveBaseGridMapLayer::putGridmapInCostmap(costmap_2d::Costmap2D& outputCostmap) {
  // compute transform and put gridmap values on costmap (assumed that value in gridmap are z values)
  std::function<void(const grid_map::GridMap&)> tmp = [this, &outputCostmap](const grid_map::GridMap& in_grid_map) {
    ROS_DEBUG_STREAM(this->name_.c_str() << " grid_map locked for reading...");
    Eigen::MatrixXf mat_costmap(costmap_size_(0), costmap_size_(1));
    mat_costmap.setConstant(-std::numeric_limits<float>::infinity());
    ROS_DEBUG_STREAM(this->name_.c_str() << " is updating...");
    grid_map::Position3 pos;
    for (grid_map::GridMapIterator iterator(in_grid_map); !iterator.isPastEnd(); ++iterator) {
      grid_map::Index index = iterator.getUnwrappedIndex();
      in_grid_map.getPosition3(gridmap_layer_name_, index, pos);
      Eigen::Vector4f v_odom(pos(0), pos(1), pos(2), 1.0);
      float value = in_grid_map.at(gridmap_layer_name_, index);

      // Odom vector projection to map
      Eigen::Vector4f v_map = this->tf_gridmap_frame_to_costmap_frame_ * v_odom;
      int mx, my;
      outputCostmap.worldToMapEnforceBounds(v_map[0], v_map[1], mx, my);
      mat_costmap(mx, my) = std::fmaxf(value, mat_costmap(mx, my));

      outputCostmap.setCost(mx, my, this->toCostmap(mat_costmap(mx, my)));
    }
    ROS_DEBUG_STREAM(this->name_.c_str() << " is updated");
  };
  grid_map_->callReadingFunc(tmp);
  cells_in_footprint = mbf_costmap_nav::FootprintHelper::getFootprintCells(
      robot_translation_[0], robot_translation_[1], robot_yaw_costmap_, layered_costmap_->getFootprint(), outputCostmap,
      true);
  for (const auto cell : cells_in_footprint) {
    outputCostmap.setCost(cell.x + 1, cell.y + 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x + 1, cell.y - 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x - 1, cell.y + 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x - 1, cell.y - 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x, cell.y + 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x, cell.y - 1, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x + 1, cell.y, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x - 1, cell.y, costmap_2d::FREE_SPACE);
    outputCostmap.setCost(cell.x, cell.y, costmap_2d::FREE_SPACE);
  }
}

bool set_gridmap_to_mb_gridmap_layer(costmap_2d::LayeredCostmap* layered_costmap, std::string name_space,
                                     boost::shared_ptr<mbf_gridmap_core::GridmapHandler>& grid_map_ptr) {
  XmlRpc::XmlRpcValue my_list;
  // search in param plugins parameter
  ros::NodeHandle nh_local("~/" + name_space);
  nh_local.getParam("plugins", my_list);
  bool ret = false;
  for (int32_t i = 0; i < my_list.size(); ++i) {
    std::string MB_layer = "mbf_gridmap_costmap_wrapper::MoveBaseGridMapLayer";
    std::string pname = static_cast<std::string>(my_list[i]["name"]);
    std::string type = static_cast<std::string>(my_list[i]["type"]);
    if (strcmp(type.c_str(), MB_layer.c_str()) == 0) {
      boost::shared_ptr<MoveBaseGridMapLayer> costmap_layer(new mbf_gridmap_costmap_wrapper::MoveBaseGridMapLayer);
      get_costmap_layer(layered_costmap, costmap_layer, name_space + "/" + pname, true);
      (costmap_layer)->setGridmap(grid_map_ptr);
      ret = true;
      ROS_DEBUG_STREAM("set gridmap for " << pname.c_str() << " layer");
    }
  }
  return ret;
}
}  // namespace mbf_gridmap_costmap_wrapper