#include "ros/ros.h"
#include "binary_telemetry_client.h"
#include <nlohmann/json.hpp>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32MultiArray.h>
#include <barakuda_node/BarakudaStatus.h>
#include <std_srvs/SetBool.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class BarakudaDriver
{
  template <typename T>
  static inline T safeGet(const std::map<std::string, float>& telemetry, const std::string& value, const T defaultValue)
  {
    if (telemetry.count(value) > 0)
    {
      return static_cast<T>(telemetry.at(value));
    }
    return defaultValue;
  }

  void telemetryCallback(const std::map<std::string, float>& telemetry, const uint32_t packetTimestamp)
  {
    // ====== IMU ======
    if (telemetry.count("gyro-x") > 0 && telemetry.count("gyro-y") > 0 && telemetry.count("gyro-z") > 0
      && telemetry.count("roll") > 0 && telemetry.count("pitch") > 0 && telemetry.count("yaw") > 0)
    {
      sensor_msgs::Imu imu;
      imu.header.frame_id = imuFrame_;
      imu.header.stamp = ros::Time(static_cast<double>(packetTimestamp) / 1000);
      imu.angular_velocity.x = telemetry.at("gyro-x");
      imu.angular_velocity.y = telemetry.at("gyro-y");
      imu.angular_velocity.z = telemetry.at("gyro-z");

      tf2::Quaternion quat;
      quat.setRPY(telemetry.at("roll"), telemetry.at("pitch"), telemetry.at("yaw"));
      imu.orientation = tf2::toMsg(quat);
      imuPub_.publish(imu);
    }

    // ====== Status ======
    barakuda_node::BarakudaStatus status;
    status.stamp = ros::Time(static_cast<double>(packetTimestamp) / 1000);
    status.emergency_stop = safeGet(telemetry, "emergency-stop", false);
    status.emergency_stop_valid = telemetry.count("emergency-stop") > 0;
    if (status.emergency_stop_valid)
    {
      lastValidEmergencyStop_ = status.emergency_stop;
    }
    status.last_valid_emergency_stop = lastValidEmergencyStop_;
    status.prevent_external_control = safeGet(telemetry, "prevent-external-control", false);
    status.prevent_external_control_valid = telemetry.count("prevent-external-control") > 0;
    if (status.prevent_external_control_valid)
    {
      lastValidPreventExternalControl_ = status.prevent_external_control;
    }
    status.hotswap_state = safeGet(telemetry, "hotswap-state", -1);
    status.hotswap_state_valid = telemetry.count("hotswap-state") > 0;

    status.robot_battery_voltage = safeGet(telemetry, "robot-battery-voltage", -1.0);
    status.robot_battery_voltage_valid = telemetry.count("robot-battery-voltage") > 0;
    status.robot_battery = safeGet(telemetry, "robot-battery", -1.0);
    status.robot_battery_valid = telemetry.count("robot-battery") > 0;

    status.internal_temperature = safeGet(telemetry, "temp-internal", 0.0);
    status.internal_temperature_valid = telemetry.count("temp-internal") > 0;
    status.external_temperature = safeGet(telemetry, "temp-external", 0.0);
    status.external_temperature_valid = telemetry.count("temp-external") > 0;

    status.odom_velocity_linear_x = safeGet(telemetry, "velocity-linear-x", 0.0);
    status.odom_velocity_linear_x_valid = telemetry.count("velocity-linear-x") > 0;
    status.odom_velocity_angular_z = safeGet(telemetry, "velocity-angular-z", 0.0);
    status.odom_velocity_angular_z_valid = telemetry.count("velocity-angular-z") > 0;
    status.cmd_vel_linear_x = safeGet(telemetry, "cmd-vel-linear-x", 0.0);
    status.cmd_vel_linear_x_valid = telemetry.count("cmd-vel-linear-x") > 0;
    status.cmd_vel_angular_z = safeGet(telemetry, "cmd-vel-angular-z", 0.0);
    status.cmd_vel_angular_z_valid = telemetry.count("cmd-vel-angular-z") > 0;

    status.motor_rpm[0] = safeGet(telemetry, "motor-fr-rpm", 0.0);
    status.motor_rpm_valid[0] = telemetry.count("motor-fr-rpm") > 0;
    status.motor_rpm[1] = safeGet(telemetry, "motor-fl-rpm", 0.0);
    status.motor_rpm_valid[1] = telemetry.count("motor-fl-rpm") > 0;
    status.motor_rpm[2] = safeGet(telemetry, "motor-br-rpm", 0.0);
    status.motor_rpm_valid[2] = telemetry.count("motor-br-rpm") > 0;
    status.motor_rpm[3] = safeGet(telemetry, "motor-bl-rpm", 0.0);
    status.motor_rpm_valid[3] = telemetry.count("motor-bl-rpm") > 0;

    status.motor_torque[0] = safeGet(telemetry, "motor-fr-torque", 0.0);
    status.motor_torque_valid[0] = telemetry.count("motor-fr-torque") > 0;
    status.motor_torque[1] = safeGet(telemetry, "motor-fl-torque", 0.0);
    status.motor_torque_valid[1] = telemetry.count("motor-fl-torque") > 0;
    status.motor_torque[2] = safeGet(telemetry, "motor-br-torque", 0.0);
    status.motor_torque_valid[2] = telemetry.count("motor-br-torque") > 0;
    status.motor_torque[3] = safeGet(telemetry, "motor-bl-torque", 0.0);
    status.motor_torque_valid[3] = telemetry.count("motor-bl-torque") > 0;

    status.motor_current_rms[0] = safeGet(telemetry, "motor-fr-current", 0.0);
    status.motor_current_rms_valid[0] = telemetry.count("motor-fr-current") > 0;
    status.motor_current_rms[1] = safeGet(telemetry, "motor-fl-current", 0.0);
    status.motor_current_rms_valid[1] = telemetry.count("motor-fl-current") > 0;
    status.motor_current_rms[2] = safeGet(telemetry, "motor-br-current", 0.0);
    status.motor_current_rms_valid[2] = telemetry.count("motor-br-current") > 0;
    status.motor_current_rms[3] = safeGet(telemetry, "motor-bl-current", 0.0);
    status.motor_current_rms_valid[3] = telemetry.count("motor-bl-current") > 0;

    status.motor_temperature[0] = safeGet(telemetry, "motor-fr-temp", 0.0);
    status.motor_temperature_valid[0] = telemetry.count("motor-fr-temp") > 0;
    status.motor_temperature[1] = safeGet(telemetry, "motor-fl-temp", 0.0);
    status.motor_temperature_valid[1] = telemetry.count("motor-fl-temp") > 0;
    status.motor_temperature[2] = safeGet(telemetry, "motor-br-temp", 0.0);
    status.motor_temperature_valid[2] = telemetry.count("motor-br-temp") > 0;
    status.motor_temperature[3] = safeGet(telemetry, "motor-bl-temp", 0.0);
    status.motor_temperature_valid[3] = telemetry.count("motor-bl-temp") > 0;

    status.wheel_velocity[0] = safeGet(telemetry, "wheel-velocity-fr", 0.0);
    status.wheel_velocity_valid[0] = telemetry.count("wheel-velocity-fr") > 0;
    status.wheel_velocity[1] = safeGet(telemetry, "wheel-velocity-fl", 0.0);
    status.wheel_velocity_valid[1] = telemetry.count("wheel-velocity-fl") > 0;
    status.wheel_velocity[2] = safeGet(telemetry, "wheel-velocity-br", 0.0);
    status.wheel_velocity_valid[2] = telemetry.count("wheel-velocity-br") > 0;
    status.wheel_velocity[3] = safeGet(telemetry, "wheel-velocity-bl", 0.0);
    status.wheel_velocity_valid[3] = telemetry.count("wheel-velocity-bl") > 0;

    status.controller_power[0] = safeGet(telemetry, "controller-fr-power", 0.0);
    status.controller_power_valid[0] = telemetry.count("controller-fr-power") > 0;
    status.controller_power[1] = safeGet(telemetry, "controller-fl-power", 0.0);
    status.controller_power_valid[1] = telemetry.count("controller-fl-power") > 0;
    status.controller_power[2] = safeGet(telemetry, "controller-br-power", 0.0);
    status.controller_power_valid[2] = telemetry.count("controller-br-power") > 0;
    status.controller_power[3] = safeGet(telemetry, "controller-bl-power", 0.0);
    status.controller_power_valid[3] = telemetry.count("controller-bl-power") > 0;

    status.controller_temperature[0] = safeGet(telemetry, "controller-fr-temp", 0.0);
    status.controller_temperature_valid[0] = telemetry.count("controller-fr-temp") > 0;
    status.controller_temperature[1] = safeGet(telemetry, "controller-fl-temp", 0.0);
    status.controller_temperature_valid[1] = telemetry.count("controller-fl-temp") > 0;
    status.controller_temperature[2] = safeGet(telemetry, "controller-br-temp", 0.0);
    status.controller_temperature_valid[2] = telemetry.count("controller-br-temp") > 0;
    status.controller_temperature[3] = safeGet(telemetry, "controller-bl-temp", 0.0);
    status.controller_temperature_valid[3] = telemetry.count("controller-bl-temp") > 0;
    statusPub_.publish(status);

    // ====== Legacy messages ======
    std_msgs::Float32MultiArray array;
    array.data.resize(4);
    for (int i = 0; i < 4; i++)
    {
      array.data[i] = status.motor_rpm[i];
    }
    motorRPMPub_.publish(array);
    for (int i = 0; i < 4; i++)
    {
      array.data[i] = status.motor_torque[i];
    }
    motorTorquePub_.publish(array);
    for (int i = 0; i < 4; i++)
    {
      array.data[i] = status.motor_current_rms[i];
    }
    motorCurrentPub_.publish(array);
  }

  void safeSendCmd(asio::ip::udp::socket& s, std::string& req)
  {
    if (s.is_open())
    {
      s.send(asio::buffer(req));
    }
    else
    {
      ROS_WARN_STREAM("Socket is not opened! Can't send the command.");
    }
  }

  void cmdVel(geometry_msgs::Twist cmd)
  {
    if (!lastValidEmergencyStop_ && !lastValidPreventExternalControl_)
    {
      nlohmann::json req;
      req["linear"] = cmd.linear.x;
      req["angular"] = cmd.angular.z;
      std::string req_str = req.dump();
      safeSendCmd(cmdVelSock_, req_str);
    }
  }

  void cmdMotors(std_msgs::Float32MultiArray cmd)
  {
    nlohmann::json req;
    req["command_type"] = commandType_;
    req["FrontRightSpeedRequest"] = cmd.data[0];
    req["FrontLeftSpeedRequest"] = cmd.data[1];
    req["BackRightSpeedRequest"] = cmd.data[2];
    req["BackLeftSpeedRequest"] = cmd.data[3];
    std::string req_str = req.dump();
    safeSendCmd(cmdMotorsSock_, req_str);
  }

  bool stopCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    if (req.data)
    {
      ROS_WARN_STREAM("For safety reasons, until Shark patches bugs of the external command toggle, it is"
        " not allowed to deactivate the emergency stop from the driver.") ;
      res.success = false;
      return true;
    }
    else
    {
      nlohmann::json request;
      request["stop"] = true;
      std::string req_str = request.dump();
      safeSendCmd(cmdVelSock_, req_str);
      res.success = true;
      return true;
    }
  }

public:
  BarakudaDriver(ros::NodeHandle& nh) : nh_(nh), cmdVelSock_(io_), cmdMotorsSock_(io_)
  {
    // Load parameters to communicate with the robot
    nh.param<std::string>("host_ip", hostIp_, "0.0.0.0");
    nh.param<std::string>("robot_ip", robotIp_, "192.168.168.51");
    nh.param<int>("cmd_port", cmdPort_, 2422);
    nh.param<int>("wheel_port", wheelPort_, 2421);
    nh.param<int>("telemetry_api_port", telemetryAPIPort_, 8080);
    nh.param<int>("telemetry_port", telemetryPort_, 10000);
    const std::string serverUrl = "http://" + robotIp_ + ":" + std::to_string(telemetryAPIPort_);

    nh.param<std::string>("imu_frame", imuFrame_, "base_link");
    nh.param<std::string>("command_type", commandType_, "motor_rpm");

    // Load parameters related to topics
    std::string cmdVelSubTopic;
    nh.param<std::string>("cmd_vel_sub_topic", cmdVelSubTopic, "/barakuda/cmd_vel");
    std::string cmdMotorsSubTopic;
    nh.param<std::string>("cmd_motors_sub_topic", cmdMotorsSubTopic, "/barakuda/cmd_motors");
    std::string stopService;
    nh.param<std::string>("stop_service", stopService, "/barakuda/stop");

    std::string statusTopic;
    nh.param<std::string>("status_topic", statusTopic, "/barakuda/status");
    std::string imuTopic;
    nh.param<std::string>("imu_topic", imuTopic, "/barakuda/imu");
    std::string motorRPMTopic;
    nh.param<std::string>("motor_rpm_topic", motorRPMTopic, "/barakuda/motor_rpm");
    std::string motorTorqueTopic;
    nh.param<std::string>("motor_torque_topic", motorTorqueTopic, "/barakuda/motor_torque");
    std::string motorCurrentTopic;
    nh.param<std::string>("motor_current_topic", motorCurrentTopic, "/barakuda/motor_current");

    // Create inputs for the node
    cmdVelSub_ = nh.subscribe(cmdVelSubTopic, 10, &BarakudaDriver::cmdVel, this);
    cmdMotorsSub_ = nh.subscribe(cmdMotorsSubTopic, 10, &BarakudaDriver::cmdMotors, this);
    stopServiceServer_ = nh.advertiseService(stopService, &BarakudaDriver::stopCallback, this);

    // Create publishers
    statusPub_ = nh.advertise<barakuda_node::BarakudaStatus>(statusTopic, 5);
    imuPub_ = nh.advertise<sensor_msgs::Imu>(imuTopic, 5);
    motorRPMPub_ = nh.advertise<std_msgs::Float32MultiArray>(motorRPMTopic, 5);
    motorTorquePub_ = nh.advertise<std_msgs::Float32MultiArray>(motorTorqueTopic, 5);
    motorCurrentPub_ = nh.advertise<std_msgs::Float32MultiArray>(motorCurrentTopic, 5);

    // Start clients with barakuda
    ROS_INFO_STREAM("Connecting to robot interface");
    // cmd_vel
    cmdVelSock_.open(asio::ip::udp::v4());
    cmdVelSock_.connect(asio::ip::udp::endpoint(asio::ip::make_address(robotIp_), cmdPort_));
    // cmd_motors
    cmdMotorsSock_.open(asio::ip::udp::v4());
    cmdMotorsSock_.connect(asio::ip::udp::endpoint(asio::ip::make_address(robotIp_), wheelPort_));
    // telemetry
    client_ = std::make_unique<BinaryTelemetryClient>(hostIp_, telemetryPort_,
                                                      [this](const std::map<std::string, float>& d,
                                                             const uint32_t t)
                                                      {
                                                        telemetryCallback(d, t);
                                                      },
                                                      false,
                                                      serverUrl);
    clientThread_ = std::thread([this]
    {
      client_->start();
    });
    clientThread_.detach();
  }

  ~BarakudaDriver()
  {
    client_->stop();
  }

private:
  ros::NodeHandle nh_;

  // === params
  /// This is the ip of the host where the code runs
  ///   - it needs to be in the same subnetwork as the barakuda (192.168.168.*)
  ///   - it needs not to conflict with other existing ips
  std::string hostIp_;
  /// The ip of the barakuda robot. At the time of writing it is always 192.168.168.51.
  std::string robotIp_;
  /// The port on which linear/angular commands are sent. At the time of writing it is always 2422.
  int cmdPort_;
  /// The port on which per-wheel commands are sent. At the time of writing it is always 2421.
  int wheelPort_;
  /// The port on which the binary telemetry client can register/unregister. At the time of writing it is always 8080.
  int telemetryAPIPort_;
  /// The port on which the telemetry data is received. Can be customized (but should not conflict with previous ports)
  ///    and seems required to be above 10 000
  int telemetryPort_;
  /// Define the units of the per-wheel message (either motor_rpm, wheel_rad_s, wheel_linear_velocity).
  ///   See shark robotics documentation
  std::string commandType_;
  /// The frame used in the imu message
  std::string imuFrame_;

  // === saved values
  bool lastValidEmergencyStop_ = false;
  bool lastValidPreventExternalControl_ = false;

  // === inputs and outputs
  ros::Subscriber cmdVelSub_;
  ros::Subscriber cmdMotorsSub_;
  ros::ServiceServer stopServiceServer_;

  ros::Publisher imuPub_;
  ros::Publisher motorRPMPub_;
  ros::Publisher motorTorquePub_;
  ros::Publisher motorCurrentPub_;
  ros::Publisher statusPub_;

  // === robot connections
  asio::io_context io_;
  /// UDP sockets to send cmd_vel or cmd_motors commands
  asio::ip::udp::socket cmdVelSock_;
  asio::ip::udp::socket cmdMotorsSock_;
  /// binary client to get the telemetry data
  std::unique_ptr<BinaryTelemetryClient> client_;
  std::thread clientThread_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barakuda_driver");
  ros::NodeHandle nh("~");
  BarakudaDriver bd(nh);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
