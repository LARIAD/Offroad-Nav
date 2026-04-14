# the packet time stamp
time stamp

# Emergency Stop
bool emergency_stop
bool emergency_stop_valid
# this should be used in priority as emergency_stop can be set to a default value (when emergency_stop_valid==false)
# this will be the last valid emergency stop value
bool last_valid_emergency_stop
# Prevent External Control
bool prevent_external_control
bool prevent_external_control_valid
# Hotswap State
int32 hotswap_state
bool hotswap_state_valid

# Range 46 - 60 V
float32 robot_battery_voltage
bool robot_battery_voltage_valid
# Range 0 - 100 %
float32 robot_battery
bool robot_battery_valid

# Range -10 - 80 °C
float32 internal_temperature
bool internal_temperature_valid
# Range -10 - 80 °C
float32 external_temperature
bool external_temperature_valid

# Range -6 - 6 m/s
float32 odom_velocity_linear_x
bool odom_velocity_linear_x_valid
# Range -3 - 3 rad/s
float32 odom_velocity_angular_z
bool odom_velocity_angular_z_valid

# Range -6 - 6 m/s
float32 cmd_vel_linear_x
bool cmd_vel_linear_x_valid
# Range -3 - 3 rad/s
float32 cmd_vel_angular_z
bool cmd_vel_angular_z_valid

# ==== For the following values, orders is always FR, FL, BR, BL ====

# Range -5300 - 5300 RPM
float32[4] motor_rpm
bool[4] motor_rpm_valid
# Range -10 - 10 Nm
float32[4] motor_torque
bool[4] motor_torque_valid
# Range 0 - 130 A
float32[4] motor_current_rms
bool[4] motor_current_rms_valid
# Range -10 - 100 °C
float32[4] motor_temperature
bool[4] motor_temperature_valid
# Range -100 - 100 rad/s
float32[4] wheel_velocity
bool[4] wheel_velocity_valid

# Range 0 - 10000 W
float32[4] controller_power
bool[4] controller_power_valid
# Range -10 - 100 °C
float32[4] controller_temperature
bool[4] controller_temperature_valid