# The quality of the signal between the robot and the mirador server
uint8 signal_quality

# The GNSS position of the robot (including altitude and heading)
mirador_msgs/GeoPose pose
geometry_msgs/Vector3 robot_angle
geometry_msgs/Twist robot_twist

# The current mode of the robot
uint8 IDLE=0
uint8 GUIDE=1
uint8 ROUTE=2
uint8 EXPLORATION=3
uint8 mode

# The current mission
mirador_msgs/Mission mission

# The robot is doing a mission
bool is_running

# Additional information regarding the mission
mirador_msgs/MissionContext mission_context
bool red_detect

# 0-100 charge of the batteries of the robot
float32[] batteries

# The current status of flight
uint8 LANDED=0
uint8 TAKING_OFF=1
uint8 FLYING=2
uint8 LANDING=3
uint8 HOVERING=4
uint8 flight_status

# camera orientation
float32 camera_pitch
float32 camera_yaw
float32 camera_zoom
float32 camera_fov

# emergency stop
bool e_stop

# the list of SRC_IP:UDP_PORT used for mirador to receive the RTP/JPEG streams
string[] stream_port
# the list of topic names to enable or disable the RTP/JPEG streams
string[] stream_enable