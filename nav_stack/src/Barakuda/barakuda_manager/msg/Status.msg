uint8 IDLE=0
uint8 GUIDE=1
uint8 ROUTE=2
uint8 EXPLORATION=3

uint8 LANDED=0
uint8 TAKING_OFF=1
uint8 FLYING=2
uint8 LANDING=3
uint8 HOVERING=4

uint8 URL=0
uint8 WEB_RTC=1
uint8 ROS_TOPIC=2

uint8 signal_quality
barakuda_manager/GeoPose pose
uint8 mode
barakuda_manager/Mission mission
bool is_running
uint8 state_of_charge
uint8 flight_status
float32 camera_elevation
uint8 camera_zoom
bool e_stop
uint8 stream_method
string[] stream_address
string stream_topic 
