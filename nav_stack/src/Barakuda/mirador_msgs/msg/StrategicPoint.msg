string id

# Position of the point of interest
geographic_msgs/GeoPoint position

# Type of the point of interest
uint8 UNKNOWN=0
uint8 HYBRID=1
uint8 TERRESTRIAL=2
uint8 AERIAL=3
uint8 type

# The current status of the point of interest
uint8 DEFUSED=0
uint8 ACTIVE=1
uint8 UNALTERABLE=2
uint8 status

# Radius of action of the point of interest
float32 radius

# Additional comment
string message