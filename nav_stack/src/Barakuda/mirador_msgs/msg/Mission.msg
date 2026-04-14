Header header
string id

# The type of mission
uint8 GUIDE=1
uint8 ROUTE=2
uint8 EXPLORATION=3
uint8 type

# The key GNSS points related to this mission
geographic_msgs/GeoPoint[] points