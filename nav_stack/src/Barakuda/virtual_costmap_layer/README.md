# virtual_costmap_layer

This package includes a plugin to add a virtual layer of obstacles and to define a navigation zone in the 2D costmap. 
The user can define several geometric forms (point, line, circle, polygon) in the map frame.   

1. **Point**: geometry_msgs::Point [x, y, z=0]
2. **Line**:  2 * geometry_msgs::Point[x, y, z=0]
3. **Circle**: geometry_msgs::Point[x, y, z>0]
4. **Polygon with n edges**: n * geometry_msgs::Point[x, y, z=0]

The plugin subscribes to different topics for zone and for obstacles to receive data. It is also possible to define all those forms in the config YAML file.

![Presentation](/demo/presentation.gif "Presentation")

_It seems to be a fork of https://github.com/GMahmoud/virtual_costmap_layer/tree/master_

## Config

> [!IMPORTANT]
> This code was tested on a Jura responding `ty:EF532M V02.03` `TY:` and `tl:BL_RL78 V01.31` to `TL:`.

```yaml
virtual_layer:
  # if the plugin is activated (removed see virtual_layer.cpp:285)
  enabled:              true
  # the topics to declare zone (whitelist location). Coordinates should be in the _map_frame.
  zone_topics:          [/virtual_costamp_layer/zone]
  # the topics to declare exclusion zone. Coordinates should be in the _map_frame.
  obstacle_topics:      [/virtual_costamp_layer/obsctacles]
  # if there can only be one whitelist zone
  one_zone:             true
  # define the form of the zone. Not used in this version.
  forms:                []
```