# ntrip_client


## How to use

This package is used to collect RTCM data over a NTRIP caster. The data are then send on a ROS topic (type: /rtcm_msgs/rtcm) and can be collected by a IMU driver in order to use RTK correction (see SBG_driver)

## Parameters

In order to collect those data we need a few informations about the NTRIP caster we want to connect to. For the moment we use the open Centipede NTRIP caster network (https://docs.centipede.fr/). We need information about the server IP or Adress (caster.centipede.fr), the connexion port used (2101), the nearest mountpoint that you can find on the interactive map on centipede.fr (EVCC for Palaiseau), ID and PWD (centipede/centipede) and the name of the topic where the RTCM data will be send (here: /ntrip_client/rtcm ).
