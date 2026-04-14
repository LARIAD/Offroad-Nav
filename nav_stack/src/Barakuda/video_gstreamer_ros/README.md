# video_gstreamer_ros

This small ros noetic package can create:

- a node that get ros image from a topic and send them using RTP over JPEG protocol (via a gstreamer pipeline)

## Requirements

This package needs the packages for gstreamer (see [the wiki](https://u2is.ovh/en/tech/algorithms/gst#deps)).
Additionally, it requires `ros-noetic-cv-bridge` and `ros-noetic-image-transport`.

## Configuration

See the [example.yaml](config/param_example.yaml)