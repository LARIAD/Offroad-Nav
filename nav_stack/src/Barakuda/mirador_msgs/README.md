# Mirador msgs

Definition of messages to interface ROS robots with the [Mirador HMI](https://gitlab.ensta.fr/ssh/mirador)

## Description and list of available drivers

This package defines the standard messages to interact and be controled by the [Mirador HMI](https://gitlab.ensta.fr/ssh/mirador). Driver packages
for each robot need to use this messages in order to be compatible with Mirador.

Currently, the following drivers are implemented:
- Barakuda: https://gitlab.ensta.fr/ssh/barakuda_manager
- Anafi: https://gitlab.ensta.fr/ssh/anafi_mirador
- Tundra: https://gitlab.ensta.fr/ssh/tundra_mirador
- Husky: https://gitlab.ensta.fr/ssh/husky_mirador

## Dependencies and installation

### Prerequisites

First, you need [**ROS environment**](http://wiki.ros.org/fr/ROS/Installation) distro if not already installed.

Install dependencies:

```bash
sudo apt update
sudo apt-get install -y ros-noetic-geographic-msgs ros-noetic-move-base-msgs
sudo apt-get install -y libgeographic-dev geographiclib-tools
```

GeographicLib
```bash
wget https://sourceforge.net/projects/geographiclib/files/distrib-C++/GeographicLib-2.3.tar.gz && \
    tar -xvzf GeographicLib-2.3.tar.gz && \
    cd GeographicLib-2.3 && \
    mkdir BUILD && \
    cd BUILD && \
    cmake .. -DCMAKE_CXX_COMPILER=/usr/bin/g++ && \
    make && make test && make install
```

### Installation

1. Clone this repository `git clone https://gitlab.ensta.fr/ssh/mirador_msgs.git`
2. Compile using `catkin build mirador_msgs`
3. Then this package can be used like any other ROS messages package.


## Available messages

To ensure up-to-date documentation, the description of the message is provided direcly in the [msg files](./msg).